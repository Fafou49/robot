"""Software-PWM motor driver -- the single owner of the two DC motors'
GPIO lines on this robot.

Refactored (2026-09-07) out of two previously separate, duplicate, and
partly-buggy copies of this exact logic:
  - motor_control/pwm.py: a blocking script that opened the GPIO chip and
    read stdin as a side effect of being imported -- not an importable
    function, and never actually wired to anything (see the "STP"/"DRV"
    notes this replaces in link/robot_state.py's module docstring).
    Archived to archive/pwm_2026.py; superseded by this module.
  - motor_control/remote_control.py's Remote.pwm() method: a working
    copy of the same algorithm, but private to that class and only ever
    driven by its own gamepad-reading loop.

Having exactly one class own the GPIO chip matters for real: gpiod line
requests are exclusive, so two objects (in one process or two separate
ones) both requesting the same output lines fail or fight each other.
Now link/server.py (TCP commands + link/gamepad_handler.py's manual
joystick driving, both feeding link/robot_state.py's RobotState) and
motor_control/remote_control.py's standalone Remote (used directly by the
motor_control/gps_log_on_*.py field-test scripts, no TCP server involved)
both create their OWN MotorDriver instance -- but never at the same time
in the same process, and never sharing a chip, so there's no conflict in
practice; the shared code here just means one algorithm to trust instead
of two.

Honesty note (same caveat as link/gps_reader.py): gpiod could not be
installed in the sandbox this was refactored in (no PyPI access there),
so this was rewritten carefully against gpiod v2's documented API and
cross-checked line-for-line against remote_control.py's own already-
working (real-hardware-tested, per this repo's README) version -- but the
refactor itself, as a whole, has NOT been re-run against real hardware.
Run it for real on the Pi (motor_control/remote_control.py's field-test
scripts, or link/server.py with a gamepad/TCP client) before trusting it
further than "the pure duty-cycle-to-line-state logic is unit-tested".
"""
import logging
import threading

from motor_control.gpiochip import detect_rp1_gpiochip

log = logging.getLogger("motor_control.motor_driver")

try:
    import gpiod
    from gpiod.line import Direction, Value
    _GPIOD_AVAILABLE = True
    _GPIOD_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover -- exercised whenever gpiod
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    gpiod = None
    Direction = None
    Value = None
    _GPIOD_AVAILABLE = False
    _GPIOD_IMPORT_ERROR = exc

# Pin numbers as confirmed working in motor_control/remote_control.py
# against real hardware (per this repo's README's 2026-09-06 bug-fix
# history, and reconfirmed 2026-09-07 -- see below). Note that
# motor_control/pwm.py (now archived to archive/pwm_2026.py, and never
# actually wired to anything real) had MOTOR1_SENS1/MOTOR1_SENS2 the
# other way around (15/14 instead of 14/15) -- that discrepancy was just
# a leftover from an untested file, not a hint of a wiring inversion to
# reproduce.
#
# Wiring compensation, confirmed 2026-09-07: the two drive motors are
# mounted mirrored on the chassis (one on each side) and, BY DESIGN,
# wired with reversed polarity relative to each other so a single motor
# model can be used on both sides. That compensation is already fully
# handled in the PHYSICAL WIRING (which motor lead is connected to which
# H-bridge terminal) -- confirmed by the 2026-09-05/06 field test
# (motor_control/gps_log_on_full_throttle.py, both motors driven at the
# SAME PWM sign, +255/+255): the robot already drove straight, not in a
# circle. Because of that, drive()/the software-PWM loop below
# deliberately do NOT apply any additional sign inversion between
# left_pwm and right_pwm -- doing so on top of an already-compensated
# wiring would UNDO the compensation and make the robot pivot instead of
# drive straight. If a motor or its wiring is ever replaced and this
# stops being true, the fix belongs on the bench (re-wire, or literally
# swap SENS1/SENS2 below for the affected motor) -- not as a runtime
# software flag, precisely because getting a software-level "which side
# is inverted" flag wrong here is what would silently break already-
# working, already-field-tested driving.
MOTOR1_SENS1 = 14  # left motor, forward
MOTOR1_SENS2 = 15  # left motor, reverse
MOTOR2_SENS1 = 2   # right motor, forward
MOTOR2_SENS2 = 3   # right motor, reverse

PWM_STEPS = 255   # software-PWM resolution -- matches the -255..255 duty-cycle range
PWM_DEADZONE = 20  # |duty| at or below this is treated as "stopped" (both lines inactive)


def _sense_line_values(duty_cycle, counter, deadzone=PWM_DEADZONE):
    """Pure step of the software-PWM algorithm, no hardware involved:
    given one motor's duty cycle (-255..255) and where a 0..254 counter
    currently is within one PWM period, returns (sens1_active,
    sens2_active) as booleans for that motor's two direction/enable
    lines. A duty cycle whose magnitude is at or below `deadzone` is
    treated as neutral (both lines inactive), same dead zone the
    pre-refactor code (motor_control/pwm.py, motor_control/
    remote_control.py) already used. Kept standalone so the actual duty-
    cycle math can be unit-tested without a real (or even stubbed)
    gpiod.Chip."""
    if duty_cycle > deadzone:
        return (counter <= duty_cycle, False)
    if duty_cycle < -deadzone:
        return (False, counter <= abs(duty_cycle))
    return (False, False)


class MotorDriver:
    """Owns the GPIO chip and runs the software-PWM loop in a background
    thread. drive(left, right) just updates two ints under a lock --
    cheap, non-blocking, safe to call from any thread (link/server.py's
    TCP handlers, link/gamepad_handler.py's gamepad thread, or
    motor_control/remote_control.py's Remote) any number of times per
    second; the loop thread re-reads them once per PWM period.

    Degrades gracefully (logs a warning, stays inert) if gpiod isn't
    installed or the chip/lines can't be opened -- same pattern as
    link/gps_reader.py for a missing/absent GPS receiver -- so a dev
    machine, or a Pi with the motor driver board unplugged, can still run
    link/server.py (or a gps_log_on_*.py script) for everything else;
    drive() just becomes a no-op as far as anything physical goes."""

    def __init__(self, gpiochip=None):
        self._lock = threading.Lock()
        self.left_pwm = 0
        self.right_pwm = 0
        self._running = False
        self._lines = None
        self._gpiochip_path = gpiochip or detect_rp1_gpiochip()

    def start(self):
        """Opens the GPIO chip and starts the PWM loop thread. Returns
        the Thread on success, or None if motors are disabled (gpiod
        missing, or the chip/lines couldn't be requested) -- callers
        don't need to check the return value, drive() stays safe to call
        either way."""
        if not _GPIOD_AVAILABLE:
            log.warning(
                "gpiod not installed (%s) -- motor driver disabled, DRV/RTE/gamepad "
                "commands will be accepted and validated but nothing will physically "
                "move. Run `pip install -r requirements.txt` on the Pi to enable it.",
                _GPIOD_IMPORT_ERROR,
            )
            return None

        try:
            chip = gpiod.Chip(self._gpiochip_path)
            self._lines = chip.request_lines(
                consumer="motor_driver",
                config={
                    MOTOR1_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT),
                    MOTOR1_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT),
                    MOTOR2_SENS1: gpiod.LineSettings(direction=Direction.OUTPUT),
                    MOTOR2_SENS2: gpiod.LineSettings(direction=Direction.OUTPUT),
                },
            )
        except Exception as exc:  # chip/lines busy, no such device, permission...
            log.warning(
                "could not open GPIO chip %s for the motor driver (%s) -- motors "
                "disabled, same graceful degradation as a missing GPS receiver.",
                self._gpiochip_path, exc,
            )
            return None

        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        log.info("motor driver started on %s", self._gpiochip_path)

        # Last-resort safety net: if the process is killed uncleanly
        # (crash, unhandled exception, Ctrl+C racing the normal cleanup
        # path), this still gets a chance to zero the outputs before the
        # interpreter actually exits, so a crash never leaves a motor
        # spinning at whatever duty cycle it last had.
        import atexit
        atexit.register(self._safe_stop)
        return thread

    def drive(self, left_pwm, right_pwm):
        """Sets the target duty cycles (-255..255, not re-validated here
        -- callers like link/robot_state.py's drive() already validate
        the range and raise CommandError for anything out of bounds).
        Safe to call even if start() was never called or failed -- just
        updates state nothing reads yet."""
        with self._lock:
            self.left_pwm = left_pwm
            self.right_pwm = right_pwm

    def stop(self):
        self.drive(0, 0)

    def shutdown(self):
        """Stops the PWM loop thread. Not called anywhere in normal
        operation (link/server.py's background threads, like
        link/gps_reader.py's, are daemon threads that simply die with the
        process) -- provided mainly so tests can start() a driver, drive
        it briefly, and cleanly join the thread afterwards instead of
        leaving a stray thread running for the rest of the test process."""
        self._running = False

    def _safe_stop(self):
        try:
            if self._lines is not None:
                self._set_lines(False, False, False, False)
        except Exception:
            # Runs during interpreter shutdown (atexit) -- raising here
            # would just hide whatever the real problem was, and there's
            # no one left to report it to anyway.
            pass

    def _set_lines(self, m1s1, m1s2, m2s1, m2s2):
        self._lines.set_value(MOTOR1_SENS1, Value.ACTIVE if m1s1 else Value.INACTIVE)
        self._lines.set_value(MOTOR1_SENS2, Value.ACTIVE if m1s2 else Value.INACTIVE)
        self._lines.set_value(MOTOR2_SENS1, Value.ACTIVE if m2s1 else Value.INACTIVE)
        self._lines.set_value(MOTOR2_SENS2, Value.ACTIVE if m2s2 else Value.INACTIVE)

    def _loop(self):
        while self._running:
            with self._lock:
                left, right = self.left_pwm, self.right_pwm
            # Duty cycles are snapshotted once per PWM period (not once
            # per counter tick) -- a new drive() call takes effect within
            # one period (at PWM_STEPS=255 iterations of 4 GPIO writes
            # each, a period is short enough for this to feel instant to
            # an operator, and avoids taking the lock 255x as often for
            # no real benefit).
            for counter in range(PWM_STEPS):
                if not self._running:
                    return
                l1, l2 = _sense_line_values(left, counter)
                r1, r2 = _sense_line_values(right, counter)
                self._set_lines(l1, l2, r1, r2)
