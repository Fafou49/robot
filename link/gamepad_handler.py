"""Generic Xbox-controller reader, shared by the two things in this repo
that need one: link/server.py (feeds a live link.robot_state.RobotState,
wired into the TCP command protocol) and motor_control/remote_control.py's
Remote class (a standalone tool with no RobotState/TCP server involved at
all, used directly by the motor_control/gps_log_on_*.py field-test
scripts). Both used to have their own separate copy of this exact
gamepad-reading logic; mutualized here (2026-09-07) into one evdev-based
implementation driven purely by callbacks, so the two consumers differ
only in what they DO with an axis change or a button press, never in how
one is read off the hardware.

Why evdev over pygame (remote_control.py's previous choice -- still
listed in requirements.txt, now otherwise unused): pygame's joystick
module needs SDL's video subsystem initialized just to read a gamepad,
which can fail in a headless session (SSH, no display) unless
SDL_VIDEODRIVER=dummy is set; evdev reads /dev/input/eventX directly, has
no display dependency, and delivers real button-press EVENTS (a clean
0->1 transition) instead of a "currently held" level the caller has to
diff against the previous frame itself. It also avoids hardcoding a
device path like /dev/input/event5 (remote_control.py's previous
approach, which breaks the day the controller re-enumerates under a
different event number) -- devices are found by capability instead.

Honesty note (same caveat as link/gps_reader.py and motor_control.
motor_driver): evdev could not be installed in the sandbox this was
written in (no PyPI access there), so this was written carefully against
its documented public API (list_devices(), InputDevice, capabilities(),
read_loop(), the AbsInfo namedtuple) but has NOT been run against a real
controller. Run it for real on the Pi before trusting it further than
"the pure axis/PWM math is unit-tested, and the no-controller-found path
degrades without crashing".
"""
import logging
import threading
import time

log = logging.getLogger("link.gamepad_handler")

try:
    import evdev
    from evdev import ecodes
    _EVDEV_AVAILABLE = True
    _EVDEV_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover -- exercised whenever evdev
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    evdev = None
    ecodes = None
    _EVDEV_AVAILABLE = False
    _EVDEV_IMPORT_ERROR = exc

# Xbox controller stick axes, as exposed by the Linux kernel's xpad driver
# via evdev. Names (not raw numbers) so they can be looked up on the real
# `ecodes` module at run time -- overridable per-instance (see
# GamepadReader.__init__) in case a particular controller/driver
# combination maps these differently.
DEFAULT_LEFT_Y_CODE = "ABS_Y"
DEFAULT_RIGHT_Y_CODE = "ABS_RY"

AXIS_DEADZONE = 0.08     # normalized (-1..1) stick movement below this counts as centered
PWM_SCALE = 255          # matches link/robot_state.py's PWM_MIN/PWM_MAX
RETRY_INTERVAL_S = 5.0   # how often to re-scan for a controller if none is found/it disconnects


def _normalize_axis(raw_value, info_min, info_max):
    """Maps a raw evdev ABS event value onto -1.0..1.0, centered on the
    middle of THIS controller's own reported range. Deliberately does not
    assume a fixed range like -32768..32767 -- unlike pygame (which
    always normalizes to -1..1 itself), evdev reports whatever range the
    specific device/driver combination declares, found via
    device.capabilities(absinfo=True)."""
    center = (info_max + info_min) / 2
    half_range = (info_max - info_min) / 2
    if half_range == 0:
        return 0.0
    value = (raw_value - center) / half_range
    return max(-1.0, min(1.0, value))


def _pwm_from_axis(normalized, deadzone=AXIS_DEADZONE, scale=PWM_SCALE):
    """Stick pushed away from center -> motor PWM (-scale..scale), with a
    dead zone around center so small analog noise doesn't produce a tiny
    nonzero duty cycle. Sign is inverted: evdev (like the SDL/pygame
    convention remote_control.py's previous code relied on) reports a
    stick pushed UP/forward as a NEGATIVE Y value -- this flips it so
    "forward" maps to positive PWM, matching link/robot_state.py's DRV
    convention."""
    if abs(normalized) < deadzone:
        return 0
    return int(round(-normalized * scale))


class GamepadReader:
    """Finds an Xbox-style controller and reads it forever, either in a
    background thread (start(), used by link/server.py, whose main thread
    is busy serving TCP connections) or in the calling thread
    (run_blocking(), used by motor_control/remote_control.py's Remote,
    which has always blocked the whole script this way).

    Two axes (left/right stick Y) call on_drive(left_pwm, right_pwm)
    whenever either one changes. Every button press AND release calls
    on_button(code, pressed) with the raw evdev key code (e.g.
    evdev.ecodes.BTN_A) -- callers that only care about presses just
    check `if pressed`."""

    def __init__(self, on_drive, on_button=None, device_path=None,
                 left_y_code=DEFAULT_LEFT_Y_CODE, right_y_code=DEFAULT_RIGHT_Y_CODE,
                 retry_interval=RETRY_INTERVAL_S):
        self.on_drive = on_drive
        self.on_button = on_button or (lambda code, pressed: None)
        self.device_path = device_path
        self.left_y_code = left_y_code
        self.right_y_code = right_y_code
        self.retry_interval = retry_interval
        self._running = False
        self._left_norm = 0.0
        self._right_norm = 0.0

    def start(self):
        """Starts reading in a background daemon thread."""
        thread = threading.Thread(target=self.run_blocking, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def run_blocking(self):
        """Same loop as start(), but runs in the calling thread. Retries
        device discovery every `retry_interval` seconds for as long as no
        suitable controller is found, and again if one disconnects mid-
        session -- so plugging the controller in after this has already
        started still works, and unplugging it doesn't crash whatever
        called this (link/server.py's whole control server, or a
        gps_log_on_*.py script)."""
        self._running = True
        if not _EVDEV_AVAILABLE:
            log.warning(
                "evdev not installed (%s) -- gamepad disabled, no joystick/button "
                "input will reach the robot. Run `pip install -r requirements.txt` "
                "on the Pi to enable it.",
                _EVDEV_IMPORT_ERROR,
            )
            return

        while self._running:
            device = self._find_device()
            if device is None:
                log.warning(
                    "no gamepad found (looked for a device exposing an absolute "
                    "axis and BTN_A) -- retrying in %ss. Plug in the Xbox controller.",
                    self.retry_interval,
                )
                time.sleep(self.retry_interval)
                continue

            log.info("gamepad connected: %s (%s)", device.name, device.path)
            try:
                self._read_events(device)
            except OSError as exc:
                # Most common real-world cause: the controller was
                # unplugged mid-session -- reads start failing (ENODEV).
                # Go back to scanning rather than crashing the whole
                # thread (and, in link/server.py's case, the whole
                # control server) over a disconnect that could just be
                # someone changing batteries.
                log.warning("gamepad disconnected (%s) -- rescanning.", exc)

    def _find_device(self):
        if self.device_path:
            try:
                return evdev.InputDevice(self.device_path)
            except OSError as exc:
                log.warning("configured gamepad device %s not available (%s)",
                            self.device_path, exc)
                return None

        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
            except OSError:
                continue
            capabilities = device.capabilities()
            has_abs = ecodes.EV_ABS in capabilities
            has_a_button = (
                ecodes.EV_KEY in capabilities
                and ecodes.BTN_A in capabilities[ecodes.EV_KEY]
            )
            if has_abs and has_a_button:
                return device
        return None

    def _read_events(self, device):
        abs_info = dict(device.capabilities(absinfo=True).get(ecodes.EV_ABS, []))
        left_code = getattr(ecodes, self.left_y_code)
        right_code = getattr(ecodes, self.right_y_code)

        for event in device.read_loop():
            if not self._running:
                return

            if event.type == ecodes.EV_ABS and event.code in (left_code, right_code):
                info = abs_info.get(event.code)
                if info is None:
                    continue
                normalized = _normalize_axis(event.value, info.min, info.max)
                if event.code == left_code:
                    self._left_norm = normalized
                else:
                    self._right_norm = normalized
                self.on_drive(
                    _pwm_from_axis(self._left_norm),
                    _pwm_from_axis(self._right_norm),
                )

            elif event.type == ecodes.EV_KEY and event.value in (0, 1):
                # value 1 = pressed, 0 = released -- the hold-repeat value
                # (2) some drivers send is deliberately ignored here, it's
                # not a new press.
                self.on_button(event.code, event.value == 1)


def robot_state_button_handler(state):
    """Returns an on_button callback wiring the gamepad's buttons to a
    link.robot_state.RobotState -- used by link/server.py.

    BTN_A arms AUTO mode -- this is the controller's "go to the next
    point" button: as of 2026-09-07 this really drives the robot toward
    whatever NAV/RTE last set as nav_target (see link/robot_state.py's
    module docstring and link/autopilot.py for the heading/distance PID
    loop this now runs on every GPS fix), not just an armed-but-inert
    mode. BTN_B stops the robot -- same as the STP sentence (full stop:
    motors zeroed, mode back to IDLE, any in-progress route cleared),
    not just a hand-back to MANUAL, since a plain mode switch on its own
    doesn't cancel whatever nav_target/route is still armed. BTN_START is
    a second, always-available physical emergency stop, same as STP --
    kept alongside BTN_B (redundant on purpose: two ways to stop is safer
    than one) mirroring remote_control.py's previous convention where the
    Start button ended the program.

    Taking the joystick back over from an active AUTO drive is handled
    separately, in robot_state_drive_handler() below -- not here, since
    that needs to distinguish an actual stick push from the idle/centered
    stick's own analog noise, which button presses don't have."""
    def _on_button(code, pressed):
        if not pressed or not _EVDEV_AVAILABLE:
            return
        if code == ecodes.BTN_A:
            state.set_mode("AUTO")
        elif code == ecodes.BTN_B:
            state.stop()
        elif code == ecodes.BTN_START:
            state.stop()
    return _on_button


def robot_state_drive_handler(state):
    """Returns an on_drive callback wiring the gamepad's sticks to a
    link.robot_state.RobotState -- used by link/server.py. Forwards every
    axis change to state.drive(), same as before, but ALSO switches back
    to MANUAL mode first, and only when the stick is actually pushed away
    from center (nonzero PWM on either side) -- a physical operator can
    always take back control from an active AUTO drive by touching a
    stick, same "manual override always wins" convention this project
    already uses for NAV/STP clearing an in-progress route.

    The nonzero gate matters: GamepadReader calls on_drive on every ABS
    event, including the analog noise/jitter a centered, untouched stick
    can still produce -- _pwm_from_axis maps those to (0, 0), but if
    EVERY such call forced MANUAL, an idle stick's own noise would
    silently cancel AUTO-mode driving moments after BTN_A armed it,
    defeating the whole point of that button. A genuine push doesn't have
    this problem (an idle controller doesn't produce one by definition),
    so only a nonzero result triggers the mode switch."""
    def _on_drive(left_pwm, right_pwm):
        if left_pwm != 0 or right_pwm != 0:
            state.set_mode("MANUAL")
        state.drive(left_pwm, right_pwm)
    return _on_drive
