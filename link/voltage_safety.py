"""Battery-critical-voltage safety: rumble warning + forced idle + AUTO
lockout.

Explicit user request (2026-10-12), verbatim:

    "je ne veux pas que la pi se coupe lors d'un appel de courant du
    au mode manuel. Si la tension batterie devient critique (on
    commence a un seuil de 11.5 Volts) alors les vibreurs de la
    manette doivent s'activer fort en continue jusqu'a ce que les
    joysticks reviennent au neutre. Alors on passe en idle et on
    attends la recharge."

    (I don't want the Pi to get cut off by a current draw caused by
    manual driving. If the battery voltage becomes critical (starting
    at a threshold of 11.5V), the gamepad's vibration motors must
    activate strongly and continuously until the joysticks return to
    neutral. Then we switch to idle and wait for the recharge.)

This is a SEPARATE, short-timescale safety layer from
link.shutdown_policy: that module schedules a clean, graceful poweroff
hours ahead of time, from an AVERAGED multi-hour power projection
("won't reach the next sunrise"). This one reacts within
VOLTAGE_SAFETY_CHECK_INTERVAL_S to the RAW, instantaneous
battery_voltage reading, specifically to stop drawing motor current
(by forcing the robot to idle) before a heavy, sustained manual-drive
current draw drags the battery voltage down far enough to trip the
EPever Tracer's own hardware low-voltage load-disconnect protection --
which would cut power to the Pi itself, ungracefully, with nothing
this project's software could do about it after the fact. Neither
module touches the other's state.

Design, confirmed with the user via three follow-up questions
(2026-10-12):
  - MANUAL mode: no hard lock on the joystick -- a physical operator
    pushing the stick always keeps driving, same "manual always wins"
    convention this project uses everywhere else (see
    link.gamepad_handler.robot_state_drive_handler). The continuous
    strong rumble is the ONLY deterrent while voltage stays critical
    and a stick is off-center; releasing both sticks to neutral stops
    the rumble and drops the mode to IDLE. Pushing the stick again
    later, even while still critical, is allowed to re-arm MANUAL --
    the rumble simply starts again, same as the first time.
  - AUTO mode: no joystick to warn through, so a critical reading
    forces IDLE immediately (no rumble -- nobody is holding the
    controller in AUTO). Unlike MANUAL, AUTO is also LOCKED OUT from
    being (re-)armed (link.robot_state.RobotState.set_mode() and
    .start_waypoint_return() both refuse "AUTO" with
    CommandError("26", "AUTO_LOCKED_LOW_VOLTAGE")) until the voltage
    has genuinely recovered -- not just ticked back above 11.5V, but
    past RECOVERY_VOLTAGE_THRESHOLD_V (a +0.5V hysteresis margin the
    user chose explicitly, to avoid flapping the lock on and off right
    at the bare threshold). This lockout is independent of which mode
    actually triggered it -- see update_auto_lockout() below.

Same "pure decision function, separate I/O-performing background-
thread wrapper" split as link.camera_power_policy/link.shutdown_policy,
and the same choice to read raw values off link.robot_state.RobotState
every tick rather than share state with any other background thread
(this project's established "RobotState as the single shared hub,
everything else just reads or writes through it" convention).
"""
import logging
import threading
import time

log = logging.getLogger(__name__)

# The user's own explicit value: "on commence a un seuil de 11.5 Volts".
CRITICAL_VOLTAGE_THRESHOLD_V = 11.5

# User-chosen margin (2026-10-12, answer to the "seuil de reprise"
# follow-up question): the AUTO lockout stays on until voltage climbs
# back above this, not merely back above CRITICAL_VOLTAGE_THRESHOLD_V,
# so a voltage oscillating right around 11.5V doesn't flap the lock on
# and off every tick.
VOLTAGE_RECOVERY_MARGIN_V = 0.5
RECOVERY_VOLTAGE_THRESHOLD_V = CRITICAL_VOLTAGE_THRESHOLD_V + VOLTAGE_RECOVERY_MARGIN_V

# Fast enough that releasing the joystick stops the rumble and drops to
# IDLE promptly -- this is a safety reaction, not a slow trend like
# link.led_control's 10-minute low-light timer.
VOLTAGE_SAFETY_CHECK_INTERVAL_S = 0.5


def is_voltage_critical(battery_voltage, critical_threshold_v=CRITICAL_VOLTAGE_THRESHOLD_V):
    """Pure predicate. A missing reading (None -- Tracer unavailable, or
    not read yet) is deliberately NOT critical, same "uncertainty -> do
    not act" posture link.shutdown_policy uses for its own, much
    higher-stakes decision: a missing reading must never force an idle/
    lockout, or every cold start (before the Tracer's first successful
    poll) would start locked out for no reason."""
    if battery_voltage is None:
        return False
    return battery_voltage <= critical_threshold_v


def decide_voltage_safety_action(battery_voltage, mode, left_pwm, right_pwm,
                                  critical_threshold_v=CRITICAL_VOLTAGE_THRESHOLD_V):
    """Pure decision function for ONE tick, given the raw inputs
    link.robot_state.RobotState.power_status()/status() report.
    Returns (should_rumble: bool, should_force_idle: bool).

      - should_rumble is True only in MANUAL mode, while critical, with
        at least one stick off-center (a genuine push) -- a centered
        stick draws no meaningful extra motor current, so there is
        nothing to warn about yet even if voltage is already low.
      - should_force_idle is True either in MANUAL once both sticks
        are back at neutral (critical voltage was draining the battery
        while driving; stop driving, and stay stopped), or in AUTO
        immediately (no stick to wait for -- autonomous driving draws
        the same kind of current with nobody there to warn).
      - IDLE mode (or anything once not critical) never produces
        either action -- there's nothing left to stop."""
    if not is_voltage_critical(battery_voltage, critical_threshold_v):
        return False, False

    joystick_neutral = left_pwm == 0 and right_pwm == 0

    if mode == "MANUAL":
        return (not joystick_neutral), joystick_neutral
    if mode == "AUTO":
        return False, True
    return False, False


def update_auto_lockout(currently_locked, battery_voltage,
                         critical_threshold_v=CRITICAL_VOLTAGE_THRESHOLD_V,
                         recovery_threshold_v=RECOVERY_VOLTAGE_THRESHOLD_V):
    """Pure hysteresis step for the AUTO lockout flag (see this
    module's docstring) -- independent of which mode is active or
    which one triggered the lock in the first place:

      - a critical reading always (re-)locks it;
      - a reading at/above recovery_threshold_v always clears it;
      - anything in between (including a missing/None reading) leaves
        it exactly as it was -- that's the hysteresis band, and the
        same "uncertainty changes nothing" posture as
        is_voltage_critical() above."""
    if battery_voltage is None:
        return currently_locked
    if battery_voltage <= critical_threshold_v:
        return True
    if battery_voltage >= recovery_threshold_v:
        return False
    return currently_locked


class VoltageSafety:
    """Background-thread wrapper: samples
    link.robot_state.RobotState.power_status()/status() every
    check_interval seconds, drives the gamepad's rumble through
    decide_voltage_safety_action(), forces IDLE through
    RobotState.set_mode() when that function says to, and keeps
    RobotState's AUTO lockout flag (set_auto_locked_low_voltage()) in
    sync via update_auto_lockout().

    `gamepad` is expected to be the same
    link.gamepad_handler.GamepadReader instance link/server.py already
    wires the joystick/buttons through -- None is also accepted (e.g.
    GAMEPAD_ENABLED=false), in which case the IDLE-forcing and AUTO
    lockout still work exactly the same, just with no rumble to drive
    (nothing to warn through with no controller)."""

    def __init__(self, state, gamepad=None,
                 check_interval=VOLTAGE_SAFETY_CHECK_INTERVAL_S,
                 critical_threshold_v=CRITICAL_VOLTAGE_THRESHOLD_V,
                 recovery_threshold_v=RECOVERY_VOLTAGE_THRESHOLD_V):
        self.state = state
        self.gamepad = gamepad
        self.check_interval = check_interval
        self.critical_threshold_v = critical_threshold_v
        self.recovery_threshold_v = recovery_threshold_v
        self._running = False
        self._thread = None
        self._locked = False
        # Tracks whether THIS policy currently believes it's rumbling,
        # so _check_once only calls start_rumble()/stop_rumble() on an
        # actual transition -- both are safe/idempotent to call every
        # tick regardless, but this keeps the intent explicit, same
        # "write hardware only on a real transition" habit as
        # link.led_control.LedController._leds_off.
        self._rumbling = False

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            self._check_once()
            time.sleep(self.check_interval)

    def _check_once(self):
        try:
            power = self.state.power_status()
            status = self.state.status()
        except Exception:
            log.exception("VoltageSafety: failed to read state while checking")
            return

        battery_voltage = power.get("battery_voltage") if power.get("available") else None

        self._locked = update_auto_lockout(
            self._locked, battery_voltage, self.critical_threshold_v, self.recovery_threshold_v,
        )
        try:
            self.state.set_auto_locked_low_voltage(self._locked)
        except Exception:
            log.exception("VoltageSafety: failed to record the AUTO lockout on state")

        should_rumble, should_force_idle = decide_voltage_safety_action(
            battery_voltage, status.get("mode"),
            status.get("left_pwm", 0), status.get("right_pwm", 0),
            self.critical_threshold_v,
        )

        if self.gamepad is not None:
            if should_rumble and not self._rumbling:
                self.gamepad.start_rumble(strong=True)
                self._rumbling = True
            elif not should_rumble and self._rumbling:
                self.gamepad.stop_rumble()
                self._rumbling = False

        if should_force_idle:
            try:
                log.warning(
                    "VoltageSafety: battery voltage critical (%.2fV <= %.1fV) -- "
                    "forcing IDLE and waiting for recharge",
                    battery_voltage, self.critical_threshold_v,
                )
                self.state.set_mode("IDLE")
            except Exception:
                log.exception("VoltageSafety: failed to force IDLE on state")
