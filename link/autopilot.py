"""Pure GPS-navigation math and the AUTO-mode drive loop's PID stage.

Added 2026-09-07: this is what makes AUTO mode (see link/robot_state.py)
actually drive the robot toward nav_target / the current route waypoint,
instead of just arming a mode that leaves the motors at zero. The
gamepad's BTN_A ("go to the next point", link/gamepad_handler.py) and
the web UI's "GPS route" button both end up here, through
RobotState.update_gps_fix()'s autonomous-driving tick, on every new GPS
fix.

Deliberately independent of two other pieces of this codebase, for two
different reasons:
  - gps/gps_delta.py has a real, pre-existing bug (its functions
    reference an undefined module-level A/B instead of their own
    parameters -- flagged in this repo's README) and was never fixed
    because nothing actually called it; distance/bearing are
    reimplemented here from scratch instead (same reasoning as
    link/robot_state.py's own _haversine_distance_m before this refactor
    -- now moved here so there's a single copy).
  - pid/pid_controller.py's module-level `pid_distance`/`pid_angle`
    globals back that file's own standalone CLI (`python3 -m
    pid.pid_controller`, reading distance/angle off stdin) and its
    offline simulation tooling (pid/simulate_pid.py,
    pid/simulate_motor_commands.py) -- reusing those two specific
    instances here would mean the CLI and this live control loop fight
    over the same PID integral/derivative state. Autopilot below creates
    its OWN PIDController instances (same class, fresh objects) instead.

Honesty note: the default gains reused below (kp=1/ki=0/kd=0.5 for
distance, kp=0.5/ki=0/kd=1 for angle) are the ones pid/pid_controller.py
already ships as its defaults, validated only in pid/simulate_pid.py's
and pid/simulate_motor_commands.py's OFFLINE simulation against a
generic, abstract differential-drive plant (see this repo's README,
"Reglage des PID hors robot") -- not yet tuned, or even run, on this
actual robot with real GPS. The mixing/scaling math (speed_pwm +/-
angular_pwm, +/-2.0 m/s and +/-360 deg/s clamps) is copied verbatim from
pid_controller.py's move_to_target() so the two stay behaviorally
identical, but the two SIGN conventions below (heading_error_for_pid,
and feeding -distance_m rather than distance_m into the distance PID)
are a first-principles choice made HERE, not a copy of an established
convention -- move_to_target() has never been wired to a real distance/
angle source in this project to know which sign it actually expects.
Expect to retune live, at very low speed, under supervision, before
trusting this for anything beyond a first careful test.

Known real limitation, not papered over: `cap` (course over ground, from
the GPS receiver's own GPRMC sentence -- see link/gps_reader.py) is the
only heading source this project has -- there is no compass/IMU. Course
over ground is only meaningful while the robot is actually moving; near
a waypoint, or right after starting from a stop, it can be stale or
noisy, which can make the angle correction below chase the wrong way for
a moment. Fixing this properly needs a compass or dead-reckoning, which
this project doesn't have yet -- worth knowing before trusting AUTO mode
at anything but a slow, supervised pace.
"""
import math

from pid.pid_controller import PIDController

EARTH_RADIUS_M = 6371000.0

# Same clamps and PWM scaling as pid/pid_controller.py's move_to_target()
# -- copied rather than imported, since that function also prints to
# stdout and reads its own module-level PID instances, neither of which
# this live loop wants.
MAX_SPEED_MPS = 2.0
MAX_ANGULAR_DEG_S = 360.0
PWM_MAX = 255

DEFAULT_DISTANCE_GAINS = (1.0, 0.0, 0.5)  # kp, ki, kd -- see honesty note above
DEFAULT_ANGLE_GAINS = (0.5, 0.0, 1.0)


def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two decimal-degree points.
    Moved here 2026-09-07 from link/robot_state.py (used to be a private
    _haversine_distance_m there) so navigation math lives in one place;
    robot_state.py now imports this instead of keeping its own copy."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing from point 1 to point 2, in compass
    degrees (0 = north, 90 = east, ...), normalized to [0, 360)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)
    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    return math.degrees(math.atan2(y, x)) % 360


def _normalize_angle_deg(angle):
    """Wraps any angle in degrees to (-180, 180]."""
    wrapped = (angle + 180) % 360 - 180
    return 180.0 if wrapped == -180 else wrapped


def heading_error_for_pid(current_heading_deg, target_bearing_deg):
    """Signed value to feed the angle PID loop below (setpoint always
    0). Worked out once, carefully, so it's not re-derived (or gotten
    backwards) at every call site:

    The mixing formula (see Autopilot.compute()) is left = speed +
    angular, right = speed - angular -- so a POSITIVE angular_pwm makes
    the left wheel spin faster than the right, which turns the robot
    RIGHT (clockwise), same as a tank pivoting on its right track.
    PIDController.update() always computes error = setpoint - measured,
    i.e. error = -measured here (setpoint is 0) -- so for the PID output
    to come out positive (turn right) when the target is clockwise of
    the current heading, `measured` must be NEGATIVE in that case. That
    is current_heading - target_bearing (not the more intuitive
    target-minus-current): e.g. current=0 deg, target=90 deg (target is
    90 deg clockwise, robot needs to turn right) -> 0 - 90 = -90, a
    negative measured value -> positive error -> positive angular_pwm ->
    turns right. Confirmed correct by hand for this and the opposite
    case (current=90, target=0) plus a wraparound case (current=350,
    target=10) before being trusted here.
    """
    return _normalize_angle_deg(current_heading_deg - target_bearing_deg)


class Autopilot:
    """Wraps two independent PIDController instances (distance, angle --
    NOT pid/pid_controller.py's own module-level pid_distance/pid_angle,
    see module docstring) and reuses move_to_target()'s mixing math
    (speed +/- angular, scaled to PWM) to turn a live (distance_m,
    heading_error_deg) pair into a (left_pwm, right_pwm) motor command.
    Pure Python, no hardware/IO -- unlike motor_control.motor_driver.
    MotorDriver, always safe to construct; link/robot_state.py's
    RobotState creates one by default."""

    def __init__(self, distance_gains=DEFAULT_DISTANCE_GAINS, angle_gains=DEFAULT_ANGLE_GAINS):
        self._distance_gains = distance_gains
        self._angle_gains = angle_gains
        self._pid_distance = self._new_pid(distance_gains)
        self._pid_angle = self._new_pid(angle_gains)

    @staticmethod
    def _new_pid(gains):
        kp, ki, kd = gains
        return PIDController(setpoint=0.0, kp=kp, ki=ki, kd=kd)

    def set_gains(self, loop, kp, ki, kd):
        """Live gain update for one loop -- "D" (distance) or "A"
        (angle), same two-letter codes as link/robot_state.py's PID
        sentence (VALID_PID_LOOPS). Takes effect on the very next
        compute() call; does not reset integral/previous_error (a plain
        gain tweak isn't a context switch the way (re)arming AUTO or
        reaching a waypoint is -- see reset())."""
        if loop == "D":
            self._pid_distance.kp, self._pid_distance.ki, self._pid_distance.kd = kp, ki, kd
        elif loop == "A":
            self._pid_angle.kp, self._pid_angle.ki, self._pid_angle.kd = kp, ki, kd
        else:
            raise ValueError(f"unknown PID loop: {loop!r}")

    def reset(self):
        """Clears both loops' integral/derivative history -- call this
        whenever autonomous driving (re)starts from a fresh context:
        link/robot_state.py's RobotState does this on every switch INTO
        AUTO mode, and whenever the route advances to a new waypoint, so
        a stale integral/derivative from a previous, unrelated leg or
        session doesn't leak into the next one (same "no derivative
        kick" reasoning as pid.PIDController.update()'s own
        previous_error handling)."""
        self._pid_distance.reset()
        self._pid_angle.reset()

    def compute(self, distance_m, heading_error_deg):
        """distance_m: current distance to the target, meters (>= 0).
        heading_error_deg: pass heading_error_for_pid()'s return value
        here, not a raw bearing difference -- see its docstring for the
        sign convention this expects. Returns (left_pwm, right_pwm),
        ints clamped to +/-255."""
        # Distance PID: setpoint 0, fed -distance_m (not distance_m).
        # PIDController.update() always computes error = setpoint -
        # measured; feeding distance_m directly would give error =
        # -distance_m -- maximum REVERSE speed far from the target,
        # settling only once arrived, which is backwards. Feeding
        # -distance_m gives error = distance_m: positive (forward) speed
        # while far away, shrinking to 0 as distance_m shrinks to 0 --
        # the intended behavior. See this module's honesty note: this
        # sign is a first-principles choice for this specific caller,
        # not a copy of an established convention.
        speed = self._pid_distance.update(-distance_m)
        angular_velocity = self._pid_angle.update(heading_error_deg)

        speed = max(min(speed, MAX_SPEED_MPS), -MAX_SPEED_MPS)
        angular_velocity = max(min(angular_velocity, MAX_ANGULAR_DEG_S), -MAX_ANGULAR_DEG_S)

        speed_pwm = (speed / MAX_SPEED_MPS) * PWM_MAX
        angular_pwm = (angular_velocity / MAX_ANGULAR_DEG_S) * PWM_MAX

        left = speed_pwm + angular_pwm
        right = speed_pwm - angular_pwm
        left = max(min(left, PWM_MAX), -PWM_MAX)
        right = max(min(right, PWM_MAX), -PWM_MAX)
        return int(round(left)), int(round(right))
