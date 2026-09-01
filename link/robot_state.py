"""Command handlers for the control link, kept separate from the socket
plumbing (link.server) so they can be unit-tested without opening a real
TCP connection.

IMPORTANT -- current scope: this updates an in-memory state and validates
inputs (ranges, allowed values), but does NOT yet drive real hardware.
motor_control/pwm.py today is a blocking script that opens the GPIO chip
and loops reading stdin -- it isn't an importable function, and there is
no safe way to call it from here without either refactoring it into a
function the way pid_controller.py already is, or having this server take
over stdin/GPIO ownership from the existing dgps_transfer.py | pid_controller.py
| pwm.py pipeline. Both are real design decisions -- see the "STP"/"DRV"
handlers below for the exact spot to wire in real motor control once
that's decided. Until then, this is honest, testable scaffolding: sending
commands, validating them, and getting the right ACK/ERR back all work
end to end, but the robot doesn't physically move yet.
"""

import threading
import time

PWM_MIN, PWM_MAX = -255, 255
VALID_MODES = ("AUTO", "MANUAL", "IDLE")
VALID_PID_LOOPS = ("D", "A")
VALID_CAM_COMMANDS = ("SNAP", "REC_START", "REC_STOP")


class CommandError(Exception):
    """Raised by a handler to signal an ERR response. `code` and
    `message` map directly onto the ERR sentence's two fields."""
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class RobotState:
    """Everything the control link needs to know / change, guarded by a
    single lock since the TCP server is multi-threaded (one thread per
    connection)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.mode = "IDLE"
        self.left_pwm = 0
        self.right_pwm = 0
        self.pid_gains = {"D": None, "A": None}  # None until PID sets them
        self.nav_target = None  # (lat, lat_dir, lon, lon_dir) once NAV is used
        self.last_command_at = None

    # -- STP: emergency stop, highest priority -------------------------
    def stop(self):
        with self._lock:
            self.left_pwm = 0
            self.right_pwm = 0
            self.mode = "IDLE"
            self.last_command_at = time.time()
            # TODO: once pwm.py exposes a callable, call it here with
            # (0, 0) instead of/in addition to updating this state.

    # -- DRV: direct manual drive ---------------------------------------
    def drive(self, left_pwm, right_pwm):
        left_pwm, right_pwm = self._validate_pwm(left_pwm), self._validate_pwm(right_pwm)
        with self._lock:
            self.left_pwm = left_pwm
            self.right_pwm = right_pwm
            self.last_command_at = time.time()
            # TODO: call into motor_control once it exposes a function
            # instead of being a blocking stdin-reading script.
        return left_pwm, right_pwm

    @staticmethod
    def _validate_pwm(value):
        try:
            value = int(float(value))
        except (TypeError, ValueError):
            raise CommandError("01", f"PWM_NOT_A_NUMBER:{value}")
        if not (PWM_MIN <= value <= PWM_MAX):
            raise CommandError("02", f"PWM_OUT_OF_RANGE:{value}")
        return value

    # -- MOD: switch operating mode -------------------------------------
    def set_mode(self, mode):
        mode = (mode or "").upper()
        if mode not in VALID_MODES:
            raise CommandError("03", f"UNKNOWN_MODE:{mode}")
        with self._lock:
            self.mode = mode
            if mode != "MANUAL":
                self.left_pwm = 0
                self.right_pwm = 0
            self.last_command_at = time.time()

    # -- NAV: set a GPS waypoint -----------------------------------------
    def set_nav_target(self, lat, lat_dir, lon, lon_dir):
        if lat_dir not in ("N", "S") or lon_dir not in ("E", "W"):
            raise CommandError("04", f"BAD_LAT_LON_DIRECTION:{lat_dir}{lon_dir}")
        try:
            float(lat)
            float(lon)
        except (TypeError, ValueError):
            raise CommandError("05", f"BAD_LAT_LON_VALUE:{lat},{lon}")
        with self._lock:
            self.nav_target = (lat, lat_dir, lon, lon_dir)
            self.last_command_at = time.time()
            # TODO: hook this into the GPS/PID pipeline (gps/dgps_transfer.py
            # and pid/pid_controller.py) so AUTO mode actually steers here.

    # -- PID: live gain update --------------------------------------------
    def set_pid_gains(self, loop, kp, ki, kd):
        loop = (loop or "").upper()
        if loop not in VALID_PID_LOOPS:
            raise CommandError("06", f"UNKNOWN_PID_LOOP:{loop}")
        try:
            kp, ki, kd = float(kp), float(ki), float(kd)
        except (TypeError, ValueError):
            raise CommandError("07", f"BAD_PID_VALUE:{kp},{ki},{kd}")
        with self._lock:
            self.pid_gains[loop] = (kp, ki, kd)
            self.last_command_at = time.time()
            # TODO: apply to the live pid.PIDController instances once this
            # server shares a process with the control pipeline.

    # -- CAM: snapshot / recording ----------------------------------------
    def camera_command(self, action):
        action = (action or "").upper()
        if action not in VALID_CAM_COMMANDS:
            raise CommandError("08", f"UNKNOWN_CAM_COMMAND:{action}")
        # Not implemented: no camera capture code exists in this project yet.
        raise CommandError("09", f"CAM_NOT_IMPLEMENTED:{action}")

    # -- STA: status snapshot for telemetry --------------------------------
    def status(self):
        with self._lock:
            return {
                "mode": self.mode,
                "left_pwm": self.left_pwm,
                "right_pwm": self.right_pwm,
                "nav_target": self.nav_target,
            }
