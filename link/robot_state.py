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

import math
import os
import threading
import time
import urllib.error
import urllib.request

from link.nmea import nmea_to_decimal

PWM_MIN, PWM_MAX = -255, 255
VALID_MODES = ("AUTO", "MANUAL", "IDLE")
VALID_PID_LOOPS = ("D", "A")
VALID_CAM_COMMANDS = ("SNAP", "REC_START", "REC_STOP")

# RTE: an ordered list of GPS waypoints the robot chases one at a time (see
# set_route()/_advance_route_if_arrived() below). A hard cap keeps a
# malformed or oversized upload (wrong file picked in the browser) from
# producing a sentence with thousands of fields -- 200 is far more than a
# manually curated route would ever realistically need.
ROUTE_MAX_POINTS = 200
# How close (meters) the live GPS fix must get to the current waypoint
# before automatically advancing to the next one. Consumer GPS without
# DGPS correction is typically only accurate to a few meters, so this is
# deliberately generous rather than tight -- override via the environment
# if a particular receiver/route needs something stricter or looser.
ROUTE_ARRIVAL_RADIUS_M = float(os.environ.get("ROUTE_ARRIVAL_RADIUS_M", "5.0"))


def _haversine_distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two decimal-degree points.
    Written from scratch here rather than reusing gps/gps_delta.py's
    distance_to_target_meter(), which has a pre-existing bug (references
    undefined A/B instead of its own parameters, flagged in this repo's
    README) -- same reasoning as robot-webserver's own independent
    client-side haversine in /control's status bar."""
    R = 6371000.0  # Earth radius, meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# CAM,SNAP is handled by calling into camera/stream_server.py's own /snap
# endpoint over plain HTTP rather than sharing a process with it (same
# reasoning as robot-webserver's own camera proxy: simplest thing that
# works over a stable local connection). Both processes run on this same
# Pi, hence 127.0.0.1 by default -- override via env vars if the camera
# script ever runs elsewhere. Read from os.environ at call time (not at
# import time) so tests can monkeypatch them per-case, same pattern as
# link.server's GPS_DEVICE/GPS_BAUDRATE.
CAMERA_HOST_DEFAULT = "127.0.0.1"
CAMERA_PORT_DEFAULT = 8000
CAMERA_SNAP_PATH_DEFAULT = "/snap"
CAMERA_SNAP_TIMEOUT = 3  # seconds -- how long to wait before giving up


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
        self.nav_target = None  # (lat, lat_dir, lon, lon_dir) once NAV/RTE is used
        # RTE: the full ordered waypoint list (empty = no active route) and
        # the index of the one currently in nav_target. route_index reaches
        # len(route) once the last waypoint has been reached -- that's the
        # "route complete" state, distinct from "route index still moving
        # through the list" (see _advance_route_if_arrived below).
        self.route = []
        self.route_index = 0
        self.last_command_at = None
        # Live GPS fix, set by link.gps_reader.GPSReader in the background.
        # None until the first fix arrives -- STA reports the honest 0.0
        # placeholder for as long as that's true (no GPS receiver attached,
        # or no fix yet).
        self.current_lat = None
        self.current_lon = None
        self.cap = 0.0          # course over ground, degrees -- from GPRMC
        self.speed_kmh = 0.0    # from GPRMC's speed over ground
        self.last_fix_at = None

    # -- STP: emergency stop, highest priority -------------------------
    def stop(self):
        with self._lock:
            self.left_pwm = 0
            self.right_pwm = 0
            self.mode = "IDLE"
            # An emergency stop also cancels any route in progress -- the
            # whole point of STP is "stop and wait for a person", not
            # "stop, then quietly resume chasing waypoints on the next fix".
            self.route = []
            self.route_index = 0
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

    # -- NAV: set a single GPS waypoint ----------------------------------
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
            # A manual NAV takes back control from an active route -- without
            # this, the very next GPS fix's arrival check (see
            # _advance_route_if_arrived) could silently overwrite the
            # operator's manual target with whatever the route was pursuing.
            self.route = []
            self.route_index = 0
            self.last_command_at = time.time()
            # TODO: hook this into the GPS/PID pipeline (gps/dgps_transfer.py
            # and pid/pid_controller.py) so AUTO mode actually steers here.

    # -- RTE: set an ordered list of GPS waypoints to chase automatically --
    def set_route(self, fields):
        """fields = [count, lat1, lat_dir1, lon1, lon_dir1, lat2, ...] --
        same per-point encoding as NAV, just repeated `count` times.
        Replaces any previous route and re-arms automatic advancement from
        the first waypoint. The robot doesn't drive itself yet (same
        scaffolding caveat as NAV, see module docstring), but nav_target is
        kept in sync with "the waypoint currently being pursued" so
        everything already reading it (STA, /control's status bar) shows
        route progress with no further changes needed on their end."""
        if not fields:
            raise CommandError("13", "RTE_NEEDS_COUNT_AND_POINTS")
        try:
            count = int(fields[0])
        except (TypeError, ValueError):
            raise CommandError("13", f"RTE_BAD_COUNT:{fields[0]}")
        if count < 1:
            raise CommandError("13", f"RTE_EMPTY_ROUTE:{count}")
        if count > ROUTE_MAX_POINTS:
            raise CommandError("13", f"RTE_TOO_MANY_POINTS:{count}>{ROUTE_MAX_POINTS}")

        point_fields = fields[1:]
        if len(point_fields) != count * 4:
            raise CommandError(
                "13",
                f"RTE_FIELD_COUNT_MISMATCH:expected_{count * 4}_fields_got_{len(point_fields)}",
            )

        points = []
        for i in range(count):
            lat, lat_dir, lon, lon_dir = point_fields[i * 4:(i + 1) * 4]
            if lat_dir not in ("N", "S") or lon_dir not in ("E", "W"):
                raise CommandError("13", f"RTE_BAD_LAT_LON_DIRECTION:point_{i}:{lat_dir}{lon_dir}")
            try:
                float(lat)
                float(lon)
            except (TypeError, ValueError):
                raise CommandError("13", f"RTE_BAD_LAT_LON_VALUE:point_{i}:{lat},{lon}")
            points.append((lat, lat_dir, lon, lon_dir))

        with self._lock:
            self.route = points
            self.route_index = 0
            self.nav_target = points[0]
            self.last_command_at = time.time()

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

        if action == "SNAP":
            self._request_snapshot()
            with self._lock:
                self.last_command_at = time.time()
            return

        # REC_START/REC_STOP: not implemented -- no video recording code
        # exists in this project yet (camera/stream_server.py only ever
        # provides the live preview and one-shot snapshots).
        raise CommandError("09", f"CAM_NOT_IMPLEMENTED:{action}")

    def _request_snapshot(self):
        """Asks camera/stream_server.py (a separate process, possibly not
        even running) to save the current frame via its GET /snap
        endpoint. Any failure -- camera script not running, no frame
        grabbed yet, network hiccup -- becomes one CommandError so the
        console shows a clear reason instead of a raw socket/HTTP
        traceback; nothing here assumes the camera is actually available."""
        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        path = os.environ.get("CAMERA_SNAP_PATH", CAMERA_SNAP_PATH_DEFAULT)
        url = f"http://{host}:{port}{path}"
        try:
            with urllib.request.urlopen(url, timeout=CAMERA_SNAP_TIMEOUT) as resp:
                if resp.status != 200:
                    raise CommandError("12", f"CAMERA_SNAP_FAILED:HTTP_{resp.status}")
        except urllib.error.HTTPError as exc:
            # 503 from _handle_snap means "no frame yet" (grabber hasn't
            # produced one, e.g. camera just starting up) -- still a clean,
            # specific reason rather than a stack trace.
            raise CommandError("12", f"CAMERA_SNAP_FAILED:HTTP_{exc.code}")
        except urllib.error.URLError as exc:
            raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")

    # -- GPS: live fix from link.gps_reader.GPSReader, if a receiver is
    #    attached -------------------------------------------------------
    def update_gps_fix(self, lat, lon, speed_kmh=None, cap=None):
        with self._lock:
            self.current_lat = lat
            self.current_lon = lon
            if speed_kmh is not None:
                self.speed_kmh = speed_kmh
            if cap is not None:
                self.cap = cap
            self.last_fix_at = time.time()
            self._advance_route_if_arrived()

    def _advance_route_if_arrived(self):
        """Called with self._lock already held, on every new GPS fix. This
        is what makes RTE "chase the whole list" rather than just remember
        it: once the fix is within ROUTE_ARRIVAL_RADIUS_M of the waypoint
        currently in nav_target, moves on to the next one in self.route --
        same one-at-a-time semantics as sending a fresh NAV yourself, just
        automatic. No-op once the route is exhausted (route_index ==
        len(route)) or if there's no active route at all."""
        if not self.route or self.route_index >= len(self.route):
            return
        if self.current_lat is None or self.current_lon is None:
            return

        target_lat, target_lat_dir, target_lon, target_lon_dir = self.route[self.route_index]
        target_lat_decimal = nmea_to_decimal(target_lat, target_lat_dir)
        target_lon_decimal = nmea_to_decimal(target_lon, target_lon_dir)
        if target_lat_decimal is None or target_lon_decimal is None:
            return

        distance_m = _haversine_distance_m(
            self.current_lat, self.current_lon, target_lat_decimal, target_lon_decimal
        )
        if distance_m <= ROUTE_ARRIVAL_RADIUS_M:
            self.route_index += 1
            if self.route_index < len(self.route):
                self.nav_target = self.route[self.route_index]
            # else: route complete -- nav_target is left on the last
            # waypoint and route_index stays at len(self.route) as the
            # "done" marker (see status() below).

    # -- STA: status snapshot for telemetry --------------------------------
    def status(self):
        with self._lock:
            return {
                "mode": self.mode,
                "left_pwm": self.left_pwm,
                "right_pwm": self.right_pwm,
                "nav_target": self.nav_target,
                "current_lat": self.current_lat,
                "current_lon": self.current_lon,
                "cap": self.cap,
                "speed_kmh": self.speed_kmh,
                # Not (yet) part of the STA wire sentence -- nav_target
                # above already reflects route progress for anything
                # reading it today. Exposed here for tests and for a
                # future STA extension (extend-only, see
                # pages/protocole_controle.html) without needing another
                # change to this method's shape.
                "route_total": len(self.route),
                "route_index": self.route_index,
            }
