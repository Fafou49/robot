"""Command handlers for the control link, kept separate from the socket
plumbing (link.server) so they can be unit-tested without opening a real
TCP connection.

UPDATE (2026-09-07, part 1) -- STP/DRV/MOD now really drive the motors:
RobotState can be constructed with a `motor_driver` (a motor_control.
motor_driver.MotorDriver -- see link/server.py, which wires one in), and
drive()/stop()/set_mode() call into it whenever left_pwm/right_pwm
actually change. This resolves the design decision this docstring used
to flag as open (whether to refactor motor_control/pwm.py into a
callable, or have this server take over the existing dgps_transfer.py |
pid_controller.py | pwm.py pipeline -- the former is what happened:
pwm.py's logic now lives in motor_control/motor_driver.py, an
importable, reusable class).

UPDATE (2026-09-07, part 2) -- AUTO mode now really drives too: RobotState
also owns a `link.autopilot.Autopilot` (see that module for the full
heading/distance-to-PWM math and its honesty notes on untested gains and
the no-compass limitation). update_gps_fix() -- called on every new GPS
fix by link/gps_reader.py's GPSReader -- runs one autopilot tick whenever
mode == "AUTO" and there's a target (nav_target, set by NAV or RTE):
computes distance + heading error to that target, feeds them through the
autopilot, and drives the motors with the result, exactly like an
operator sending DRV would. The gamepad's BTN_A (link/gamepad_handler.py)
arms this by calling set_mode("AUTO"); BTN_B calls stop() (full stop,
same as STP) rather than just handing back to MANUAL, and touching the
joystick always overrides back to MANUAL immediately (see
link.gamepad_handler.robot_state_drive_handler) -- a physical operator
can always take back control mid-route.

`motor_driver` and `autopilot` are both optional constructor params
(motor_driver defaults to None, autopilot defaults to a fresh
link.autopilot.Autopilot() -- cheap, pure Python, no hardware) so every
existing test that constructs a bare RobotState() keeps working exactly
as before.
"""

import os
import threading
import time
import urllib.error
import urllib.request

from link.autopilot import Autopilot, bearing_deg, haversine_distance_m, heading_error_for_pid
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

# CAM,REC_START/REC_STOP (2026-09-18, genuinely implemented -- see
# camera/stream_server.py's VideoRecorder and _request_recording() below;
# both used to always raise CommandError("09", "CAM_NOT_IMPLEMENTED...")).
# Same cross-process HTTP pattern and same host/port as SNAP above --
# these just hit two different endpoints on that same camera process.
CAMERA_REC_START_PATH_DEFAULT = "/rec/start"
CAMERA_REC_STOP_PATH_DEFAULT = "/rec/stop"

# CAM,X -- link/gamepad_handler.py's save_waypoint_btn (default BTN_X, see
# robot_state_button_handler()) -- appends the live GPS fix to this file,
# see save_waypoint() below. Lives at <repo_root>/waypoints/waypoints.txt
# by default (a plain text file, not inside link/ itself, same "own data
# directory next to the code that owns it" idea as camera/tmp/ for
# snapshots) -- override with the WAYPOINTS_FILE environment variable.
DEFAULT_WAYPOINTS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "waypoints", "waypoints.txt"
)


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

    def __init__(self, motor_driver=None, autopilot=None):
        self._lock = threading.Lock()
        # Optional motor_control.motor_driver.MotorDriver -- see drive()/
        # stop()/set_mode() below. None (the default) keeps this class
        # fully hardware-free, which is what every pure-logic test in
        # tests/test_link_server.py relies on; link/server.py's
        # ControlServer is the one place that actually passes one in.
        self.motor_driver = motor_driver
        # link.autopilot.Autopilot -- pure Python, no hardware/IO, so
        # (unlike motor_driver) there's no reason not to default to a
        # real one; tests that want to inspect/replace it can still pass
        # their own. See update_gps_fix()/_autonomous_pwm_locked() below.
        self.autopilot = autopilot if autopilot is not None else Autopilot()
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
        # DGPS fix-quality flag (2026-09-19): None until a GGA sentence
        # with a quality field has actually arrived (see
        # link/gps_reader.py's DGPS_QUALITY), then True/False from then on
        # -- kept in sync by update_gps_fix() below. Distinct from a plain
        # bool default for the same reason nav_target/current_lat/lon
        # start as None: "no GGA received yet" and "received, not a DGPS
        # fix" are different states, and the web UI's DGPS badge should
        # only ever show for the former (see pages/protocole_controle.html
        # for the STA field this backs).
        self.is_dgps = None
        # CAM,REC_START/REC_STOP (2026-09-18): whether camera/stream_server.
        # py is currently believed to be writing a video file. This is
        # this class's own bookkeeping, not a live query of the camera
        # process -- kept in sync by camera_command() below (only flipped
        # AFTER the corresponding HTTP call actually succeeds, so a failed
        # REC_START never leaves this True with nothing really recording).
        # link/gamepad_handler.py's record_btn reads this to decide
        # whether the next press should send REC_START or REC_STOP, so
        # one button toggles both directions.
        self.is_recording = False

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
        if self.motor_driver is not None:
            self.motor_driver.drive(0, 0)

    # -- DRV: direct manual drive ---------------------------------------
    def drive(self, left_pwm, right_pwm):
        left_pwm, right_pwm = self._validate_pwm(left_pwm), self._validate_pwm(right_pwm)
        with self._lock:
            self.left_pwm = left_pwm
            self.right_pwm = right_pwm
            self.last_command_at = time.time()
        if self.motor_driver is not None:
            self.motor_driver.drive(left_pwm, right_pwm)
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
            zero_pwm = mode != "MANUAL"
            if zero_pwm:
                self.left_pwm = 0
                self.right_pwm = 0
            if mode == "AUTO":
                # Fresh PID state every time AUTO is (re-)armed -- e.g.
                # the gamepad's BTN_A, see link/gamepad_handler.py's
                # robot_state_button_handler() -- so a stale integral/
                # derivative from a previous, unrelated driving session
                # doesn't produce a derivative-kick-style jolt on the
                # first tick of this one.
                self.autopilot.reset()
            self.last_command_at = time.time()
        # Leaving MANUAL always stops the motors first -- driving only
        # resumes once update_gps_fix()'s autonomous tick has a fresh GPS
        # fix to compute a real correction from (typically within one GPS
        # update period), so there's a brief, deliberate pause rather
        # than the motors keeping whatever duty cycle they had a moment
        # ago.
        if zero_pwm and self.motor_driver is not None:
            self.motor_driver.drive(0, 0)

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
            # Applied to the live loop link/autopilot.py's Autopilot runs
            # during AUTO-mode driving (see update_gps_fix()) -- this used
            # to be a TODO ("once this server shares a process with the
            # control pipeline"), which is exactly what happened
            # 2026-09-07: PID (D=distance, A=angle) now tunes AUTO mode
            # live, from the console or the web UI, while it's driving.
            self.autopilot.set_gains(loop, kp, ki, kd)
            self.last_command_at = time.time()

    # -- CAM: snapshot / recording ----------------------------------------
    def camera_command(self, action):
        """UPDATE (2026-09-18): REC_START/REC_STOP used to always raise
        CommandError("09", "CAM_NOT_IMPLEMENTED:...") here -- no video
        recording code existed anywhere in this project. Both are now
        genuinely implemented, via camera/stream_server.py's VideoRecorder
        and _request_recording() below, mirroring SNAP's existing
        cross-process HTTP pattern (this NMEA control link and the camera
        script are two separate processes -- see _request_snapshot()'s
        own docstring for why). self.is_recording is only ever flipped
        AFTER the matching HTTP call actually succeeds, and both REC_START
        while already recording and REC_STOP while not are treated as a
        harmless no-op rather than an error -- link/gamepad_handler.py's
        record_btn only ever sends whichever one is_recording says is
        next, but the website's console can send either CAM,REC_START/
        CAM,REC_STOP command directly at any time, and there's no useful
        difference between "start recording, it already was" and "it just
        started"."""
        action = (action or "").upper()
        if action not in VALID_CAM_COMMANDS:
            raise CommandError("08", f"UNKNOWN_CAM_COMMAND:{action}")

        if action == "SNAP":
            self._request_snapshot()
            with self._lock:
                self.last_command_at = time.time()
            return

        if action == "REC_START":
            if self.is_recording:
                with self._lock:
                    self.last_command_at = time.time()
                return
            self._request_recording(start=True)
            with self._lock:
                self.is_recording = True
                self.last_command_at = time.time()
            return

        # action == "REC_STOP" (the only remaining VALID_CAM_COMMANDS value)
        if not self.is_recording:
            with self._lock:
                self.last_command_at = time.time()
            return
        self._request_recording(start=False)
        with self._lock:
            self.is_recording = False
            self.last_command_at = time.time()

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

    def _request_recording(self, start: bool):
        """Asks camera/stream_server.py to start or stop writing the live
        feed to a video file, via its GET /rec/start or /rec/stop endpoint
        -- same cross-process HTTP call as _request_snapshot() above, just
        a different path and (for /rec/start) the same 503-means-no-frame-
        yet convention as /snap. That 503 is exactly what makes REC_START
        "record a video if the camera is present" in practice: no camera
        delivering frames yet is treated the same as no camera plugged in
        at all, and turns into a clean CommandError here rather than
        arming a recorder with nothing to encode."""
        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        if start:
            path = os.environ.get("CAMERA_REC_START_PATH", CAMERA_REC_START_PATH_DEFAULT)
        else:
            path = os.environ.get("CAMERA_REC_STOP_PATH", CAMERA_REC_STOP_PATH_DEFAULT)
        url = f"http://{host}:{port}{path}"
        try:
            with urllib.request.urlopen(url, timeout=CAMERA_SNAP_TIMEOUT) as resp:
                if resp.status != 200:
                    raise CommandError("12", f"CAMERA_REC_FAILED:HTTP_{resp.status}")
        except urllib.error.HTTPError as exc:
            # 503 from /rec/start means "no frame yet" (mirrors /snap's own
            # 503) -- still a clean, specific reason rather than a stack
            # trace. /rec/stop never answers 503 (see stream_server.py).
            raise CommandError("12", f"CAMERA_REC_FAILED:HTTP_{exc.code}")
        except urllib.error.URLError as exc:
            raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")

    # -- GPS: live fix from link.gps_reader.GPSReader, if a receiver is
    #    attached -------------------------------------------------------
    def update_gps_fix(self, lat, lon, speed_kmh=None, cap=None, is_dgps=None):
        motor_output = None
        with self._lock:
            self.current_lat = lat
            self.current_lon = lon
            if speed_kmh is not None:
                self.speed_kmh = speed_kmh
            if cap is not None:
                self.cap = cap
            if is_dgps is not None:
                self.is_dgps = is_dgps
            self.last_fix_at = time.time()
            self._advance_route_if_arrived()
            if self.mode == "AUTO":
                motor_output = self._autonomous_pwm_locked()
                if motor_output is not None:
                    self.left_pwm, self.right_pwm = motor_output
        # motor_driver.drive() is called outside the lock, same pattern
        # as drive()/stop()/set_mode() above -- this class's own lock
        # never needs to be held while calling into MotorDriver (which
        # takes its own, separate lock).
        if motor_output is not None and self.motor_driver is not None:
            self.motor_driver.drive(*motor_output)

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

        distance_m = haversine_distance_m(
            self.current_lat, self.current_lon, target_lat_decimal, target_lon_decimal
        )
        if distance_m <= ROUTE_ARRIVAL_RADIUS_M:
            self.route_index += 1
            # New leg (or route just completed): start the PID loops
            # fresh so a stale integral/derivative from the leg that
            # just ended doesn't produce a jolt on the first tick of the
            # next one (or linger pointlessly once the route is done).
            self.autopilot.reset()
            if self.route_index < len(self.route):
                self.nav_target = self.route[self.route_index]
            # else: route complete -- nav_target is left on the last
            # waypoint and route_index stays at len(self.route) as the
            # "done" marker (see status() below).

    def _autonomous_pwm_locked(self):
        """Called with self._lock already held, only while mode == "AUTO".
        Returns (left_pwm, right_pwm) computed by self.autopilot from the
        live distance/heading to self.nav_target, or None if there's
        nothing to compute yet (no GPS fix, or no target at all -- AUTO
        armed with neither NAV nor RTE ever having been sent). Arrival at
        a lone NAV target (no route to advance into, see
        _advance_route_if_arrived above) is handled here rather than
        there, since a single NAV target has no "next waypoint" to
        advance to -- once within ROUTE_ARRIVAL_RADIUS_M with nothing
        left to chase, this holds position (0, 0) instead of letting the
        PID loops hunt around a target they've already reached."""
        if self.current_lat is None or self.current_lon is None or self.nav_target is None:
            return None

        target_lat, target_lat_dir, target_lon, target_lon_dir = self.nav_target
        target_lat_decimal = nmea_to_decimal(target_lat, target_lat_dir)
        target_lon_decimal = nmea_to_decimal(target_lon, target_lon_dir)
        if target_lat_decimal is None or target_lon_decimal is None:
            return None

        distance_m = haversine_distance_m(
            self.current_lat, self.current_lon, target_lat_decimal, target_lon_decimal
        )
        route_exhausted = not self.route or self.route_index >= len(self.route)
        if distance_m <= ROUTE_ARRIVAL_RADIUS_M and route_exhausted:
            self.autopilot.reset()
            return (0, 0)

        bearing = bearing_deg(self.current_lat, self.current_lon, target_lat_decimal, target_lon_decimal)
        heading_error = heading_error_for_pid(self.cap, bearing)
        return self.autopilot.compute(distance_m, heading_error)

    # -- gamepad support: is a manual drive command currently allowed to
    #    reach the motors? ---------------------------------------------
    def is_manual(self):
        """True if currently in MANUAL mode. Added 2026-09-12 alongside a
        real bug fix in link/gamepad_handler.py's robot_state_drive_handler():
        GamepadReader.on_drive fires on EVERY stick axis event, including
        the analog noise/jitter an idle, centered stick still produces --
        before this fix, that idle (0, 0) was forwarded to drive()
        unconditionally, which zeroes left_pwm/right_pwm and tells
        motor_driver to stop no matter what mode the robot was actually
        in. In AUTO mode that meant every idle-stick event silently
        cancelled whatever PWM update_gps_fix()'s autopilot tick had just
        computed a moment earlier -- indistinguishable from "AUTO mode
        never actually drives", which is exactly what was reported. The
        fix: an idle/centered stick now only reaches drive() while
        actually in MANUAL (so releasing the stick during a genuine
        manual drive still stops the motors normally); this method is
        what the gamepad's drive handler checks to tell the two cases
        apart."""
        with self._lock:
            return self.mode == "MANUAL"

    # -- gamepad support: is there anything to drive toward? ---------------
    def has_nav_target(self):
        """True once a target has been set via NAV or RTE (a route's first
        waypoint sets nav_target too -- see set_route() above). Used by
        the gamepad's "re-arm AUTO" button (BTN_Y, see
        link/gamepad_handler.py's robot_state_button_handler()) to refuse
        arming AUTO with nothing to drive toward, rather than switching
        into a mode that would just sit there computing nothing every GPS
        fix (see _autonomous_pwm_locked() above)."""
        with self._lock:
            return self.nav_target is not None

    # -- GRT: the currently active route (for robot-webserver's map) ------
    def get_route(self):
        """Thread-safe read of the currently active route (the last RTE
        upload, i.e. "GPS Driving" -- see set_route() above): a list of
        (lat, lat_dir, lon, lon_dir) tuples, already in this protocol's
        on-the-wire encoding, in the order they're chased. Empty once no
        route has ever been sent, or after a fresh NAV/STP cleared it (see
        set_nav_target()/stop()). Backs the GRT sentence (link/server.py),
        added 2026-09-19 for robot-webserver's /control map's yellow
        markers -- every other read of shared state in this class already
        goes through a lock-protected method (status(), has_nav_target()),
        this just extends that to self.route, which server.py used to read
        directly before this method existed. Returns a copy (list(...)),
        not the live list, so a caller holding onto the result can't end
        up seeing a route mutated out from under it by a later RTE/NAV/STP
        on another thread."""
        with self._lock:
            return list(self.route)

    # -- gamepad support: save the current GPS fix as a waypoint ------------
    def save_waypoint(self):
        """CAM,X on the gamepad (link/gamepad_handler.py's
        save_waypoint_btn, default BTN_X, see robot_state_button_handler())
        -- appends the robot's current live GPS fix to a plain-text
        waypoints file, one "lat,lon,timestamp" line per point, lat/lon in
        plain decimal degrees.

        Deliberately the SAME two-leading-columns shape robot-webserver's
        own "GPS Driving" file upload already parses (see that project's
        app.py/parseGpsRouteFile: one "lat,lon" per line, decimal degrees,
        extra columns and blank/"#" lines ignored) -- so a point saved
        here, copied off the robot, can be re-uploaded there as a route
        with zero conversion; the timestamp is just an extra column that
        upload already knows to ignore, kept for whoever reads the raw
        file later.

        Raises CommandError("14", "NO_GPS_FIX_YET") if there's no live fix
        yet (no GPS receiver attached, or it hasn't produced one yet) --
        there is nothing meaningful to save. File location is the
        WAYPOINTS_FILE environment variable if set, else
        DEFAULT_WAYPOINTS_FILE (<repo_root>/waypoints/waypoints.txt) --
        read at call time, not import time, same per-call-configurable
        pattern as CAMERA_HOST/CAMERA_PORT above. Returns the path written
        to, so callers (link/gamepad_handler.py's _on_button) can log
        exactly where the point went."""
        with self._lock:
            lat, lon = self.current_lat, self.current_lon
        if lat is None or lon is None:
            raise CommandError("14", "NO_GPS_FIX_YET")

        path = os.environ.get("WAYPOINTS_FILE", DEFAULT_WAYPOINTS_FILE)
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
        with open(path, "a") as f:
            f.write(f"{lat:.6f},{lon:.6f},{timestamp}\n")

        with self._lock:
            self.last_command_at = time.time()
        return path

    # -- WPT: list saved waypoints (for robot-webserver's map, 2026-09-19) --
    def list_waypoints(self):
        """Reads and parses the waypoints file (see save_waypoint() above)
        into a list of (lat, lon) decimal-degree tuples, in the order they
        were saved. Backs the WPT sentence (link/server.py) that
        robot-webserver's /control map polls for its blue markers
        (waypoints saved via the gamepad's X button, see
        robot_state_button_handler()'s save_waypoint_btn).

        Same tolerant, line-oriented parsing as robot-webserver's own GPS-
        route-file upload (parseGpsRouteFile in that project's app.py):
        blank lines and lines starting with "#" are skipped, and a line
        that isn't at least two comma-separated numbers is skipped too
        rather than raising -- this file is hand-editable, and a single
        stray line shouldn't take the whole WPT query down. Returns an
        empty list if the file doesn't exist yet (no waypoint saved so
        far), the same "nothing to report yet" convention has_nav_target()/
        status() already use elsewhere in this class.

        Deliberately does not take self._lock: this only reads a file on
        disk, never any of this instance's own in-memory state, so there's
        nothing here for that lock to protect (save_waypoint() itself only
        holds it while reading current_lat/current_lon, not while writing
        the file -- see its own comment)."""
        path = os.environ.get("WAYPOINTS_FILE", DEFAULT_WAYPOINTS_FILE)
        try:
            with open(path) as f:
                lines = f.readlines()
        except FileNotFoundError:
            return []

        points = []
        for raw_line in lines:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            points.append((lat, lon))
        return points

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
                "is_dgps": self.is_dgps,
                # Not (yet) part of the STA wire sentence -- nav_target
                # above already reflects route progress for anything
                # reading it today. Exposed here for tests and for a
                # future STA extension (extend-only, see
                # pages/protocole_controle.html) without needing another
                # change to this method's shape.
                "route_total": len(self.route),
                "route_index": self.route_index,
                # Not (yet) part of the STA wire sentence either -- same
                # extend-only reasoning as route_total/route_index above.
                # Exposed here now that REC_START/REC_STOP genuinely track
                # a real state (2026-09-18) for tests and any future STA
                # extension.
                "is_recording": self.is_recording,
            }
