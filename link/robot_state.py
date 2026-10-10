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

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.request

from link import solar_map
from link.autopilot import Autopilot, bearing_deg, haversine_distance_m, heading_error_for_pid
from link.cpu_temp import read_cpu_temperature_c
from link.uptime import read_uptime_s
from link.nmea import decimal_to_nmea, nmea_to_decimal
from link.power_history import delete_media_row, fetch_media_positions, log_media
from link.power_history import resolve_db_path as resolve_history_db_path
from link.power_history import system_clock_is_plausible

log = logging.getLogger("link.robot_state")

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

# Catches the single most common manual-NAV/RTE mistake (2026-10-07): an
# operator typing a point in plain decimal degrees -- e.g. copied straight
# from /control's own "current"/"target" status display, which IS decimal
# degrees (see robot-webserver's app.py), or from a map -- where this
# protocol's ddmm.mmmm wire format was expected. Before this existed, that
# mistake was accepted silently: nmea_to_decimal() has no way to tell "a
# valid-looking but wrong point" from a genuine one, so e.g. typing
# "47.391534,N" instead of "4723.492,N" for the SAME intended latitude
# decoded as roughly 0.79 degrees North -- ~45x off -- with no error
# anywhere, and nothing but a suspiciously-wrong "target" reading downstream
# to notice by.
#
# The check below is deliberately NOT a "this must be somewhere near
# France" geography box: this project's own protocol examples and test
# suite already use points as far as Munich (48.12N, 11.52E) and 6N/20E
# purely to exercise the sentence format, and a geography-based box would
# wrongly reject those. Instead it uses a fact that's true regardless of
# where the robot is: a GENUINE ddmm.mmmm latitude's raw numeric value is
# at least "degree * 100" -- so for any site more than roughly half a
# degree from the equator (every real site and every test fixture this
# project has), the raw value is comfortably >= 50. A raw magnitude below
# that can only be a plain decimal-degrees latitude (whose maximum
# possible magnitude is 90) typed where ddmm.mmmm was expected.
#
# Longitude can't carry this same check on its own: near the Greenwich
# meridian (degree 0), this protocol's own ddmm.mmmm longitude is
# numerically small too (e.g. "00044.340"), indistinguishable from decimal
# degrees by magnitude alone -- so the format decision for a whole point
# is made from latitude only and applied to both of its fields together.
# This never touches the robot's own live GPS fix (link/gps_reader.py's
# own code path, never this one) -- only an operator's/the website's own
# typed-in target. GPS Driving's route-file upload and the Distance+angle
# panel already build correct NMEA fields themselves and will simply
# never trip this; same mirrored check client-side in robot-webserver's
# app.py (normalizeGpsCommand) auto-converts the mistake away before it
# ever reaches here, so this is mainly a safety net for anything that
# talks to this TCP port directly, bypassing the website.
RAW_LAT_MAGNITUDE_FLOOR = 90.0


def _point_is_plausible(raw_lat):
    """True if `raw_lat` (the field as typed/received, BEFORE
    nmea_to_decimal()) has the numeric shape of a genuine ddmm.mmmm
    latitude rather than a plain decimal-degrees value typed by mistake --
    see RAW_LAT_MAGNITUDE_FLOOR's honesty note above. Callers already know
    `raw_lat` parses as a float (the existing BAD_LAT_LON_VALUE-style check
    runs first)."""
    return abs(float(raw_lat)) >= RAW_LAT_MAGNITUDE_FLOOR


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
        # True while self.route is a "waypoint return" route (BTN_A,
        # see start_waypoint_return() below) rather than an ordinary GPS
        # Driving upload (RTE) -- distinguishes the two so
        # _advance_route_if_arrived() knows whether reaching a waypoint
        # should also delete it from waypoints.txt (return routes: yes,
        # the whole point is to empty the saved-waypoint pile as the robot
        # retraces it; RTE routes: no, an uploaded route doesn't touch
        # waypoints.txt at all). Reset to False everywhere self.route is
        # cleared/replaced (stop(), set_nav_target(), set_route()) so a
        # later, unrelated route never inherits stale deletion behavior.
        self.route_is_return = False
        # Parallel to self.route while route_is_return is True: the exact
        # raw waypoints.txt line each entry in self.route came from (see
        # _read_waypoint_entries()/start_waypoint_return() below), so the
        # right line can be deleted by exact text match -- never by
        # recomputing/parsing it back from the wire-format fields, which
        # would risk a rounding mismatch against what's actually on disk.
        self._return_raw_lines = []
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
        # Last UTC date/time reported by the GPS receiver's own GPRMC
        # sentence, as a Unix timestamp -- None until at least one GPRMC
        # sentence with a valid fix has arrived (see link/gps_reader.py's
        # parse_fix(), which only fills this in for RMC, not GGA: GGA
        # carries a time-of-day but no date, so it alone can't produce a
        # full timestamp). Explicit user request (2026-10-10): PWR's
        # onboard_time field falls back to this whenever Pi #1's own
        # system clock still looks unset (see link.power_history.
        # system_clock_is_plausible) -- a GPS fix's UTC time is correct
        # from the moment a fix is acquired, independent of WiFi/NTP.
        self.last_gps_utc_ts = None
        # Solar-exposure survey (2026-10-07, explicit user request):
        # resolved once here -- not inside update_gps_fix(), which runs
        # on every single GPS fix -- so the SOLAR_SURVEY_TMP_PATH env
        # override is only ever read once per process, same reasoning as
        # DEFAULT_WAYPOINTS_FILE's own one-time resolution elsewhere in
        # this file. _solar_survey_last_point is the (lat, lon) of the
        # last point actually buffered (None until the first one), kept
        # here so update_gps_fix()'s "moved >=5m?" check survives across
        # calls -- see link.solar_map.should_record_point().
        self._solar_survey_tmp_path = solar_map.resolve_tmp_path()
        self._solar_survey_last_point = None
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
        # Bridges CAM,REC_START -> CAM,REC_STOP for video geotagging (see
        # _request_recording() below): the GPS fix live when REC_START
        # succeeded, held here until REC_STOP hands back the actual
        # filename (camera/recordings.py's VideoRecorder only assigns one
        # lazily, on its first written frame). None whenever no recording
        # is currently armed.
        self._recording_start_latlon = None
        # PWR (2026-10-03): latest EPever Tracer reading, set by
        # link.tracer_reader.TracerReader in the background -- same "live
        # hardware fix, set by a background reader" shape as current_lat/
        # current_lon above. None until the first successful read ever
        # arrives, so power_status() can report the honest "no cable
        # connected / not read yet" placeholder for as long as that's
        # true, exactly like current_lat/current_lon do for the GPS.
        self.pv_voltage = None
        self.pv_current = None
        self.pv_power = None
        self.battery_voltage = None
        self.battery_charging_current = None
        self.battery_charging_power = None
        self.load_voltage = None
        self.load_current = None
        self.load_power = None
        self.battery_soc = None
        self.battery_temp = None
        self.controller_temp = None
        # Tri-state, same reasoning as is_dgps above: None = never read
        # successfully (no cable, or the reader hasn't run yet), True =
        # the most recent poll succeeded, False = the most recent poll
        # failed (cable unplugged mid-run, Tracer unpowered, CRC error...)
        # -- distinguishing "never worked" from "was working, now isn't"
        # is worth keeping for the power page's status badge. The last
        # known-good values above are deliberately left untouched on a
        # failed poll rather than reset to None/0 -- a stale-but-plausible
        # reading is more useful on screen than a flicker back to zero.
        self.power_available = None
        self.last_power_reading_at = None

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
            self.route_is_return = False
            self._return_raw_lines = []
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
            else:
                # Bug fix (2026-10-10): leaving AUTO -- back to MANUAL (a
                # gamepad stick push, see
                # link.gamepad_handler.robot_state_drive_handler, or a
                # website DRV) or to IDLE via a plain MOD command -- now
                # cancels any route in progress, same "newest command
                # wins" convention NAV/RTE/STP already use elsewhere in
                # this class (see set_nav_target()/set_route()/stop()
                # above). Without this, a BTN_A waypoint-return route
                # (start_waypoint_return() below) kept reporting itself
                # as an active route (route_is_return stayed True, so
                # robot-webserver's /control map kept drawing the
                # waypoints as an active NAV/route rather than plain blue
                # points) AND kept being silently chased by
                # _advance_route_if_arrived() -- that method runs on
                # every GPS fix regardless of mode, so it went right on
                # advancing route_index and deleting "arrived" waypoints
                # from waypoints.txt even while the operator had already
                # taken back manual control. By the time BTN_A was
                # pressed again, the saved waypoints were already
                # partially or fully consumed, or the route already
                # looked exhausted, so start_waypoint_return() either had
                # nothing left to build a route from or simply re-armed
                # AUTO on a target already reached -- the robot never
                # actually moved. Clearing the route here (nav_target is
                # deliberately left alone, matching stop()'s own
                # precedent) means a fresh BTN_A press after returning to
                # MANUAL always starts a brand new return route from
                # whatever waypoints are still genuinely unvisited.
                self.route = []
                self.route_index = 0
                self.route_is_return = False
                self._return_raw_lines = []
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
        # Catches the single most common manual-NAV mistake: typing the
        # point in plain decimal degrees instead of this protocol's
        # ddmm.mmmm wire format -- see RAW_LAT_MAGNITUDE_FLOOR's honesty
        # note above for why this can't just be folded into the
        # BAD_LAT_LON_VALUE check above (both are "valid floats", just one
        # is the wrong shape for this field).
        if not _point_is_plausible(lat):
            raise CommandError("22", f"NAV_LAT_LON_IMPLAUSIBLE:{lat},{lat_dir},{lon},{lon_dir}")
        with self._lock:
            self.nav_target = (lat, lat_dir, lon, lon_dir)
            # A manual NAV takes back control from an active route -- without
            # this, the very next GPS fix's arrival check (see
            # _advance_route_if_arrived) could silently overwrite the
            # operator's manual target with whatever the route was pursuing.
            self.route = []
            self.route_index = 0
            self.route_is_return = False
            self._return_raw_lines = []
            self.last_command_at = time.time()
            # AUTO mode actually steers toward this target -- see
            # link.autopilot.Autopilot and update_gps_fix() below (2026-09-07,
            # see this module's own top-of-file docstring); this used to be
            # a TODO pointing at the old gps/pid_controller.py pipeline,
            # which this class never ended up calling into.

    # -- RTE: set an ordered list of GPS waypoints to chase automatically --
    def set_route(self, fields):
        """fields = [count, lat1, lat_dir1, lon1, lon_dir1, lat2, ...] --
        same per-point encoding as NAV, just repeated `count` times.
        Replaces any previous route and re-arms automatic advancement from
        the first waypoint. AUTO mode actually drives toward it (see
        link.autopilot.Autopilot, same as NAV -- this module's top-of-file
        docstring has the full story); nav_target is kept in sync with
        "the waypoint currently being pursued" so
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
            # Same plausibility check as NAV's own (see
            # RAW_LAT_MAGNITUDE_FLOOR's honesty note above) -- catches a
            # route file/point typed in plain decimal degrees by mistake,
            # one point at a time, with the field index (point_{i}) in the
            # error.
            if not _point_is_plausible(lat):
                raise CommandError("13", f"RTE_LAT_LON_IMPLAUSIBLE:point_{i}:{lat},{lat_dir},{lon},{lon_dir}")
            points.append((lat, lat_dir, lon, lon_dir))

        with self._lock:
            self.route = points
            self.route_index = 0
            # RTE ("GPS Driving") is priority-LOWER than an active
            # waypoint-return route (BTN_A, see start_waypoint_return()
            # below) in the other direction -- but a fresh RTE upload
            # itself always wins over whatever was running before,
            # including a return in progress, same "newest command always
            # wins" convention as NAV/STP above. route_is_return/
            # _return_raw_lines are cleared here so this new, ordinary
            # route is never mistaken for a return route (which would
            # wrongly start deleting waypoints.txt lines as it's chased).
            self.route_is_return = False
            self._return_raw_lines = []
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
        traceback; nothing here assumes the camera is actually available.

        2026-10-05: on success, also geotags the snapshot -- reads the
        filename back out of /snap's own JSON body (it already returns
        one, see camera/stream_server.py's _handle_snap) and records it
        against whatever GPS fix is live right now (link.power_history.
        log_snapshot) for pages/power.html's history section. This is
        deliberately best-effort: a geotagging failure (disk error, no
        GPS fix at all) must never turn a snapshot that the camera
        genuinely saved into a CAM,SNAP error for the console."""
        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        path = os.environ.get("CAMERA_SNAP_PATH", CAMERA_SNAP_PATH_DEFAULT)
        url = f"http://{host}:{port}{path}"
        try:
            with urllib.request.urlopen(url, timeout=CAMERA_SNAP_TIMEOUT) as resp:
                if resp.status != 200:
                    raise CommandError("12", f"CAMERA_SNAP_FAILED:HTTP_{resp.status}")
                body = resp.read()
        except urllib.error.HTTPError as exc:
            # 503 from _handle_snap means "no frame yet" (grabber hasn't
            # produced one, e.g. camera just starting up) -- still a clean,
            # specific reason rather than a stack trace.
            raise CommandError("12", f"CAMERA_SNAP_FAILED:HTTP_{exc.code}")
        except urllib.error.URLError as exc:
            raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")

        try:
            filename = json.loads(body).get("file")
            if filename:
                with self._lock:
                    lat, lon = self.current_lat, self.current_lon
                log_media(resolve_history_db_path(), filename, "photo", lat, lon)
        except Exception:
            # Best-effort geotagging (see docstring above) -- the photo
            # itself is already safely on disk by this point, so a
            # malformed JSON body or a DB hiccup here is logged and
            # swallowed rather than surfaced as a CAM,SNAP failure.
            log.exception("failed to geotag snapshot %r in power_history.db", body)

    def _request_recording(self, start: bool):
        """Asks camera/stream_server.py to start or stop writing the live
        feed to a video file, via its GET /rec/start or /rec/stop endpoint
        -- same cross-process HTTP call as _request_snapshot() above, just
        a different path and (for /rec/start) the same 503-means-no-frame-
        yet convention as /snap. That 503 is exactly what makes REC_START
        "record a video if the camera is present" in practice: no camera
        delivering frames yet is treated the same as no camera plugged in
        at all, and turns into a clean CommandError here rather than
        arming a recorder with nothing to encode.

        2026-10-05: also geotags the recording, at the GPS position live
        when REC_START succeeded (explicit user request: "geolocaliser les
        videos lors du start") -- but the actual DB write only happens
        once REC_STOP hands back the real filename (camera/recordings.py's
        VideoRecorder only assigns one lazily, on its first written frame,
        so REC_START's own response has nothing to key the geotag on yet;
        see stream_server.py's _handle_rec_stop). self._recording_start_latlon
        bridges the two calls: set here on a successful REC_START, read
        and cleared on the matching REC_STOP. Best-effort, same spirit as
        _request_snapshot()'s own geotagging -- a geotag failure must
        never turn a recording that genuinely started/stopped into a
        CAM,REC_START/REC_STOP error for the console."""
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
                body = resp.read()
        except urllib.error.HTTPError as exc:
            # 503 from /rec/start means "no frame yet" (mirrors /snap's own
            # 503) -- still a clean, specific reason rather than a stack
            # trace. /rec/stop never answers 503 (see stream_server.py).
            raise CommandError("12", f"CAMERA_REC_FAILED:HTTP_{exc.code}")
        except urllib.error.URLError as exc:
            raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")

        if start:
            with self._lock:
                self._recording_start_latlon = (self.current_lat, self.current_lon)
            return

        try:
            filename = json.loads(body).get("file")
            with self._lock:
                start_latlon = self._recording_start_latlon
                self._recording_start_latlon = None
            if filename and start_latlon is not None:
                lat, lon = start_latlon
                log_media(resolve_history_db_path(), filename, "video", lat, lon)
        except Exception:
            log.exception("failed to geotag recording %r in power_history.db", body)

    # -- GPS: live fix from link.gps_reader.GPSReader, if a receiver is
    #    attached -------------------------------------------------------
    def update_gps_fix(self, lat, lon, speed_kmh=None, cap=None, is_dgps=None, gps_utc_ts=None):
        motor_output = None
        # (ts, lat, lon, pv_power) to buffer to the solar-survey tmp file
        # once the lock is released, or None if this fix doesn't qualify
        # -- see the comment below.
        solar_survey_sample = None
        with self._lock:
            self.current_lat = lat
            self.current_lon = lon
            if speed_kmh is not None:
                self.speed_kmh = speed_kmh
            if cap is not None:
                self.cap = cap
            if is_dgps is not None:
                self.is_dgps = is_dgps
            if gps_utc_ts is not None:
                self.last_gps_utc_ts = gps_utc_ts
            self.last_fix_at = time.time()
            self._advance_route_if_arrived()
            if self.mode == "AUTO":
                motor_output = self._autonomous_pwm_locked()
                if motor_output is not None:
                    self.left_pwm, self.right_pwm = motor_output

            # Solar-exposure survey (2026-10-07, explicit user request):
            # buffer one (ts, lat, lon, pv_power) sample every time the
            # robot has moved >= link.solar_map.SOLAR_SURVEY_MIN_DISTANCE_M
            # since the last buffered point -- while driving via the
            # gamepad (MANUAL) or in AUTO, exactly like every other fix
            # handled here; no separate "is it actually driving" check is
            # needed, since this is only ever reached from a real GPS fix.
            #
            # Gating (the two explicit refinements the user agreed to):
            # (1) this fix is always genuinely valid already -- see
            # link.solar_map's own module docstring for why -- so nothing
            # extra is needed for that here; (2) only sample while
            # self.power_available is True, the exact same rule
            # link.power_history's power_log already applies, so a
            # disconnected Tracer cable never pairs a real position with
            # a stale/placeholder pv_power reading. (3) the system clock
            # must look plausible (2026-10-10, explicit user request) --
            # this Pi has no RTC battery, so right after boot and before
            # NTP has had a chance to correct the clock, time.time() can
            # read months/years off; buffering (and, in link.power_history,
            # eventually writing to the DB) a sample timestamped with that
            # fallback value would permanently burn in a wrong `ts` that
            # can never be fixed up once the clock corrects itself a
            # moment later. See link.power_history.system_clock_is_plausible
            # for the actual check -- same gate that module's own
            # _log_once() applies to power_log. Deliberately NOT updating
            # _solar_survey_last_point in that case either: the "moved
            # >=5m" distance gate should still measure from the last
            # point that was ACTUALLY buffered (or nothing, from the very
            # start) once the clock recovers, not from a point that was
            # silently skipped.
            if (
                self.power_available is True
                and system_clock_is_plausible()
                and solar_map.should_record_point(self._solar_survey_last_point, lat, lon)
            ):
                self._solar_survey_last_point = (lat, lon)
                solar_survey_sample = (self.last_fix_at, lat, lon, self.pv_power)
        # motor_driver.drive() and the solar-survey tmp-file append both
        # happen outside the lock, same reasoning as the drive() call
        # below already had: neither is RobotState's own in-memory state,
        # and file I/O (like motor I/O) has no business holding this lock.
        if motor_output is not None and self.motor_driver is not None:
            self.motor_driver.drive(*motor_output)
        if solar_survey_sample is not None:
            solar_map.append_point(self._solar_survey_tmp_path, *solar_survey_sample)

    # -- PWR: latest EPever Tracer reading, set by link.tracer_reader ----
    def update_power_reading(self, available, pv_voltage=None, pv_current=None,
                              pv_power=None, battery_voltage=None,
                              battery_charging_current=None, battery_charging_power=None,
                              load_voltage=None, load_current=None, load_power=None,
                              battery_soc=None, battery_temp=None, controller_temp=None):
        """Called by link.tracer_reader.TracerReader on every poll (success
        or failure). `available=False` (a failed poll -- cable unplugged,
        Tracer unpowered, a CRC/timeout error) only updates power_available
        and leaves every other field at its last known-good value -- see
        the __init__ comment above these fields for why a stale reading
        beats flickering back to a 0.0 placeholder. `available=True`
        always comes with every field actually filled in (see
        link.tracer_reader.read_power_snapshot, which is all-or-nothing by
        construction), so there's no partial-update case to handle here."""
        with self._lock:
            self.power_available = available
            if not available:
                return
            self.pv_voltage = pv_voltage
            self.pv_current = pv_current
            self.pv_power = pv_power
            self.battery_voltage = battery_voltage
            self.battery_charging_current = battery_charging_current
            self.battery_charging_power = battery_charging_power
            self.load_voltage = load_voltage
            self.load_current = load_current
            self.load_power = load_power
            self.battery_soc = battery_soc
            self.battery_temp = battery_temp
            self.controller_temp = controller_temp
            self.last_power_reading_at = time.time()

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
            arrived_index = self.route_index
            self.route_index += 1
            # Waypoint-return route (BTN_A, see start_waypoint_return()):
            # the waypoint just reached is permanently removed from
            # waypoints.txt here, at actual arrival -- not when the return
            # was started -- so the saved-waypoint "pile" genuinely empties
            # out one point at a time as the robot retraces it (explicit
            # user request). Best-effort: a file error here must never
            # block the route from advancing to its next leg.
            if self.route_is_return and arrived_index < len(self._return_raw_lines):
                try:
                    self._remove_waypoint_line(self._return_raw_lines[arrived_index])
                except Exception:
                    log.exception("failed to remove consumed waypoint from %s", DEFAULT_WAYPOINTS_FILE)
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

    # -- GRT's progress fields: for the /control waypoint-return/GPS-driving
    # progress bar (2026-10-07) ---------------------------------------------
    def get_route_progress(self):
        """Thread-safe atomic read of the currently active route together
        with how far into it the robot already is and whether it's a
        waypoint return (BTN_A) rather than an ordinary GPS Driving (RTE)
        route. Added for robot-webserver's /control progress bar, which
        needs the REMAINING points only (the caller slices route[
        route_index:] itself -- this still returns the FULL route, same
        shape as get_route(), so GRT's existing full-route map markers are
        unaffected) plus which colour/label to show.

        Returns (route, route_index, route_is_return) as one atomic
        snapshot under a single lock acquisition -- reading route via
        get_route() and route_is_return via a second, separate lock
        acquisition could otherwise observe a route cleared/replaced by a
        concurrent RTE/NAV/STP in between the two reads (e.g. "route"
        reflects the old route but "route_is_return" already reflects the
        new one). Returns a copy of route, not the live list, same
        reasoning as get_route()."""
        with self._lock:
            return list(self.route), self.route_index, self.route_is_return

    # -- RTD: delete one point from the active route (2026-10-06) -----------
    def delete_route_point(self, index):
        """Deletes the `index`'th point of the currently active route
        in-memory (0-based, same order self.route/get_route()/GRT already
        report) -- backs the /control map's right-click "delete this
        point" on a red marker, same spirit as delete_waypoint() (blue)
        and delete_media() (violet) above, extended to the last marker
        colour that didn't have one yet (explicit user request).

        Unlike delete_waypoint() (a plain text file on disk, untouched by
        any in-flight driving decision), self.route is live navigation
        state read on every GPS fix by _advance_route_if_arrived()/
        _autonomous_pwm_locked() above, so this needs the lock and a bit
        of index bookkeeping to stay consistent with whatever leg is
        currently being chased:
          - a point already behind the robot (index < route_index) is
            simply dropped; route_index shifts down by one so it still
            names the same upcoming leg it did before the deletion.
          - the point currently being chased (index == route_index) is
            dropped too; nav_target moves on to whatever is now at that
            same index (the old next point) -- or, if the route is now
            empty, nav_target is simply left as-is (the just-deleted
            point's own coordinates), same "done" convention
            _advance_route_if_arrived() already uses once a route
            finishes rather than resetting it to None.
          - a point further ahead (index > route_index) is dropped
            without touching route_index/nav_target at all -- the robot
            keeps chasing exactly what it was already chasing.
        self.autopilot is reset on the middle case only, same "fresh PID
        state for a new leg" reasoning as _advance_route_if_arrived().

        If this is a waypoint-return route (route_is_return), the
        matching entry in _return_raw_lines is dropped in lockstep so a
        later arrival still deletes the right line from waypoints.txt
        (see _advance_route_if_arrived()) -- deliberately does NOT touch
        waypoints.txt itself here: this only edits the live route held in
        memory, the saved waypoint this point came from is left on disk
        exactly as WPD would leave it (explicit user request -- WPD is
        the only thing that ever edits that file).

        Raises CommandError("21", "ROUTE_INDEX_OUT_OF_RANGE:<index>") if
        there's no such point right now (already consumed by the robot
        arriving at it, already deleted by a concurrent request, or the
        route was replaced/cleared by a fresh RTE/NAV/STP in the
        meantime)."""
        with self._lock:
            if index < 0 or index >= len(self.route):
                raise CommandError("21", f"ROUTE_INDEX_OUT_OF_RANGE:{index}")
            del self.route[index]
            if self.route_is_return and index < len(self._return_raw_lines):
                del self._return_raw_lines[index]
            if index < self.route_index:
                self.route_index -= 1
            elif index == self.route_index:
                self.autopilot.reset()
                if self.route_index < len(self.route):
                    self.nav_target = self.route[self.route_index]
                # else: route now empty -- nav_target left as-is, see
                # docstring above.
            # else (index > self.route_index): a point further ahead than
            # what's currently being chased -- nothing else to update.
            self.last_command_at = time.time()

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
    def _read_waypoint_entries(self):
        """Reads and parses the waypoints file (see save_waypoint() above)
        into a list of (lat, lon, raw_line) tuples, in the order they were
        saved -- `raw_line` (the exact original line, newline included) is
        kept so a specific entry can later be deleted by exact text match
        (see _remove_waypoint_line() below) without ever having to
        recompute/re-format it from the parsed float and risk a mismatch
        against what's actually on disk.

        Same tolerant, line-oriented parsing as robot-webserver's own GPS-
        route-file upload (parseGpsRouteFile in that project's app.py):
        blank lines and lines starting with "#" are skipped, and a line
        that isn't at least two comma-separated numbers is skipped too
        rather than raising -- this file is hand-editable, and a single
        stray line shouldn't take the whole WPT query (or a waypoint
        return, see start_waypoint_return() below) down. Returns an empty
        list if the file doesn't exist yet (no waypoint saved so far), the
        same "nothing to report yet" convention has_nav_target()/status()
        already use elsewhere in this class.

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

        entries = []
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
            entries.append((lat, lon, raw_line))
        return entries

    def list_waypoints(self):
        """Backs the WPT sentence (link/server.py) that robot-webserver's
        /control map polls for its blue markers (waypoints saved via the
        gamepad's X button, see robot_state_button_handler()'s
        save_waypoint_btn) -- just the (lat, lon) pairs, see
        _read_waypoint_entries() above for the actual file parsing."""
        return [(lat, lon) for lat, lon, _raw_line in self._read_waypoint_entries()]

    def _remove_waypoint_line(self, raw_line):
        """Deletes the FIRST line equal to `raw_line` (as returned by
        _read_waypoint_entries() above, newline included) from the
        waypoints file -- called from _advance_route_if_arrived() as each
        waypoint of an active return route (see start_waypoint_return()
        below) is actually reached, so the saved-waypoint "pile" genuinely
        empties out as the robot retraces it. A no-op if the line is no
        longer present (already removed, or the file was hand-edited in
        the meantime) rather than an error -- same "best-effort against a
        hand-editable file" spirit as _read_waypoint_entries() itself.
        Deliberately does not take self._lock, same reasoning as
        _read_waypoint_entries() (file I/O only, no in-memory state)."""
        path = os.environ.get("WAYPOINTS_FILE", DEFAULT_WAYPOINTS_FILE)
        try:
            with open(path) as f:
                lines = f.readlines()
        except FileNotFoundError:
            return
        try:
            lines.remove(raw_line)
        except ValueError:
            return
        with open(path, "w") as f:
            f.writelines(lines)

    # -- WPD: delete one saved waypoint (for robot-webserver's map, 2026-10-05) --
    def delete_waypoint(self, index):
        """Deletes the `index`'th saved waypoint (0-based, same order
        _read_waypoint_entries()/list_waypoints()/WPT already report) from
        waypoints.txt -- backs the /control map's right-click "delete this
        point" on a blue marker. Every OTHER line in the file (including
        comments/blank lines, and any entry that fails to parse) is kept
        byte-for-byte as-is; only the one PARSED entry at this position is
        dropped. The map's connecting segments reform on their own once
        the page re-polls WPT -- removing one entry from the middle of the
        list just means the next poll's "join consecutive points" loop
        (robot-webserver's app.py) connects what's now its new neighbour,
        no special-casing needed here.

        Raises CommandError("18", "WAYPOINT_INDEX_OUT_OF_RANGE:<index>") if
        there's no such entry -- the file is shorter than the caller
        thinks (already deleted by a concurrent request, or hand-edited in
        the meantime). Deliberately does not take self._lock, same
        reasoning as _read_waypoint_entries()/_remove_waypoint_line()
        above (file I/O only, no in-memory state)."""
        path = os.environ.get("WAYPOINTS_FILE", DEFAULT_WAYPOINTS_FILE)
        try:
            with open(path) as f:
                lines = f.readlines()
        except FileNotFoundError:
            raise CommandError("18", f"WAYPOINT_INDEX_OUT_OF_RANGE:{index}")

        # Same tolerant parsing rule as _read_waypoint_entries() above,
        # kept in sync with it deliberately (both must agree on which
        # lines count as "entry number N") -- tracks each kept entry's
        # position in `lines` so exactly one can be removed by index
        # without touching any other line's text.
        parsed_line_indices = []
        for i, raw_line in enumerate(lines):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                float(parts[0])
                float(parts[1])
            except ValueError:
                continue
            parsed_line_indices.append(i)

        if index < 0 or index >= len(parsed_line_indices):
            raise CommandError("18", f"WAYPOINT_INDEX_OUT_OF_RANGE:{index}")

        del lines[parsed_line_indices[index]]
        with open(path, "w") as f:
            f.writelines(lines)

    # -- BTN_A (extended 2026-10-05): drive back through saved waypoints --
    def start_waypoint_return(self):
        """Builds a route from every waypoint saved so far (gamepad's X
        button, see save_waypoint()/_read_waypoint_entries() above) and
        arms AUTO to drive back through them in REVERSE save order -- last
        saved first, back to the very first -- "retrace its steps" home.
        This OVERRIDES/cancels whatever GPS Driving route (RTE) was active
        (self.route is simply replaced below), same "newest command always
        wins" convention NAV/RTE/STP already use elsewhere in this class --
        explicit user requirement: the return route is priority over an
        active GPS Driving route.

        Each waypoint is permanently DELETED from waypoints.txt the moment
        the robot actually arrives at it (see _advance_route_if_arrived()
        above), not when this is called -- so the saved-waypoint "stack"
        genuinely empties out as the robot retraces it (explicit user
        request). This method only ever snapshots the file once, right
        here: a fresh X-button press mid-return saves a new point that
        simply isn't part of the route already under way.

        Raises CommandError("14", "NO_GPS_FIX_YET") -- same code
        save_waypoint() uses -- if there's no live GPS fix (nothing to
        navigate with), and CommandError("17", "NO_WAYPOINTS_SAVED") if
        the file has no usable entries. Both are caught by
        link/gamepad_handler.py's BTN_A handler, which falls back to the
        previous plain "re-arm AUTO if there's already a nav_target"
        behavior in either case -- see that module. Returns the number of
        waypoints now queued."""
        with self._lock:
            have_fix = self.current_lat is not None and self.current_lon is not None
        if not have_fix:
            raise CommandError("14", "NO_GPS_FIX_YET")

        entries = list(reversed(self._read_waypoint_entries()))
        if not entries:
            raise CommandError("17", "NO_WAYPOINTS_SAVED")

        route_points = []
        raw_lines = []
        for lat, lon, raw_line in entries:
            lat_str, lat_dir = decimal_to_nmea(lat, is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(lon, is_longitude=True)
            route_points.append((lat_str, lat_dir, lon_str, lon_dir))
            raw_lines.append(raw_line)

        with self._lock:
            self.route = route_points
            self.route_index = 0
            self.route_is_return = True
            self._return_raw_lines = raw_lines
            self.nav_target = route_points[0]
            # Same effect as set_mode("AUTO") (fresh PID state, PWM
            # untouched since AUTO never zeroes it) without re-entering
            # that method and re-acquiring self._lock -- this block
            # already holds it.
            self.mode = "AUTO"
            self.autopilot.reset()
            self.last_command_at = time.time()
        return len(route_points)

    # -- MED: geotagged photos/videos (for robot-webserver's map, 2026-10-05) --
    def media_positions(self):
        """Returns [(filename, kind, lat, lon, ts), ...] for every photo
        (CAM,SNAP) or video (CAM,REC_START/STOP) currently still on disk
        in the camera process's own capped-5-FIFO stores (see
        camera/snapshots.py's SnapshotStore / camera/recordings.py's
        VideoRecorder), cross-referenced against this process's own
        geotag DB (link.power_history) -- a file that's been rotated out
        of those stores never comes back here (see fetch_media_positions's
        own docstring for why that filtering happens in the DB layer).
        Backs the MED sentence (link/server.py) that robot-webserver's
        /control map polls for its violet markers.

        Best-effort, same "camera is optional" spirit as
        _request_snapshot()/_request_recording(): if the camera process is
        unreachable, returns an empty list rather than raising -- a MED
        query should never fail the whole map just because the camera
        script isn't running."""
        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        filenames = []
        kind_by_filename = {}
        for label, path, kind in (
            ("snapshots", "/snapshots", "photo"),
            ("recordings", "/recordings", "video"),
        ):
            url = f"http://{host}:{port}{path}"
            try:
                with urllib.request.urlopen(url, timeout=CAMERA_SNAP_TIMEOUT) as resp:
                    names = json.loads(resp.read()).get(label, [])
            except Exception:
                log.exception("failed to list %s from camera process for MED", label)
                continue
            for name in names:
                filenames.append(name)
                kind_by_filename[name] = kind

        if not filenames:
            return []
        rows = fetch_media_positions(resolve_history_db_path(), filenames)
        # kind_by_filename (the live store the file is CURRENTLY in) wins
        # over the DB's own `kind` column -- the two only disagree if a
        # filename were ever somehow reused across stores, which can't
        # happen in practice (see the "snap_"/"rec_" prefix comment on
        # power_history.py's schema), but preferring the live answer costs
        # nothing and is the more honest one if it ever did.
        return [
            (filename, kind_by_filename.get(filename, db_kind), lat, lon, ts)
            for filename, db_kind, lat, lon, ts in rows
        ]

    # -- MDD: delete one photo/video (for robot-webserver's map, 2026-10-05) --
    def delete_media(self, filename, kind):
        """Deletes one photo/video from BOTH the camera process's own
        capped store (camera/snapshots.py's SnapshotStore or
        camera/recordings.py's VideoRecorder, over a new HTTP DELETE to
        camera/stream_server.py -- the same cross-process split SNAP/
        REC_START/REC_STOP already use) AND this process's own geotag
        row for it (link.power_history) -- backs the /control map's
        right-click "delete this point" on a violet marker, which the
        website only sends after the user confirms a popup (robot-
        webserver's app.py), since unlike a waypoint this also removes
        the underlying file.

        `kind` is "photo" or "video" (same vocabulary as
        media_positions()). Raises CommandError("19",
        "BAD_MEDIA_KIND:<kind>") for anything else.

        The DB row is cleaned up on a best-effort basis regardless of
        whether the camera reports the file was actually still there --
        once this call is made, there is deliberately no reason for a
        geotag row describing a deliberately-deleted file to linger
        (media_positions() already tolerates a DB row outliving its file,
        see fetch_media_positions()'s own docstring, but there's no
        reason to leave one around on purpose). Raises CommandError("20",
        "MEDIA_NOT_FOUND:<filename>") if the camera process answers but
        says the file wasn't there (already rotated out by the normal
        FIFO cap, or already deleted) -- same "best-effort against a
        store that can empty itself" spirit as _remove_waypoint_line().
        Raises CommandError("12", "CAMERA_UNAVAILABLE:...") -- same code
        _request_snapshot()/_request_recording() use -- if the camera
        process can't be reached at all; the DB row is deliberately left
        alone in that case, since the file might still genuinely exist."""
        if kind not in ("photo", "video"):
            raise CommandError("19", f"BAD_MEDIA_KIND:{kind}")

        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        remote_path = "/snapshots/" if kind == "photo" else "/recordings/"
        url = f"http://{host}:{port}{remote_path}{filename}"
        request = urllib.request.Request(url, method="DELETE")
        try:
            with urllib.request.urlopen(request, timeout=CAMERA_SNAP_TIMEOUT) as resp:
                body = json.loads(resp.read())
            deleted = bool(body.get("ok"))
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                deleted = False
            else:
                raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")
        except urllib.error.URLError as exc:
            raise CommandError("12", f"CAMERA_UNAVAILABLE:{exc.reason}")

        try:
            delete_media_row(resolve_history_db_path(), filename)
        except Exception:
            log.exception("failed to remove DB row for deleted media %s", filename)

        if not deleted:
            raise CommandError("20", f"MEDIA_NOT_FOUND:{filename}")

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
                # Now part of GRT's wire sentence (2026-10-07, see
                # get_route_progress()/server.py's GRT handler) -- kept here
                # too since every other piece of shared state already goes
                # through this dict, and tests read it from here rather
                # than reaching into self.route_is_return directly.
                "route_is_return": self.route_is_return,
                # Not (yet) part of the STA wire sentence either -- same
                # extend-only reasoning as route_total/route_index above.
                # Exposed here now that REC_START/REC_STOP genuinely track
                # a real state (2026-09-18) for tests and any future STA
                # extension.
                "is_recording": self.is_recording,
            }

    # -- PWR: solar/battery/load status snapshot for the power page --------
    def power_status(self):
        # cpu_temp is read fresh here rather than cached on self like the
        # Tracer fields above: it's Pi #1's own CPU, not something
        # link.tracer_reader polls over RS485, so it has nothing to do
        # with power_available (still reported even if the Tracer cable
        # is unplugged) and no background thread to keep it updated --
        # see link.cpu_temp for why a fresh read is cheap enough to just
        # do here, on every PWR request. Read outside the lock since it's
        # a plain file read with nothing to do with RobotState itself.
        cpu_temp = read_cpu_temperature_c()
        # uptime_s (2026-10-10, explicit user request): same "fresh read,
        # outside the lock" treatment as cpu_temp above -- see link.uptime
        # for why this is immune to the clock-not-synced problem
        # onboard_time (below) can hit.
        uptime_s = read_uptime_s()
        with self._lock:
            return {
                "pv_voltage": self.pv_voltage,
                "pv_current": self.pv_current,
                "pv_power": self.pv_power,
                "battery_voltage": self.battery_voltage,
                "battery_charging_current": self.battery_charging_current,
                "battery_charging_power": self.battery_charging_power,
                "load_voltage": self.load_voltage,
                "load_current": self.load_current,
                "load_power": self.load_power,
                "battery_soc": self.battery_soc,
                "battery_temp": self.battery_temp,
                "controller_temp": self.controller_temp,
                "cpu_temp": cpu_temp,
                "available": self.power_available,
                "uptime_s": uptime_s,
                # Last known UTC date/time reported by the GPS receiver's
                # own RMC sentence (set in update_gps_fix() below) --
                # explicit user request (2026-10-10): link/server.py's PWR
                # handler falls back to this for onboard_time whenever
                # system_clock_is_plausible() says Pi #1's own system
                # clock still looks unset, since a GPS fix's UTC time is
                # correct the moment a fix is acquired, with no dependency
                # on WiFi/NTP at all. None until at least one GPRMC
                # sentence with a valid fix has been seen.
                "gps_utc_ts": self.last_gps_utc_ts,
            }
