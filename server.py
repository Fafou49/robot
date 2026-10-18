"""TCP control server for the robot (Raspberry Pi #1): receives NMEA-style
sentences from the web server (Raspberry Pi #2) over the local WiFi
network, one sentence per line, and replies with ACK/ERR/STA sentences.

Run with:
    python3 -m link            # binds 0.0.0.0:5050 by default
    CONTROL_HOST=0.0.0.0 CONTROL_PORT=5050 python3 -m link

Protocol reference: pages/protocole_controle.html in the robot-webserver
project documents every sentence type this dispatch table implements.
"""

import logging
import os
import shlex
import socketserver
import subprocess
import time

from link.camera_power_policy import (
    BATTERY_CAPACITY_AH as CAMERA_POWER_BATTERY_CAPACITY_AH,
    LOW_PV_SUSTAINED_RECORDS as CAMERA_POWER_LOW_PV_RECORDS,
    LOW_PV_THRESHOLD_W as CAMERA_POWER_LOW_PV_THRESHOLD_W,
    PROJECTION_INTERVAL_S as CAMERA_POWER_PROJECTION_INTERVAL_S,
    SAFETY_MARGIN_HOURS as CAMERA_POWER_SAFETY_MARGIN_HOURS,
    SAMPLE_INTERVAL_S as CAMERA_POWER_SAMPLE_INTERVAL_S,
    CameraPowerPolicy,
)
from link.gamepad_handler import GamepadReader, robot_state_button_handler, robot_state_drive_handler
from link.gps_reader import DEFAULT_BAUDRATE, DEFAULT_DEVICE, GPSReader
from link.led_control import (
    LED_CHECK_INTERVAL_S,
    LOW_VOLTAGE_DURATION_S,
    LOW_VOLTAGE_THRESHOLD_V,
    LedController,
)
from link.nmea import SentenceError, build_sentence, decimal_to_nmea, parse_sentence
from link.power_history import (
    DEFAULT_LOG_INTERVAL_S as HISTORY_DEFAULT_LOG_INTERVAL_S,
    DEFAULT_RETENTION_DAYS as HISTORY_DEFAULT_RETENTION_DAYS,
    HIS_PERIODS,
    PowerHistoryLogger,
    fetch_obstacle_chunk,
    fetch_period_chunk,
    fetch_solar_map_chunk,
    fetch_stats_summary,
    resolve_db_path,
    system_clock_is_plausible,
)
from link.robot_state import CommandError, RobotState
from link.shutdown_policy import (
    BATTERY_CAPACITY_AH as SHUTDOWN_BATTERY_CAPACITY_AH,
    PROJECTION_INTERVAL_S as SHUTDOWN_PROJECTION_INTERVAL_S,
    SAFETY_MARGIN_HOURS as SHUTDOWN_SAFETY_MARGIN_HOURS,
    SAMPLE_INTERVAL_S as SHUTDOWN_SAMPLE_INTERVAL_S,
    SHUTDOWN_LEAD_HOURS,
    ShutdownPolicy,
)
from link.tracer_reader import (
    DEFAULT_BAUDRATE as TRACER_DEFAULT_BAUDRATE,
    DEFAULT_DEVICE as TRACER_DEFAULT_DEVICE,
    DEFAULT_POLL_INTERVAL_S as TRACER_DEFAULT_POLL_INTERVAL_S,
    DEFAULT_SLAVE_ID as TRACER_DEFAULT_SLAVE_ID,
    TracerReader,
)
from link.voltage_safety import (
    CRITICAL_VOLTAGE_THRESHOLD_V,
    RECOVERY_VOLTAGE_THRESHOLD_V,
    VOLTAGE_SAFETY_CHECK_INTERVAL_S,
    VoltageSafety,
)
from motor_control.motor_driver import MotorDriver

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("link.server")

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5050

# What BTN_START on the gamepad runs to power off the Raspberry Pi (see
# ControlServer._shutdown_pi() below and link/gamepad_handler.py's
# robot_state_button_handler()). Overridable via SHUTDOWN_CMD for testing
# or a different init system; whatever user runs run_robot.sh needs
# passwordless sudo for exactly this command -- see this project's
# README ("Pilotage moteur et manette") for the one-time sudoers setup.
SHUTDOWN_CMD = shlex.split(os.environ.get("SHUTDOWN_CMD", "sudo poweroff"))

# Which gamepad button does what, by evdev.ecodes attribute name -- see
# link/gamepad_handler.py's robot_state_button_handler() docstring for
# the full rationale (2026-09-12, extended 2026-09-18) and for why these
# are overridable at all: this project's actual controller/receiver may
# report a physical button under a different evdev code than its label
# suggests (a field report that pressing "Y" behaves like "A" -- see that
# module's "BUTTON MAPPING" comment). Run `python3 -m
# motor_control.dump_gamepad_buttons` on the Pi, press each physical
# button in turn, and set whichever variable below (in .env, see
# .env.example) doesn't match its default if the real code differs.
#
# 2026-09-18 right-cluster remap: A arms AUTO (was Y), B toggles video
# recording (new), X saves the current GPS fix to a waypoints file (new),
# Y takes a camera snapshot (new). GAMEPAD_STOP_BTN is unset (None) by
# default -- B used to be a dedicated full-stop button, but now records
# video instead; see robot_state_button_handler()'s docstring for the
# safety reasoning and how to rebind a stop button (e.g. to a shoulder
# button/bumper) if wanted.
GAMEPAD_ARM_AUTO_BTN = os.environ.get("GAMEPAD_ARM_AUTO_BTN", "BTN_A")
GAMEPAD_RECORD_BTN = os.environ.get("GAMEPAD_RECORD_BTN", "BTN_B")
GAMEPAD_SAVE_WAYPOINT_BTN = os.environ.get("GAMEPAD_SAVE_WAYPOINT_BTN", "BTN_X")
GAMEPAD_SNAPSHOT_BTN = os.environ.get("GAMEPAD_SNAPSHOT_BTN", "BTN_Y")
GAMEPAD_SHUTDOWN_BTN = os.environ.get("GAMEPAD_SHUTDOWN_BTN", "BTN_START")
GAMEPAD_STOP_BTN = os.environ.get("GAMEPAD_STOP_BTN") or None

# How long the gamepad buzzes when the live GPS fix's quality changes
# (see ControlServer._on_gps_quality_change() and
# link.gamepad_handler.GamepadReader.pulse()) -- strong for
# DGPS-acquired, weak for DGPS-lost, both this same duration. 0.5s is
# long enough to clearly notice, short enough to stay a "ping" rather
# than a "the controller is stuck buzzing".
DGPS_PULSE_DURATION_S = 0.5


def _handle_sentence(state: RobotState, sentence_type: str, fields: list, server=None) -> str:
    """Dispatches one parsed sentence to a RobotState method and returns
    the response sentence (ACK/STA) to send back. Raises CommandError
    (caught by the caller) for invalid input or an unimplemented command.

    `server` (2026-10-12, added for the new SHD sentence below) is the
    owning ControlServer instance -- optional and unused by every other
    sentence type, which only ever need `state`. Passed in rather than
    reached via some module-level global so this function stays a plain,
    easily-testable dispatcher; ControlHandler.handle() is the only
    caller and always has `self.server` on hand to pass."""
    if sentence_type == "STP":
        state.stop()
        return build_sentence("ACK", "STP")

    if sentence_type == "SHD":
        # Command (2026-10-12, explicit user request): manual "shut down
        # Pi #1 now" from robot-webserver's /control page, the button
        # placed directly under STOP -- same underlying mechanism as the
        # gamepad's BTN_START (see ControlServer._shutdown_pi() and
        # robot_state_button_handler()'s on_shutdown callback below).
        # `server` must be the owning ControlServer -- it always is in
        # production (ControlHandler.handle() passes self.server on
        # every call); only a direct unit-test call to this function
        # with server left at its default None could hit the guard
        # below, which fails loudly rather than crashing on
        # AttributeError.
        if server is None:
            raise CommandError("25", "SHD_UNAVAILABLE")
        # Same "call from a thread other than serve_forever()" rule
        # _shutdown_pi() documents -- ControlHandler instances run on
        # their own per-connection worker thread (ThreadingTCPServer),
        # which satisfies it, exactly like the gamepad handler's own
        # thread does.
        server._shutdown_pi()
        return build_sentence("ACK", "SHD")

    if sentence_type == "DRV":
        if len(fields) != 2:
            raise CommandError("10", "DRV_NEEDS_2_FIELDS")
        # A manual DRV always asserts direct control -- same "manual
        # override always wins" convention NAV/STP already use for the
        # route (see link/robot_state.py). Set BEFORE drive() so an
        # autonomous tick racing on another thread (link.gps_reader's
        # background thread, see RobotState.update_gps_fix) can't slot in
        # between the mode switch and the requested pwm actually landing.
        state.set_mode("MANUAL")
        state.drive(*fields)
        return build_sentence("ACK", "DRV")

    if sentence_type == "MOD":
        if len(fields) != 1:
            raise CommandError("10", "MOD_NEEDS_1_FIELD")
        state.set_mode(*fields)
        return build_sentence("ACK", "MOD")

    if sentence_type == "NAV":
        if len(fields) != 4:
            raise CommandError("10", "NAV_NEEDS_4_FIELDS")
        state.set_nav_target(*fields)
        return build_sentence("ACK", "NAV")

    if sentence_type == "RTE":
        # Field count isn't fixed here (depends on how many waypoints the
        # route has) -- state.set_route() validates the leading count field
        # against the rest and raises CommandError("13", ...) for anything
        # inconsistent, so there's nothing more to check before calling it.
        if len(fields) < 1:
            raise CommandError("10", "RTE_NEEDS_COUNT_AND_POINTS")
        state.set_route(fields)
        return build_sentence("ACK", "RTE")

    if sentence_type == "PID":
        if len(fields) != 4:
            raise CommandError("10", "PID_NEEDS_4_FIELDS")
        state.set_pid_gains(*fields)
        return build_sentence("ACK", "PID")

    if sentence_type == "CAM":
        if len(fields) != 1:
            raise CommandError("10", "CAM_NEEDS_1_FIELD")
        # SNAP calls out to camera/stream_server.py's /snap endpoint (see
        # RobotState.camera_command) and can genuinely succeed; REC_START/
        # REC_STOP still always raise (not implemented).
        state.camera_command(*fields)
        return build_sentence("ACK", "CAM")

    if sentence_type == "WPT":
        # Query, no fields: returns every waypoint saved so far (gamepad's
        # X button, see RobotState.save_waypoint()/list_waypoints()) for
        # robot-webserver's /control map (2026-09-19) -- its blue markers.
        # Same "count then count*4 fields" shape as RTE's own request, so
        # the website can reuse one parser for both; unlike the route,
        # decimal degrees straight out of the waypoints file need
        # converting to this protocol's ddmm.mmmm+direction pairs first.
        points = state.list_waypoints()
        fields = [len(points)]
        for lat, lon in points:
            lat_str, lat_dir = decimal_to_nmea(lat, is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(lon, is_longitude=True)
            fields.extend([lat_str, lat_dir, lon_str, lon_dir])
        return build_sentence("WPT", *fields)

    if sentence_type == "WPD":
        # Command (2026-10-05): deletes one saved waypoint by its 0-based
        # position in WPT's own listing -- backs robot-webserver's
        # /control map's right-click "delete this point" on a blue
        # marker. See RobotState.delete_waypoint() for why index rather
        # than lat/lon (no float round-trip to go wrong against what's
        # actually in waypoints.txt).
        if len(fields) != 1:
            raise CommandError("10", "WPD_NEEDS_1_FIELD")
        try:
            index = int(fields[0])
        except (TypeError, ValueError):
            raise CommandError("18", f"WAYPOINT_INDEX_OUT_OF_RANGE:{fields[0]}")
        state.delete_waypoint(index)
        return build_sentence("ACK", "WPD")

    if sentence_type == "GRT":
        # Query, no fields: returns the currently active route (the last
        # RTE upload, i.e. "GPS Driving", OR an in-progress waypoint
        # return -- see RobotState.set_route()/start_waypoint_return()) for
        # robot-webserver's /control map (2026-09-19) -- one of its red
        # markers (the map's own colour scheme groups GPS Driving and NAV
        # sent together, see that project's app.py). self.route already
        # holds each point pre-encoded as (lat, lat_dir, lon, lon_dir)
        # exactly like RTE's own fields, so this just flattens it back out
        # -- no further conversion needed. Empty (count 0, no point
        # fields, route_index 0, mode "DRIVE") once no route has been sent
        # yet or after STP/a fresh NAV cleared it (see
        # set_nav_target()/stop()).
        #
        # Two trailing fields (2026-10-07, extend-only -- an older
        # robot-webserver that doesn't know about them yet just ignores
        # them, same convention as STA's DGPS field and PWR's cpu_temp):
        # route_index (0-based, how far into the route list above the
        # robot already is -- route[route_index:] is what's actually left
        # to drive; points before that are already behind the robot but
        # still listed here so the map's full-route markers are
        # unaffected) and RETURN/DRIVE (whether this route is a waypoint
        # return (BTN_A) or an ordinary GPS Driving (RTE) upload -- backs
        # robot-webserver's /control progress bar, which colours/labels
        # itself accordingly). Both come from get_route_progress() as one
        # atomic read so they can never describe two different routes.
        route, route_index, route_is_return = state.get_route_progress()
        fields = [len(route)]
        for point in route:
            fields.extend(point)
        fields.append(route_index)
        fields.append("RETURN" if route_is_return else "DRIVE")
        return build_sentence("GRT", *fields)

    if sentence_type == "RTD":
        # Command (2026-10-06): deletes one point of the currently active
        # route by its 0-based position in GRT's own listing -- backs the
        # /control map's right-click "delete this point" on a red marker,
        # same spirit as WPD (blue) and MDD (violet) above, the last of
        # the three marker colours to get one. See
        # RobotState.delete_route_point() for the live-navigation-state
        # bookkeeping this needs that WPD (a plain file on disk) doesn't.
        if len(fields) != 1:
            raise CommandError("10", "RTD_NEEDS_1_FIELD")
        try:
            index = int(fields[0])
        except (TypeError, ValueError):
            raise CommandError("21", f"ROUTE_INDEX_OUT_OF_RANGE:{fields[0]}")
        state.delete_route_point(index)
        return build_sentence("ACK", "RTD")

    if sentence_type == "MED":
        # Query, no fields (2026-10-05): returns every geotagged photo/
        # video still physically on disk in the camera process's capped
        # stores (see RobotState.media_positions()) for robot-webserver's
        # /control map -- its violet markers. Fields out: count, then
        # count * 7 fields per point (filename, kind, lat, lat_dir, lon,
        # lon_dir, ts) -- filename/kind/ts ride alongside the usual
        # lat/lat_dir/lon/lon_dir pair so robot-webserver can both place
        # the marker and later ask for that exact file's thumbnail
        # (app.py's /media/thumb/<kind>/<filename>). kind is "SNAP" or
        # "VID" (this protocol's existing all-caps vocabulary, see CAM's
        # own field values), not Python's "photo"/"video" spelling.
        points = state.media_positions()
        fields = [len(points)]
        for filename, kind, lat, lon, ts in points:
            lat_str, lat_dir = decimal_to_nmea(lat, is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(lon, is_longitude=True)
            wire_kind = "SNAP" if kind == "photo" else "VID"
            fields.extend([filename, wire_kind, lat_str, lat_dir, lon_str, lon_dir, ts])
        return build_sentence("MED", *fields)

    if sentence_type == "MDD":
        # Command (2026-10-05): deletes one photo/video, identified by its
        # exact MED filename and kind (SNAP/VID, this protocol's existing
        # vocabulary) -- backs the /control map's right-click "delete this
        # point" on a violet marker (the website only sends this after the
        # user confirms a popup, since it removes the actual file, not
        # just a map marker). See RobotState.delete_media() for what this
        # actually touches (the camera process's own capped store AND the
        # geotag DB row).
        if len(fields) != 2:
            raise CommandError("10", "MDD_NEEDS_2_FIELDS")
        filename, wire_kind = fields
        if wire_kind == "SNAP":
            kind = "photo"
        elif wire_kind == "VID":
            kind = "video"
        else:
            raise CommandError("19", f"BAD_MEDIA_KIND:{wire_kind}")
        state.delete_media(filename, kind)
        return build_sentence("ACK", "MDD")

    if sentence_type == "STA":
        s = state.status()
        target = s["nav_target"] or (0.0, "N", 0.0, "E")

        # Current position: real once link.gps_reader.GPSReader has a fix
        # (current_lat/current_lon are no longer None), the honest 0.0
        # placeholder otherwise (no receiver attached, or no fix yet).
        # Same ddmm.mmmm + direction format as NAV/target, built with
        # decimal_to_nmea so lat/lon and their direction letters stay
        # adjacent -- this used to be two separate, far-apart fields
        # (a hardcoded "N"/"E" assumption that was actually wrong for
        # this robot, which operates west of the meridian -- see
        # gps/gps_transfer.py's lon_cible) and has been consolidated here.
        if s["current_lat"] is not None and s["current_lon"] is not None:
            lat_str, lat_dir = decimal_to_nmea(s["current_lat"], is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(s["current_lon"], is_longitude=True)
        else:
            lat_str, lat_dir, lon_str, lon_dir = "0.0", "N", "0.0", "E"

        # DGPS fix-quality field (2026-09-19, extend-only -- see
        # RobotState.status()'s own comments anticipating this): "DGPS"
        # once a GGA sentence has reported a differentially-corrected fix
        # (link/gps_reader.py's DGPS_QUALITY), "GPS" once one has reported
        # a fix that isn't DGPS-corrected, "UNKNOWN" before any GGA
        # quality has been seen at all (no receiver attached, RMC-only
        # sentences so far, or no fix yet) -- three states, not a bool,
        # matching RobotState.is_dgps's own None/True/False tri-state.
        if s["is_dgps"] is None:
            dgps_field = "UNKNOWN"
        elif s["is_dgps"]:
            dgps_field = "DGPS"
        else:
            dgps_field = "GPS"

        # Fields, in order: current position (lat, lat_dir, lon, lon_dir --
        # matching NAV's own field order), cap (course over ground) and
        # speed (km/h, both from GPRMC once a fix is available, 0.0
        # otherwise), left_pwm, right_pwm, battery (still a placeholder --
        # no battery sensor in this project), mode, then target position
        # (target_lat, target_lat_dir, target_lon, target_lon_dir -- last
        # waypoint received via NAV, if any), then the DGPS field above
        # (new trailing field, extend-only).
        return build_sentence(
            "STA",
            lat_str, lat_dir, lon_str, lon_dir,
            round(s["cap"], 1), round(s["speed_kmh"], 2),
            s["left_pwm"], s["right_pwm"], 0, s["mode"],
            target[0], target[1], target[2], target[3],
            dgps_field,
        )

    if sentence_type == "PWR":
        # Query, no fields: returns the latest EPever Tracer reading (see
        # link.tracer_reader.TracerReader, which polls it in the
        # background) for robot-webserver's dedicated /power page
        # (2026-10-03). Same "0.0 placeholder until real hardware data
        # arrives" shape as STA's lat/lon -- see RobotState.power_status()
        # and its __init__ comment for why `available` is a trailing
        # 1/0 flag rather than the values themselves going None on the
        # wire (this protocol is text-only, there's no clean "null"
        # field -- the flag is what the power page actually checks before
        # trusting the numbers next to it).
        p = state.power_status()
        available = p["available"] is True
        # float(...) before round(): battery_soc in particular is a plain
        # int (see link.tracer_reader.read_power_snapshot) -- round(87, 1)
        # returns the int 87, not 87.0, which would make this one field
        # format as "87" instead of "87.0" on the wire. Forcing float
        # first keeps every numeric field consistently formatted
        # regardless of what type the underlying reading happens to be.
        #
        # cpu_temp (2026-10-05, 13th data field, extend-only -- an older
        # robot-webserver that doesn't know about it yet just ignores the
        # extra field) is Pi #1's own CPU temperature, not a Tracer
        # reading -- it does NOT depend on `available` above, and is
        # still reported even when the Tracer cable is unplugged. See
        # link.cpu_temp for why it falls back to the same 0.0 placeholder
        # as every other field here rather than needing its own flag.
        # onboard_time (15th field, 2026-10-10, extend-only -- an older
        # robot-webserver that doesn't know about it yet just ignores the
        # extra field, same convention as STA's dgps field and this same
        # sentence's own cpu_temp field above): Pi #1's own system clock,
        # as a whole-second Unix timestamp. Explicit user request -- the
        # /power page shows it as a small "onboard time" readout above the
        # Battery panel, specifically so a drifted/wrong Pi #1 clock (no
        # RTC battery, NTP not reachable, ...) is visible at a glance next
        # to the operator's own watch. Like cpu_temp, this is Pi #1 itself
        # and has nothing to do with the Tracer, so it is NOT gated by
        # `available` either.
        #
        # GPS fallback (2026-10-10, explicit user request): whenever the
        # system clock itself still looks unset (system_clock_is_plausible()
        # -- same check link.power_history's logger uses before writing
        # anything to the history DB), fall back to the last UTC date/time
        # the GPS receiver's own GPRMC sentence reported (RobotState.
        # last_gps_utc_ts, see link/gps_reader.py) -- a GPS fix's UTC time
        # is correct the instant a fix is acquired, with no dependency on
        # WiFi/NTP at all. onboard_time_source (16th field, a trailing
        # enum, same convention as STA's own dgps field) tells the website
        # which one it's actually looking at: "SYS" (the system clock,
        # trusted) or "GPS" (system clock not trusted, GPS time used
        # instead). If NEITHER is available yet (system clock unset AND no
        # GPS fix with a valid date has ever arrived), this falls back to
        # the system clock anyway, still labelled "SYS" -- reporting
        # *something* rather than a placeholder the wire format has no
        # clean way to express, same reasoning PWR already uses elsewhere
        # (see the 0.0-placeholder note above); the website's own display
        # of this value is what makes a still-wrong reading visible to the
        # operator.
        #
        # uptime_s (17th field, 2026-10-10, extend-only): Pi #1's own
        # uptime in whole seconds since boot (link.uptime), shown as a
        # small "lifetime" readout right next to onboard_time on /power.
        # Deliberately independent of the clock-plausibility question
        # above -- see link.uptime's own docstring for why a monotonic
        # boot-time counter has nothing to lose from an unset wall clock.
        #
        # shutdown_eta_ts (18th field, 2026-10-12, extend-only -- an
        # older robot-webserver that doesn't know about it yet just
        # ignores the extra field, same convention as every field added
        # above): set in the background by
        # link.shutdown_policy.ShutdownPolicy (see RobotState.
        # set_shutdown_eta()/power_status()) -- the single authoritative
        # "Pi1 will shut itself down at this time" answer, 0 meaning "no
        # shutdown currently scheduled" (this protocol is text-only, same
        # reasoning as `available` above for why a trailing sentinel is
        # used rather than a real null). Drives robot-webserver's
        # site-wide shutdown alert banner -- explicit user request.
        if system_clock_is_plausible():
            onboard_time = int(time.time())
            onboard_time_source = "SYS"
        elif p.get("gps_utc_ts") is not None:
            onboard_time = int(p["gps_utc_ts"])
            onboard_time_source = "GPS"
        else:
            onboard_time = int(time.time())
            onboard_time_source = "SYS"
        uptime_s = p.get("uptime_s")
        shutdown_eta_ts = p.get("shutdown_eta_ts")
        return build_sentence(
            "PWR",
            round(float(p["pv_voltage"] or 0.0), 2), round(float(p["pv_current"] or 0.0), 2),
            round(float(p["pv_power"] or 0.0), 2),
            round(float(p["battery_voltage"] or 0.0), 2),
            round(float(p["battery_charging_current"] or 0.0), 2),
            round(float(p["battery_charging_power"] or 0.0), 2),
            round(float(p["load_voltage"] or 0.0), 2), round(float(p["load_current"] or 0.0), 2),
            round(float(p["load_power"] or 0.0), 2),
            round(float(p["battery_soc"] or 0.0), 1),
            round(float(p["battery_temp"] or 0.0), 1), round(float(p["controller_temp"] or 0.0), 1),
            round(float(p["cpu_temp"] or 0.0), 1),
            1 if available else 0,
            onboard_time,
            onboard_time_source,
            uptime_s if uptime_s is not None else 0,
            int(shutdown_eta_ts) if shutdown_eta_ts is not None else 0,
        )

    if sentence_type == "HIS":
        # Query (2026-10-05): one page of the day/month history logged by
        # link.power_history.PowerHistoryLogger, for robot-webserver's
        # /power history charts. Paginated (see link.power_history.
        # HIS_CHUNK_ROWS) rather than returning a whole period in one
        # sentence -- a month at 5-minute samples is ~8640 rows, which
        # would make for an unreasonably long single line. Caller
        # (robot-webserver's power_history_client.py) loops, bumping
        # `offset` by the returned row count, until it has everything.
        #
        # Fields in: period ("DAY"/"MONTH"), offset (0-based row index
        # into that rolling window, oldest-first).
        # Fields out: period (echoed), total_count (rows in the whole
        # window right now), offset (echoed), returned_count, then
        # `returned_count` rows flattened back-to-back -- each row is
        # link.power_history.FIELD_ORDER's 16 fields (ts, lat, lon, then
        # the same PWR indicators PWR itself reports).
        if len(fields) != 2:
            raise CommandError("15", "HIS_NEEDS_2_FIELDS")
        period = fields[0].strip().upper()
        if period not in HIS_PERIODS:
            raise CommandError("16", f"HIS_BAD_PERIOD:{period}")
        try:
            offset = int(fields[1])
        except ValueError:
            raise CommandError("16", f"HIS_BAD_OFFSET:{fields[1]}")
        if offset < 0:
            raise CommandError("16", f"HIS_BAD_OFFSET:{offset}")

        total_count, rows = fetch_period_chunk(resolve_db_path(), period, offset)
        flat_rows = [value for row in rows for value in row]
        return build_sentence("HIS", period, total_count, offset, len(rows), *flat_rows)

    if sentence_type == "SMP":
        # Query (2026-10-07, explicit user request): one page of the
        # solar-exposure map grid computed by link.power_history.
        # recompute_solar_map() (see link.solar_map for the full
        # buffering/gating/gridding design), for robot-webserver's
        # /control map overlay. Paginated the same way as HIS above (see
        # link.power_history.SOLAR_MAP_CHUNK_ROWS) -- caller
        # (robot-webserver's power_history_client.py) loops, bumping
        # `offset`, until it has every cell.
        #
        # Fields in: offset (0-based row index -- no period field, the
        # grid itself has no time window, see fetch_solar_map_chunk()'s
        # own docstring).
        # Fields out: total_count (cells known right now), offset
        # (echoed), returned_count, then `returned_count` rows flattened
        # back-to-back -- each row is (lat, lon, avg_pv_power,
        # sample_count, last_ts).
        if len(fields) != 1:
            raise CommandError("23", "SMP_NEEDS_1_FIELD")
        try:
            offset = int(fields[0])
        except ValueError:
            raise CommandError("24", f"SMP_BAD_OFFSET:{fields[0]}")
        if offset < 0:
            raise CommandError("24", f"SMP_BAD_OFFSET:{offset}")

        total_count, rows = fetch_solar_map_chunk(resolve_db_path(), offset)
        flat_rows = [value for row in rows for value in row]
        return build_sentence("SMP", total_count, offset, len(rows), *flat_rows)

    if sentence_type == "STS":
        # Query (2026-10-18): one-shot summary for robot-webserver's
        # /stats page -- see link.power_history.fetch_stats_summary()'s
        # own docstring for exactly what each figure means and which
        # table it comes from. No pagination (unlike HIS/SMP/OBS): this
        # is a handful of numbers, not a row set.
        #
        # Fields in: none (same "query, no fields" shape as PWR/STA above).
        # Fields out: total_distance_m, avg_speed_moving_mps,
        # best_daily_distance_m, solar_harvested_kwh,
        # energy_consumed_kwh, best_solar_day_kwh, worst_solar_day_kwh,
        # obstacle_front_count, obstacle_rear_count.
        summary = fetch_stats_summary(resolve_db_path())
        return build_sentence(
            "STS",
            round(summary["total_distance_m"], 1),
            round(summary["avg_speed_moving_mps"], 3),
            round(summary["best_daily_distance_m"], 1),
            round(summary["solar_harvested_kwh"], 3),
            round(summary["energy_consumed_kwh"], 3),
            round(summary["best_solar_day_kwh"], 3),
            round(summary["worst_solar_day_kwh"], 3),
            summary["obstacle_front_count"],
            summary["obstacle_rear_count"],
        )

    if sentence_type == "OBS":
        # Query (2026-10-18): one page of obstacle_log, for /control's
        # new "Obstacle map (ultrasonic)" layer -- see
        # link.power_history.fetch_obstacle_chunk()'s own docstring.
        # Paginated the same way as SMP above. Will legitimately return
        # total_count=0 until the ultrasonic sensors exist and start
        # calling log_obstacle() -- see that function's own docstring.
        #
        # Fields in: offset (0-based row index).
        # Fields out: total_count, offset (echoed), returned_count, then
        # `returned_count` rows flattened back-to-back -- each row is
        # (ts, side, distance_m, lat, lon, heading_deg).
        if len(fields) != 1:
            raise CommandError("28", "OBS_NEEDS_1_FIELD")
        try:
            offset = int(fields[0])
        except ValueError:
            raise CommandError("29", f"OBS_BAD_OFFSET:{fields[0]}")
        if offset < 0:
            raise CommandError("29", f"OBS_BAD_OFFSET:{offset}")

        total_count, rows = fetch_obstacle_chunk(resolve_db_path(), offset)
        flat_rows = [value for row in rows for value in row]
        return build_sentence("OBS", total_count, offset, len(rows), *flat_rows)

    raise CommandError("11", f"UNKNOWN_SENTENCE_TYPE:{sentence_type}")


class ControlHandler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = self.client_address[0]
        log.info("connection from %s", peer)
        for raw_line in self.rfile:
            line = raw_line.decode("ascii", errors="replace").strip()
            if not line:
                continue

            try:
                sentence_type, fields = parse_sentence(line)
            except SentenceError as exc:
                log.warning("malformed sentence from %s: %s (%s)", peer, line, exc)
                response = build_sentence("ERR", "00", str(exc).replace(",", ";"))
                self._reply(response)
                continue

            try:
                response = _handle_sentence(self.server.state, sentence_type, fields, self.server)
                log.info("%s -> %s -> %s", peer, line, response)
            except CommandError as exc:
                response = build_sentence("ERR", exc.code, exc.message.replace(",", ";"))
                log.info("%s -> %s -> %s", peer, line, response)

            self._reply(response)

    def _reply(self, sentence: str):
        self.wfile.write((sentence + "\r\n").encode("ascii"))


class ControlServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, host, port, state: RobotState = None, start_gps: bool = True,
                 start_motor: bool = True, start_gamepad: bool = True, start_tracer: bool = True,
                 start_history: bool = True, start_led_control: bool = True,
                 start_camera_power_policy: bool = True, start_shutdown_policy: bool = True,
                 start_voltage_safety: bool = True):
        super().__init__((host, port), ControlHandler)

        # MotorDriver is always constructed (cheap -- no hardware is
        # touched until .start()) so it can be wired into RobotState right
        # away; whether it actually opens the GPIO chip is controlled by
        # start_motor/MOTOR_ENABLED below. If an existing `state` is
        # passed in (not done anywhere in this repo today, but the
        # parameter has always allowed it) it's used as-is and this
        # MotorDriver is simply never linked to it -- same "caller owns
        # what they passed in" behavior as before this change.
        self.motor_driver = MotorDriver()
        self.state = state or RobotState(motor_driver=self.motor_driver)

        # Set BEFORE the GPS reader is constructed/started (even though
        # the gamepad itself is only actually created further down) --
        # GPSReader runs its own background thread and can call
        # self._on_gps_quality_change() (wired in as on_gps_quality=
        # below) the moment a fix arrives, which could in principle race
        # ahead of the gamepad block below on a very fast first fix.
        # self.gamepad_reader existing as None from the start (rather
        # than not existing as an attribute at all until later) means
        # that callback's own `if self.gamepad_reader is not None` guard
        # is always safe to evaluate, whichever order things actually
        # finish in.
        self.gamepad_reader = None

        self.gps_reader = None
        if start_gps:
            # GPS_ENABLED=false disables this entirely -- useful if
            # gps/gps_transfer.py or gps/gps_parse.py is already holding
            # the serial port (only one process can own it at a time).
            if os.environ.get("GPS_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.gps_reader = GPSReader(
                    self.state,
                    device=os.environ.get("GPS_DEVICE", DEFAULT_DEVICE),
                    baudrate=int(os.environ.get("GPS_BAUDRATE", DEFAULT_BAUDRATE)),
                    on_gps_quality=self._on_gps_quality_change,
                )
                self.gps_reader.start()

        if start_motor:
            # MOTOR_ENABLED=false skips opening the GPIO chip entirely --
            # useful on a dev machine, or if some other process
            # (motor_control/remote_control.py, run standalone) already
            # owns the motor lines. drive()/stop()/set_mode() stay safe
            # to call either way (see RobotState's motor_driver docs);
            # they just have no physical effect when this is off.
            if os.environ.get("MOTOR_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.motor_driver.start()

        # The gamepad is a second input source feeding the SAME
        # RobotState as the TCP commands above (see link/gamepad_handler.
        # py's module docstring) -- a physical operator's joystick/button
        # presses and the website's STP/DRV/MOD/NAV/RTE commands are
        # peers, both just calling RobotState methods. (self.gamepad_reader
        # was already initialized to None above, before the GPS reader.)
        if start_gamepad:
            # GAMEPAD_ENABLED=false disables this entirely -- useful on a
            # dev machine with no controller plugged in (though this
            # degrades gracefully even when left on, see GamepadReader).
            if os.environ.get("GAMEPAD_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.gamepad_reader = GamepadReader(
                    on_drive=robot_state_drive_handler(self.state),
                    on_button=robot_state_button_handler(
                        self.state,
                        on_shutdown=self._shutdown_pi,
                        arm_auto_btn=GAMEPAD_ARM_AUTO_BTN,
                        record_btn=GAMEPAD_RECORD_BTN,
                        save_waypoint_btn=GAMEPAD_SAVE_WAYPOINT_BTN,
                        snapshot_btn=GAMEPAD_SNAPSHOT_BTN,
                        shutdown_btn=GAMEPAD_SHUTDOWN_BTN,
                        stop_btn=GAMEPAD_STOP_BTN,
                    ),
                )
                self.gamepad_reader.start()

        self.tracer_reader = None
        if start_tracer:
            # TRACER_ENABLED=false disables this entirely -- same escape
            # hatch as GPS_ENABLED/CAMERA_ENABLED/MOTOR_ENABLED/
            # GAMEPAD_ENABLED above, useful on a dev machine or if the
            # RS485 cable isn't plugged in at all (this already degrades
            # gracefully even when left on, see TracerReader, but the env
            # var avoids even trying).
            if os.environ.get("TRACER_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.tracer_reader = TracerReader(
                    self.state,
                    device=os.environ.get("TRACER_DEVICE", TRACER_DEFAULT_DEVICE),
                    baudrate=int(os.environ.get("TRACER_BAUDRATE", TRACER_DEFAULT_BAUDRATE)),
                    slave_id=int(os.environ.get("TRACER_SLAVE_ID", TRACER_DEFAULT_SLAVE_ID)),
                    poll_interval=float(
                        os.environ.get("TRACER_POLL_INTERVAL_S", TRACER_DEFAULT_POLL_INTERVAL_S)
                    ),
                )
                self.tracer_reader.start()

        self.power_history_logger = None
        if start_history:
            # POWER_LOG_ENABLED=false disables this entirely -- same
            # escape hatch as TRACER_ENABLED etc. above. Independent of
            # TRACER_ENABLED on purpose: the logger just checks
            # state.power_status()["available"] on every tick and skips
            # the write when it's False, same as it would with the
            # Tracer cable physically unplugged -- no need to couple the
            # two switches.
            if os.environ.get("POWER_LOG_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.power_history_logger = PowerHistoryLogger(
                    self.state,
                    db_path=resolve_db_path(),
                    log_interval=float(
                        os.environ.get("POWER_LOG_INTERVAL_S", HISTORY_DEFAULT_LOG_INTERVAL_S)
                    ),
                    retention_days=float(
                        os.environ.get("POWER_LOG_RETENTION_DAYS", HISTORY_DEFAULT_RETENTION_DAYS)
                    ),
                )
                self.power_history_logger.start()

        self.led_controller = None
        if start_led_control:
            # LED_CONTROL_ENABLED=false disables this entirely -- same
            # escape hatch as TRACER_ENABLED/POWER_LOG_ENABLED above,
            # useful on a dev machine with no /sys/class/leds/ at all
            # (this already degrades gracefully even when left on -- see
            # link.led_control._write_led_file -- but the env var avoids
            # even trying). Independent of TRACER_ENABLED on purpose,
            # same reasoning as power_history_logger just above: it only
            # acts on power_status()["available"]/["pv_voltage"], which
            # already read as "unavailable"/None with the Tracer off,
            # and LedController already treats that as "wait, don't
            # guess" (see its own docstring) -- no need to couple the
            # two switches.
            if os.environ.get("LED_CONTROL_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.led_controller = LedController(
                    self.state,
                    check_interval=float(
                        os.environ.get("LED_CHECK_INTERVAL_S", LED_CHECK_INTERVAL_S)
                    ),
                    threshold_v=float(
                        os.environ.get("LED_LOW_VOLTAGE_THRESHOLD_V", LOW_VOLTAGE_THRESHOLD_V)
                    ),
                    duration_s=float(
                        os.environ.get("LED_LOW_VOLTAGE_DURATION_S", LOW_VOLTAGE_DURATION_S)
                    ),
                )
                self.led_controller.start()

        self.camera_power_policy = None
        if start_camera_power_policy:
            # CAMERA_POWER_POLICY_ENABLED=false disables this entirely --
            # same escape hatch as LED_CONTROL_ENABLED/TRACER_ENABLED/
            # POWER_LOG_ENABLED above. Independent of TRACER_ENABLED and
            # GPS_ENABLED on purpose, same reasoning as those: decide_
            # camera_mode() (link.camera_power_policy) already treats
            # missing power/GPS data as "uncertainty -> eco" (the user's
            # own stated rule) rather than crashing or guessing, so there
            # is no need to couple this switch to the others -- running
            # with the Tracer or GPS off just means this policy settles
            # on ECO and stays there, exactly as it should.
            if os.environ.get("CAMERA_POWER_POLICY_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.camera_power_policy = CameraPowerPolicy(
                    self.state,
                    projection_interval=float(
                        os.environ.get("CAMERA_POWER_PROJECTION_INTERVAL_S", CAMERA_POWER_PROJECTION_INTERVAL_S)
                    ),
                    sample_interval=float(
                        os.environ.get("CAMERA_POWER_SAMPLE_INTERVAL_S", CAMERA_POWER_SAMPLE_INTERVAL_S)
                    ),
                    battery_capacity_ah=float(
                        os.environ.get("CAMERA_POWER_BATTERY_CAPACITY_AH", CAMERA_POWER_BATTERY_CAPACITY_AH)
                    ),
                    safety_margin_hours=float(
                        os.environ.get("CAMERA_POWER_SAFETY_MARGIN_HOURS", CAMERA_POWER_SAFETY_MARGIN_HOURS)
                    ),
                    low_pv_threshold_w=float(
                        os.environ.get("CAMERA_POWER_LOW_PV_THRESHOLD_W", CAMERA_POWER_LOW_PV_THRESHOLD_W)
                    ),
                    low_pv_sustained_records=int(
                        os.environ.get("CAMERA_POWER_LOW_PV_RECORDS", CAMERA_POWER_LOW_PV_RECORDS)
                    ),
                )
                self.camera_power_policy.start()

        self.shutdown_policy = None
        if start_shutdown_policy:
            # SHUTDOWN_POLICY_ENABLED=false disables this entirely --
            # same escape hatch as CAMERA_POWER_POLICY_ENABLED above.
            # Independent of TRACER_ENABLED/GPS_ENABLED on purpose, same
            # reasoning as that policy: compute_scheduled_shutdown_ts()
            # (link.shutdown_policy) already treats missing power/GPS
            # data as "uncertainty -> do NOT schedule a shutdown" rather
            # than crashing or guessing, so there is no need to couple
            # this switch to the others -- running with the Tracer or
            # GPS off just means this policy never schedules anything,
            # exactly as it should (wrongly staying silent is far safer
            # here than wrongly triggering a real poweroff).
            if os.environ.get("SHUTDOWN_POLICY_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.shutdown_policy = ShutdownPolicy(
                    self.state,
                    on_shutdown=self._shutdown_pi,
                    projection_interval=float(
                        os.environ.get("SHUTDOWN_PROJECTION_INTERVAL_S", SHUTDOWN_PROJECTION_INTERVAL_S)
                    ),
                    sample_interval=float(
                        os.environ.get("SHUTDOWN_SAMPLE_INTERVAL_S", SHUTDOWN_SAMPLE_INTERVAL_S)
                    ),
                    battery_capacity_ah=float(
                        os.environ.get("SHUTDOWN_BATTERY_CAPACITY_AH", SHUTDOWN_BATTERY_CAPACITY_AH)
                    ),
                    lead_hours=float(
                        os.environ.get("SHUTDOWN_LEAD_HOURS", SHUTDOWN_LEAD_HOURS)
                    ),
                    safety_margin_hours=float(
                        os.environ.get("SHUTDOWN_SAFETY_MARGIN_HOURS", SHUTDOWN_SAFETY_MARGIN_HOURS)
                    ),
                )
                self.shutdown_policy.start()

        self.voltage_safety = None
        if start_voltage_safety:
            # VOLTAGE_SAFETY_ENABLED=false disables this entirely --
            # same escape hatch as SHUTDOWN_POLICY_ENABLED above.
            # Independent of TRACER_ENABLED/GAMEPAD_ENABLED on purpose:
            # VoltageSafety already treats a missing battery_voltage
            # reading as "uncertainty -> do not act" (is_voltage_
            # critical()) and a missing gamepad as "no rumble to drive,
            # the IDLE-forcing/AUTO-lockout logic still works" (see
            # link.voltage_safety.VoltageSafety's own docstring) -- no
            # need to couple this switch to the others.
            if os.environ.get("VOLTAGE_SAFETY_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.voltage_safety = VoltageSafety(
                    self.state,
                    gamepad=self.gamepad_reader,
                    check_interval=float(
                        os.environ.get("VOLTAGE_SAFETY_CHECK_INTERVAL_S", VOLTAGE_SAFETY_CHECK_INTERVAL_S)
                    ),
                    critical_threshold_v=float(
                        os.environ.get("VOLTAGE_SAFETY_CRITICAL_V", CRITICAL_VOLTAGE_THRESHOLD_V)
                    ),
                    recovery_threshold_v=float(
                        os.environ.get("VOLTAGE_SAFETY_RECOVERY_V", RECOVERY_VOLTAGE_THRESHOLD_V)
                    ),
                )
                self.voltage_safety.start()

    def _on_gps_quality_change(self, is_dgps):
        """Wired in as link.gps_reader.GPSReader's on_gps_quality callback
        above -- called on the GPS background thread the instant the live
        fix's quality genuinely flips between DGPS-corrected and not (see
        GPSReader's own docstring for the exact edge-triggering rule).
        Physically confirms it to whoever's holding the controller: a
        strong DGPS_PULSE_DURATION_S pulse on gaining DGPS, a weak one of
        the same duration on losing it -- see
        link.gamepad_handler.GamepadReader.pulse().

        Guarded by `is not None` rather than assuming the gamepad is
        always present: GAMEPAD_ENABLED=false, no controller plugged in
        (still degrades gracefully -- pulse() itself is also a safe
        no-op with nothing connected, this guard just avoids calling a
        method on a None reader entirely when the whole feature is off),
        or -- start_gps=True/start_gamepad=False in a test/dev run -- the
        GPS reader firing before the gamepad block above ever runs (see
        the comment where self.gamepad_reader is first set to None)."""
        if self.gamepad_reader is not None:
            self.gamepad_reader.pulse(strong=is_dgps, duration_s=DGPS_PULSE_DURATION_S)

    def _shutdown_pi(self):
        """Called by the gamepad's BTN_START handler (see
        link/gamepad_handler.py's robot_state_button_handler(), which
        already stops the motors before calling this): powers off the
        whole Raspberry Pi, not just this process. Runs on the
        GamepadReader background thread -- a different thread from
        whichever one is running serve_forever() below, which is exactly
        the situation self.shutdown() is documented to require (calling
        it from the SAME thread that runs serve_forever() would deadlock).

        Requires passwordless sudo for SHUTDOWN_CMD (default `sudo
        poweroff`) for whichever user runs run_robot.sh -- see this
        project's README for the one-time sudoers setup. If that's
        missing, or the command otherwise fails, this logs an error and
        still stops this robot's own scripts (the `finally` below) rather
        than leaving the control server running silently as if nothing
        was wrong.

        Order matters: the OS poweroff is requested FIRST, then this
        process asks its own serve_forever() loop to stop. Once
        self.shutdown() returns, main() falls through and the process
        exits on its own (MotorDriver's atexit hook releases the GPIO
        chip) -- and since self.gamepad_reader.start() runs this as a
        daemon thread, doing the (slower, more important) OS-level call
        first means it isn't at risk of being cut short if the interpreter
        starts tearing down daemon threads once main() returns."""
        log.warning(
            "BTN_START pressed on the gamepad -- powering off the Raspberry "
            "Pi (%s) and stopping this robot's own control scripts.",
            " ".join(SHUTDOWN_CMD),
        )
        try:
            subprocess.run(SHUTDOWN_CMD, check=True, timeout=10)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            log.error(
                "could not power off the Raspberry Pi (%s) -- check the "
                "passwordless-sudo setup in README.md ('Pilotage moteur et "
                "manette'). Stopping this robot's own scripts anyway.",
                exc,
            )
        finally:
            self.shutdown()


def main():
    host = os.environ.get("CONTROL_HOST", DEFAULT_HOST)
    port = int(os.environ.get("CONTROL_PORT", DEFAULT_PORT))
    server = ControlServer(host, port)
    log.info("control server listening on %s:%s", host, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("shutting down")
        server.shutdown()


if __name__ == "__main__":
    main()
