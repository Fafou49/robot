"""Solar-exposure survey & map (2026-10-07, explicit user request): while
driving (via the gamepad OR in AUTO mode), the robot periodically samples
its GPS position and the solar panel's instantaneous PV power, buffers
those samples to a small tmp file, and -- every
link.power_history.DEFAULT_LOG_INTERVAL_S tick of that module's
PowerHistoryLogger -- flushes the buffer into a SQLite table
(power_history.py's `solar_survey_raw`) and, only while the robot is
otherwise idle (mode == "IDLE", see PowerHistoryLogger._log_once()),
recomputes a coarse grid of average PV power per cell
(`solar_map_cells`) for robot-webserver's /control map overlay (link/
server.py's new SMP sentence, robot-webserver's power_history_client.py/
app.py).

GATING (two explicit refinements the user agreed to, 2026-10-07):
  1. Only sample on an ACTUALLY VALID GPS fix. In practice this is
     already guaranteed by the call site -- link.gps_reader.GPSReader
     only calls RobotState.update_gps_fix() when link.gps_reader.
     parse_fix() returned a genuine fix (RMC status "A" / GGA quality
     != 0), never on a missing/void one -- so should_record_point()
     below never has to re-check fix validity itself, only distance.
  2. Only sample while the Tracer is actually responding
     (state.power_available is True) -- the exact same gating rule as
     link.power_history's own power_log, so a disconnected RS485 cable
     never pairs a real GPS position with a stale/placeholder pv_power
     reading. Enforced by the caller (link.robot_state.RobotState.
     update_gps_fix()), not by this module.

The "every 5 metres" rule is a straight-line (haversine) distance from
the last BUFFERED point (not the last raw GPS fix) -- link.autopilot.
haversine_distance_m is reused rather than reimplemented, same as every
other distance check in this project (see link.robot_state.
_advance_route_if_arrived's own comment on why).
"""
import json
import logging
import os

from link.autopilot import haversine_distance_m

log = logging.getLogger("link.solar_map")

# Per the user's explicit request ("tous les 5 metres").
SOLAR_SURVEY_MIN_DISTANCE_M = 5.0

DEFAULT_TMP_PATH = os.path.join("data", "solar_survey_tmp.jsonl")

# Grid cell size, in decimal degrees -- roughly 5m at mid-latitudes,
# matching SOLAR_SURVEY_MIN_DISTANCE_M so the map's own resolution is
# about as fine as the underlying sampling. Not a true constant-area
# grid (a degree of longitude shrinks with latitude) -- fine for this
# robot's small single-site operating area, same plain-degree
# approximation already used elsewhere in this project (e.g.
# robot-webserver/app.py's PAD_DEG/MAP_MIN_SPAN_DEG comments).
SOLAR_MAP_CELL_SIZE_DEG = 0.000045


def resolve_tmp_path() -> str:
    """SOLAR_SURVEY_TMP_PATH overrides the default -- same override
    convention as link.power_history.resolve_db_path()/WAYPOINTS_FILE
    elsewhere in this project."""
    return os.environ.get("SOLAR_SURVEY_TMP_PATH", DEFAULT_TMP_PATH)


def should_record_point(last_point, lat: float, lon: float) -> bool:
    """`last_point` is the (lat, lon) of the last BUFFERED sample, or
    None if nothing has been buffered yet this run (always records the
    very first point). True once the great-circle distance to
    `last_point` is >= SOLAR_SURVEY_MIN_DISTANCE_M."""
    if last_point is None:
        return True
    last_lat, last_lon = last_point
    return haversine_distance_m(last_lat, last_lon, lat, lon) >= SOLAR_SURVEY_MIN_DISTANCE_M


def append_point(tmp_path: str, ts: float, lat: float, lon: float, pv_power) -> None:
    """Appends one sample as a single JSON-lines row. Plain append-only
    file I/O, no locking needed: written only from RobotState.
    update_gps_fix() (itself only ever handling one GPS fix at a time,
    see link.gps_reader.GPSReader's own single-threaded read loop), and
    drained only by link.power_history.PowerHistoryLogger's own single
    background thread -- see drain_points() below."""
    directory = os.path.dirname(tmp_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    row = {"ts": int(ts), "lat": lat, "lon": lon, "pv_power": pv_power}
    try:
        with open(tmp_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")
    except OSError:
        # Best-effort, same "disk error -> log and move on, don't crash
        # the GPS fix handler" spirit as every other I/O in this project
        # (link.tracer_reader/link.gps_reader/link.power_history) -- one
        # missed sample isn't worth taking the control loop down over.
        log.exception("failed to append solar survey point to %s", tmp_path)


def drain_points(tmp_path: str) -> list:
    """Reads and parses every buffered line, then truncates the file
    back to empty (the same file, not a rename/delete, so the very next
    append_point() call always succeeds without needing to recreate
    anything). Returns [] without touching the file at all if it
    doesn't exist yet or is empty -- the common case ("nothing moved
    >=5m, or the Tracer's been down", checked every
    DEFAULT_LOG_INTERVAL_S tick) stays a cheap no-op. A malformed line
    (e.g. a half-written one from a crash mid-write) is skipped rather
    than aborting the whole drain, same defensive spirit as
    append_point()'s own try/except."""
    if not os.path.exists(tmp_path):
        return []
    with open(tmp_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    if not lines:
        return []

    points = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
            points.append((row["ts"], row["lat"], row["lon"], row["pv_power"]))
        except (ValueError, KeyError, TypeError):
            log.warning("skipping malformed solar survey tmp line: %r", line)

    # Truncate in place rather than delete -- see docstring above.
    with open(tmp_path, "w", encoding="utf-8"):
        pass
    return points


def cell_key(lat: float, lon: float) -> tuple:
    """Buckets (lat, lon) into a fixed-size grid cell, identified by its
    own (lat, lon) integer indices -- see SOLAR_MAP_CELL_SIZE_DEG.
    Python's `//` always floors (including for negative operands, e.g.
    this project's own west-of-Greenwich longitudes), so two points in
    the same cell always map to the same indices regardless of sign."""
    return (
        int(lat // SOLAR_MAP_CELL_SIZE_DEG),
        int(lon // SOLAR_MAP_CELL_SIZE_DEG),
    )


def cell_center(cell_lat_idx: int, cell_lon_idx: int) -> tuple:
    """The (lat, lon) center of the cell identified by cell_key()'s own
    indices -- used both when writing solar_map_cells rows
    (link.power_history.recompute_solar_map()) and, on the wire/
    robot-webserver side, for drawing each cell's square on the map."""
    return (
        (cell_lat_idx + 0.5) * SOLAR_MAP_CELL_SIZE_DEG,
        (cell_lon_idx + 0.5) * SOLAR_MAP_CELL_SIZE_DEG,
    )
