"""Historical power/GPS logger (2026-10-05): every `log_interval` seconds,
snapshots the current PWR reading (and GPS fix, if any) into a small
SQLite database on Pi #1, so robot-webserver's /power page can draw
day/month trend charts (see pages/power.html and the new HIS sentence in
link/server.py) without Pi #2 needing its own database -- Pi #1 keeps
logging even if the WiFi link to Pi #2 is briefly down, since the
logger reads straight off this process's own RobotState rather than
going over the network.

GATING (explicit project requirement, do not relax): a sample is only
written when `state.power_status()["available"]` is True, i.e. the
RS485/Tracer link is actually returning data right now. Logging on a
plain timer regardless of that flag would fill the history with rows of
pure placeholder zeros every time the cable is unplugged -- see
link.tracer_reader/link.robot_state for why `available` exists at all.

Same "optional, best-effort, degrade gracefully" shape as TracerReader/
GPSReader: a disk error logs a warning and is retried next tick rather
than crashing the control server.

RETENTION: ~5-10 months of continuous 5-minute samples is a few MB (288
rows/day * ~150 bytes/row ~= 43 KB/day ~= 1.3 MB/month), so there is no
real storage pressure to economize on -- DEFAULT_RETENTION_DAYS below is
set generously (~13 months) specifically so a full year of history is
always available for season-over-season comparison, with old rows
pruned automatically past that so the file never grows unbounded.

NOT YET TESTED against the real Tracer/GPS hardware (same caveat as
every other module in this phase) -- sqlite3 is stdlib so there's no
installability concern, but the actual gating-on-`available` behavior
and the GPS-fix-or-NULL path should both be watched on the first real
run in the field.
"""
import logging
import os
import sqlite3
import threading
import time

log = logging.getLogger("link.power_history")

# -- Schema -----------------------------------------------------------------
# One row per logged sample. `ts` (unix seconds) is both the primary key and
# the natural ordering/range-query column. `lat`/`lon` are nullable -- NULL
# means "no GPS fix at the moment this sample was logged", not "0,0" (unlike
# STA/PWR's wire format, SQLite has a real NULL, so there is no need for the
# 0.0-placeholder convention those sentences use).
_SCHEMA = """
CREATE TABLE IF NOT EXISTS power_log (
    ts INTEGER PRIMARY KEY,
    lat REAL,
    lon REAL,
    pv_voltage REAL,
    pv_current REAL,
    pv_power REAL,
    battery_voltage REAL,
    battery_charging_current REAL,
    battery_charging_power REAL,
    load_voltage REAL,
    load_current REAL,
    load_power REAL,
    battery_soc REAL,
    battery_temp REAL,
    controller_temp REAL,
    cpu_temp REAL
);
"""

# Snapshots table (2026-10-05, extended 2026-10-05 to also cover video
# recordings -- see `kind` below): one row per camera snapshot (CAM,SNAP)
# or video recording (CAM,REC_START/REC_STOP), geotagged with the GPS fix
# live at the moment it was taken. Separate from power_log above and NOT
# gated on the Tracer's `available` flag -- a photo/video has nothing to
# do with the solar charge controller, it's purely "this file, taken at
# this GPS point" (or no point, if there's no GPS fix yet -- `lat`/`lon`
# stay NULL exactly like power_log's, same reasoning). `filename` matches
# camera/stream_server.py's SnapshotStore/VideoRecorder naming (already
# unique/collision-free within its own store), so it's used as-is as the
# primary key rather than inventing a second id -- the two stores never
# share a filename prefix ("snap_"/"rec_"), so collisions across kinds
# aren't a concern either.
#
# `kind` ("photo"/"video") lets one query (fetch_media_positions below)
# serve robot-webserver's /control map's violet markers regardless of
# which store a file came from, since the map only cares about "where was
# this media taken", not which camera endpoint produced it.
_SNAPSHOT_SCHEMA = """
CREATE TABLE IF NOT EXISTS snapshots (
    filename TEXT PRIMARY KEY,
    kind TEXT NOT NULL DEFAULT 'photo',
    ts INTEGER NOT NULL,
    lat REAL,
    lon REAL
);
"""

# Column order also used, in this exact sequence, by:
#  - insert_sample()'s INSERT statement below,
#  - fetch_period_chunk()'s SELECT below, and
#  - link/server.py's HIS sentence (one flattened row = these 16 fields,
#    in this order) and robot-webserver's power_history_client.py, which
#    has its own copy of this same list to decode it (see link/nmea.py's
#    docstring for why duplicating shared-protocol knowledge by hand
#    across both repos is this project's existing convention).
FIELD_ORDER = (
    "ts", "lat", "lon",
    "pv_voltage", "pv_current", "pv_power",
    "battery_voltage", "battery_charging_current", "battery_charging_power",
    "load_voltage", "load_current", "load_power",
    "battery_soc", "battery_temp", "controller_temp", "cpu_temp",
)

DEFAULT_DB_PATH = os.path.join("data", "power_history.db")
DEFAULT_LOG_INTERVAL_S = 300  # 5 minutes, per the user's request
DEFAULT_RETENTION_DAYS = 395  # ~13 months, see module docstring

# Rolling windows (not calendar-aligned "today"/"this month" -- always full
# and up to date, whatever time of day/month the chart is viewed).
HIS_PERIODS = {
    "DAY": 24 * 3600,
    "MONTH": 30 * 24 * 3600,
}

# How many rows link/server.py's HIS sentence returns per request. A month
# at 5-minute samples is ~8640 rows -- one single-line response that big
# would work over TCP but makes for an awkwardly huge NMEA-style sentence,
# so HIS is paginated instead: robot-webserver's power_history_client.py
# loops, bumping the offset, until it has the whole period. 100 rows *
# 16 fields is a plainly reasonable line length (a few KB) either way.
HIS_CHUNK_ROWS = 100


def resolve_db_path() -> str:
    """The one place both the logger (below) and link/server.py's HIS
    handler decide where the database lives, so they always agree
    without any object wiring between them -- see this module's
    docstring. POWER_HISTORY_DB_PATH overrides the default, same pattern
    as TRACER_DEVICE/GPS_DEVICE etc."""
    return os.environ.get("POWER_HISTORY_DB_PATH", DEFAULT_DB_PATH)


def _connect(db_path: str) -> sqlite3.Connection:
    directory = os.path.dirname(db_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    # A fresh connection per call (rather than one long-lived shared
    # connection) sidesteps sqlite3's same-thread restriction entirely --
    # the background logger thread and whichever ThreadingTCPServer
    # worker thread is handling a HIS request never touch the same
    # connection object. WAL mode lets that HIS read proceed without
    # blocking on a concurrent insert (and vice versa).
    conn = sqlite3.connect(db_path, timeout=5.0)
    conn.execute("PRAGMA journal_mode=WAL")
    return conn


def ensure_schema(db_path: str) -> None:
    with _connect(db_path) as conn:
        conn.execute(_SCHEMA)
        conn.execute(_SNAPSHOT_SCHEMA)


def log_media(db_path: str, filename: str, kind: str, lat, lon) -> None:
    """Records one camera snapshot's or video recording's filename against
    a GPS position (possibly None/None, if there's no fix yet). `kind` is
    "photo" or "video". Called from link.robot_state.RobotState --
    _request_snapshot() for photos (position live at CAM,SNAP time), and
    _request_recording() for videos (position live at CAM,REC_START time,
    but only written once REC_STOP hands back the actual filename -- see
    that method's own comment for why) -- see the `snapshots` table's own
    comment above for why this isn't gated on Tracer availability the way
    power_log is."""
    ensure_schema(db_path)
    with _connect(db_path) as conn:
        conn.execute(
            "INSERT OR REPLACE INTO snapshots (filename, kind, ts, lat, lon) VALUES (?, ?, ?, ?, ?)",
            (filename, kind, int(time.time()), lat, lon),
        )


def fetch_media_positions(db_path: str, filenames) -> list:
    """Returns (filename, kind, lat, lon, ts) for every name in
    `filenames` that has a geotag on record -- order is whatever SQLite
    hands back, callers that care about order (none currently do) should
    sort it themselves. Used by link/server.py's MED sentence: `filenames`
    is meant to be the CURRENTLY-ON-DISK listing from camera/snapshots.py's
    SnapshotStore.list_files() + camera/recordings.py's VideoRecorder.
    list_files() combined (see link.robot_state.RobotState.media_positions),
    so a geotag whose file has since been rotated out of those capped-5
    FIFO stores never comes back here and never shows up as a dead link on
    the map -- this table can easily outlive the files themselves (no
    retention cap of its own beyond power_log's shared prune_old_rows,
    which doesn't even touch this table), so filtering by what's still
    actually on disk happens here, not in the caller."""
    if not filenames:
        return []
    ensure_schema(db_path)
    placeholders = ",".join("?" for _ in filenames)
    with _connect(db_path) as conn:
        cursor = conn.execute(
            f"SELECT filename, kind, lat, lon, ts FROM snapshots WHERE filename IN ({placeholders})",
            list(filenames),
        )
        return [
            (row[0], row[1], 0.0 if row[2] is None else row[2], 0.0 if row[3] is None else row[3], row[4])
            for row in cursor.fetchall()
        ]


def insert_sample(db_path: str, sample: dict) -> None:
    """`sample` must have exactly FIELD_ORDER's keys (lat/lon may be
    None)."""
    placeholders = ",".join("?" for _ in FIELD_ORDER)
    columns = ",".join(FIELD_ORDER)
    with _connect(db_path) as conn:
        conn.execute(
            f"INSERT OR REPLACE INTO power_log ({columns}) VALUES ({placeholders})",
            [sample[key] for key in FIELD_ORDER],
        )


def prune_old_rows(db_path: str, retention_days: float) -> None:
    cutoff = int(time.time()) - int(retention_days * 24 * 3600)
    with _connect(db_path) as conn:
        conn.execute("DELETE FROM power_log WHERE ts < ?", (cutoff,))


def fetch_period_chunk(db_path: str, period: str, offset: int):
    """Returns (total_count, rows) for the rolling `period` window
    ("DAY"/"MONTH", see HIS_PERIODS), oldest-first, starting at `offset`
    and returning at most HIS_CHUNK_ROWS rows -- see HIS_CHUNK_ROWS'
    comment for why this is paginated rather than one giant response.
    Each row is a tuple in FIELD_ORDER order, with lat/lon's SQL NULL
    turned into 0.0 (the same "no fix yet" placeholder STA/PWR already
    use on the wire -- NMEA-style sentences have no clean NULL field)."""
    window_seconds = HIS_PERIODS[period]
    cutoff = int(time.time()) - window_seconds
    ensure_schema(db_path)  # harmless no-op once the table exists; cheap, and
    # saves every caller from having to ensure_schema() before a read on a
    # brand new install where the logger hasn't ticked yet.
    with _connect(db_path) as conn:
        total_count = conn.execute(
            "SELECT COUNT(*) FROM power_log WHERE ts >= ?", (cutoff,)
        ).fetchone()[0]
        columns = ",".join(FIELD_ORDER)
        cursor = conn.execute(
            f"SELECT {columns} FROM power_log WHERE ts >= ? "
            "ORDER BY ts ASC LIMIT ? OFFSET ?",
            (cutoff, HIS_CHUNK_ROWS, offset),
        )
        rows = [
            tuple(0.0 if value is None else value for value in row)
            for row in cursor.fetchall()
        ]
    return total_count, rows


class PowerHistoryLogger:
    """Runs in a background thread, same "start()/stop() + daemon thread"
    shape as link.tracer_reader.TracerReader -- see that module's
    docstring. Unlike TracerReader, there's no hardware handle to open
    here: every tick just reads `state` (already updated by GPSReader/
    TracerReader on their own threads) and, if the Tracer is currently
    reporting data, writes one row."""

    def __init__(self, state, db_path=None, log_interval=DEFAULT_LOG_INTERVAL_S,
                 retention_days=DEFAULT_RETENTION_DAYS):
        self.state = state
        self.db_path = db_path or resolve_db_path()
        self.log_interval = log_interval
        self.retention_days = retention_days
        self._running = False
        ensure_schema(self.db_path)

    def start(self):
        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def _loop(self):
        log.info(
            "Power history logger started (%s, every %ss, keeping %s days)",
            self.db_path, self.log_interval, self.retention_days,
        )
        while self._running:
            try:
                self._log_once()
            except Exception:
                # Best-effort, same spirit as TracerReader's per-tick
                # try/except -- one failed write (disk full, locked file)
                # shouldn't take the whole control server down, and the
                # next tick gets another chance.
                log.exception("power history logging failed this tick, will retry next one")
            time.sleep(self.log_interval)

    def _log_once(self):
        power = self.state.power_status()
        if power.get("available") is not True:
            # The RS485/Tracer link isn't returning data right now --
            # per this module's docstring, skip the tick entirely rather
            # than logging a row of placeholder zeros. cpu_temp is
            # deliberately not logged on its own here either: a sample
            # without any Tracer context isn't useful for these charts
            # and the gating rule was explicit about the whole row.
            return
        status = self.state.status()
        sample = {
            "ts": int(time.time()),
            "lat": status.get("current_lat"),
            "lon": status.get("current_lon"),
            "pv_voltage": power.get("pv_voltage"),
            "pv_current": power.get("pv_current"),
            "pv_power": power.get("pv_power"),
            "battery_voltage": power.get("battery_voltage"),
            "battery_charging_current": power.get("battery_charging_current"),
            "battery_charging_power": power.get("battery_charging_power"),
            "load_voltage": power.get("load_voltage"),
            "load_current": power.get("load_current"),
            "load_power": power.get("load_power"),
            "battery_soc": power.get("battery_soc"),
            "battery_temp": power.get("battery_temp"),
            "controller_temp": power.get("controller_temp"),
            "cpu_temp": power.get("cpu_temp"),
        }
        insert_sample(self.db_path, sample)
        prune_old_rows(self.db_path, self.retention_days)
