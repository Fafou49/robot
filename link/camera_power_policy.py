"""Camera reactive/eco power policy.

ORIGINAL rule (2026-10-10, explicit user request):

    "je souhaites la reactivite lorsque le panneau solaire charge ou que
    j'ai assez d'energie pour passer la nuit. En cas d'incertitude on
    passe en mode eco"

    (reactivity while the solar panel is charging, or while there is
    enough energy reserve to last the night; when in doubt, fall back to
    eco mode)

Refined, also the user's own words, when asked how often to project and
what to project with:

    "a faire a bord de la pi toutes les 5 minutes: utilises la
    consommation moyenne des dernieres 5 minutes et fais une projection
    avec les reserves de la batterie et l'heure actuelle"

REVISED rule (2026-10-12, explicit user request, replaces the original
rule above -- see decide_camera_mode()'s own docstring for the exact
order this is now applied in):

    "la camera doit etre en eco la nuit quelque soit l'etat de la
    batterie. Elle sort du mode Eco la nuit si un snap ou une video est
    lancee. Elle retourne en mode eco si la batterie ne peut pas
    atteindre la fin de la journee solaire suivant. SI le jour est
    calcule et que le panneau solaire ne fournit pas 7w ou plus depuis
    15minutes (3 enregistrements en BD), alors la camera passe en Eco."

    (the camera must be in eco at night no matter the battery's state;
    it leaves eco at night when a snapshot or a recording is triggered;
    it falls back to eco if the battery can't reach [the clarified
    target below]; if it's day and the solar panel hasn't supplied 7W or
    more for the last 15 minutes [3 DB records], the camera switches to
    eco)

Two points were confirmed with the user rather than guessed:
  - "atteindre la fin de la journee solaire suivant[e]" keeps the
    ORIGINAL projection target: the next sunrise (survive the night),
    not the next sunset -- this check is only reachable during the day
    branch below (night is now an unconditional ECO), so in practice it
    answers "will the reserve, discharging at today's net rate, last
    until tomorrow morning?"
  - The old "battery is charging -> REACTIVE" shortcut is REMOVED
    entirely -- the day branch below (reserve projection + the new
    sustained-low-PV check) is now the complete day-mode rule, not an
    addition to it.

"Elle sort du mode Eco la nuit si un snap ou une video est lancee" needs
no change here at all: that's camera/stream_server.py's FrameGrabber.
request_active()/_wait_for_frame() mechanism (already built, see that
module), which wakes the camera device on demand for a single capture
regardless of which mode this policy has pushed -- the mode this module
decides is about CONTINUOUS capture, not about blocking an on-demand
snap/recording.

This module is the *decision* half of the feature: every PROJECTION_INTERVAL_S
(5 minutes), it looks at the trailing average load/PV power (sampled more
finely, every SAMPLE_INTERVAL_S), the current battery state of charge, the
day/night state and time remaining until the next local sunrise (see
link.sunrise), and the last few logged PV readings (see
link.power_history.fetch_recent_pv_readings), and decides whether
camera/stream_server.py (a SEPARATE process on Pi #1, see
link.robot_state's own _request_snapshot()/_request_recording() for the
established cross-process pattern this mirrors) should run REACTIVE
(capture continuously, like today) or ECO (only wake up on demand -- see
camera/stream_server.py's own FrameGrabber changes for what ECO actually
does on that side).

This module never talks to the camera process's sockets/I/O directly for
the decision itself (decide_camera_mode() below is a free, pure function,
same "logic separate from I/O" split as link.power_history's
system_clock_is_plausible() or link.solar_map's should_record_point()) --
CameraPowerPolicy wraps it with the actual background-thread sampling,
the power_history DB read for the sustained-low-PV check, and the HTTP
POST that tells camera/stream_server.py which mode to run in.

Honesty note: BATTERY_CAPACITY_AH=35 is reused from pages/power.html's own
existing capacity constant (already used there for this robot's actual
battery) -- not re-derived or guessed here. SAFETY_MARGIN_HOURS=1.0 is a
deliberately simple, conservative default (no battery chemistry/efficiency
curve is modelled -- the projection is linear discharge at the current
averaged rate), not a value requested by the user; worth tuning after
watching a few real nights' telemetry. LOW_PV_THRESHOLD_W=7.0 and
LOW_PV_SUSTAINED_RECORDS=3 are the user's own explicit values; see
is_sustained_low_pv() for LOW_PV_STALE_WINDOW_S, the one value here that
is NOT from the user (a slack margin against a logging gap).
"""
import json
import logging
import os
import threading
import time
import urllib.error
import urllib.request

from link import power_history
from link.power_projection import (
    BATTERY_CAPACITY_AH,
    SAFETY_MARGIN_HOURS,
    project_remaining_hours,
)
from link.sunrise import next_sunrise_utc, next_sunset_utc

log = logging.getLogger(__name__)

REACTIVE = "reactive"
ECO = "eco"

# How often decide_camera_mode() is actually evaluated and (if changed)
# pushed to the camera process -- explicit user request: "toutes les 5
# minutes".
PROJECTION_INTERVAL_S = 5 * 60
# How often power_status() is polled to build the trailing average fed
# into that 5-minute decision -- finer than the decision itself so the
# average isn't just one single reading. No preference stated by the
# user for this one; chosen to give ~20 samples per 5-minute window
# without adding meaningful extra load.
SAMPLE_INTERVAL_S = 15

# BATTERY_CAPACITY_AH and SAFETY_MARGIN_HOURS are now defined in
# link.power_projection (imported above) -- single source of truth
# shared with link.shutdown_policy. Re-exported here unchanged so
# anything importing them from this module (link.server, tests) keeps
# working -- see this module's docstring for their provenance.

# Sustained-low-PV day rule (2026-10-12, explicit user request/values):
# "le panneau solaire ne fournit pas 7w ou plus depuis 15minutes (3
# enregistrements en BD)". power_history logs every 5 minutes (see
# link.power_history.DEFAULT_LOG_INTERVAL_S), so 3 records = 15 minutes.
LOW_PV_THRESHOLD_W = 7.0
LOW_PV_SUSTAINED_RECORDS = 3
# Slack beyond the nominal 15 minutes (3 * 5-minute log interval) before
# treating the 3 most recent DB rows as too old to mean "right now" --
# NOT a user-specified value, just a guard against a logging gap (Tracer
# briefly unavailable, process restart) making stale rows look like a
# live sustained dip. Deliberately generous to avoid false ECO triggers.
LOW_PV_STALE_WINDOW_S = 20 * 60

# Cross-process HTTP notification to camera/stream_server.py -- same
# CAMERA_HOST/CAMERA_PORT env vars link.robot_state's own
# _request_snapshot()/_request_recording() already use (this process
# connects, defaulting to 127.0.0.1; the camera process binds, defaulting
# to 0.0.0.0 -- same existing convention, see link.robot_state's own
# CAMERA_HOST_DEFAULT/CAMERA_PORT_DEFAULT).
CAMERA_HOST_DEFAULT = "127.0.0.1"
CAMERA_PORT_DEFAULT = 8000
CAMERA_POWER_MODE_PATH_DEFAULT = "/power_mode"
CAMERA_POWER_MODE_TIMEOUT = 3  # seconds


def is_sustained_low_pv(
    recent_pv_readings,
    now_ts=None,
    *,
    threshold_w=LOW_PV_THRESHOLD_W,
    required_records=LOW_PV_SUSTAINED_RECORDS,
    max_age_s=LOW_PV_STALE_WINDOW_S,
):
    """Pure helper for decide_camera_mode()'s day-mode PV check.
    `recent_pv_readings` is a list of (ts, pv_power) tuples, NEWEST
    FIRST -- the exact shape link.power_history.fetch_recent_pv_readings()
    returns. True only when there are at least `required_records` rows,
    ALL of them below `threshold_w`, AND the oldest of those rows is no
    older than `max_age_s` -- that last guard is what keeps a logging gap
    (Tracer briefly unavailable, a process restart) from making 3 stale
    rows from long ago look like "low PV right now"; too few/too-old
    rows simply means this extra rule doesn't fire (returns False), it
    does NOT by itself force ECO -- the reserve-projection check in
    decide_camera_mode() is what covers genuine uncertainty."""
    if len(recent_pv_readings) < required_records:
        return False
    recent = recent_pv_readings[:required_records]
    if any(pv_power >= threshold_w for _, pv_power in recent):
        return False
    now = now_ts if now_ts is not None else time.time()
    oldest_ts = recent[-1][0]
    return (now - oldest_ts) <= max_age_s


def decide_camera_mode(
    *,
    available,
    battery_soc,
    battery_voltage,
    avg_load_power,
    avg_pv_power,
    lat,
    lon,
    sustained_low_pv=False,
    battery_capacity_ah=BATTERY_CAPACITY_AH,
    safety_margin_hours=SAFETY_MARGIN_HOURS,
    now_ts=None,
):
    """Pure decision function -- see this module's docstring for the full
    rule's history. Returns REACTIVE or ECO, never raises, never blocks
    (no I/O here at all -- next_sunrise_utc()/next_sunset_utc() are pure
    math, see link.sunrise; `sustained_low_pv` is a plain bool the caller
    computes from the DB via is_sustained_low_pv() above -- this function
    itself never reads the database).

    Applied in this order, "uncertainty -> eco" still the blanket
    fallback for anything that can't be determined:

      1. Tracer/power data not available at all            -> ECO
      2. No GPS position fix (no lat/lon)                  -> ECO  (can't
         tell day from night, or compute a local sunrise/sunset,
         without one)
      3. Day/night can't be determined at all (polar        -> ECO
         day/night at this latitude -- see link.sunrise)
      4. It's currently NIGHT                                -> ECO
         (unconditional -- explicit user request: "la camera doit etre
         en eco la nuit quelque soit l'etat de la batterie". The
         existing on-demand wake for a snap/recording is handled
         entirely in camera/stream_server.py's FrameGrabber and is
         unaffected by this mode decision -- see this module's
         docstring.)
      5. It's DAY, and PV has been sustained below            -> ECO
         LOW_PV_THRESHOLD_W for the last
         LOW_PV_SUSTAINED_RECORDS logged DB rows
      6. It's DAY, otherwise: missing SoC / battery           -> ECO
         voltage / averaged power samples
      7. It's DAY, otherwise: project how many hours the     -> REACTIVE if
         battery's remaining reserve (soc% of                   projected to
         BATTERY_CAPACITY_AH) will last at the current           outlast
         net discharge rate, and compare against the hours        sunrise +
         remaining until the NEXT SUNRISE plus a safety            margin,
         margin (confirmed with the user: the projection          else ECO
         target stays "survive the night", unchanged from
         the original rule)
    """
    if not available:
        return ECO

    if lat is None or lon is None:
        return ECO

    sunrise_ts = next_sunrise_utc(lat, lon, after_ts=now_ts)
    sunset_ts = next_sunset_utc(lat, lon, after_ts=now_ts)
    if sunrise_ts is None or sunset_ts is None:
        # Polar day/night at this latitude, or some other edge case
        # next_sunrise_utc()/next_sunset_utc() can't resolve.
        return ECO

    # Whichever solar event comes next tells us which side of it we're
    # currently on: if sunrise is the next event, we're still in the
    # night; if sunset is the next event, we're in the day.
    is_night = sunrise_ts < sunset_ts
    if is_night:
        return ECO

    if sustained_low_pv:
        return ECO

    if battery_soc is None or battery_voltage is None or battery_voltage <= 0:
        return ECO

    if avg_load_power is None or avg_pv_power is None:
        return ECO

    now = now_ts if now_ts is not None else time.time()
    hours_until_sunrise = (sunrise_ts - now) / 3600.0

    # See link.power_projection.project_remaining_hours (extracted from
    # what used to be inlined here, same math, behavior-preserving).
    remaining_hours = project_remaining_hours(
        avg_load_power, avg_pv_power, battery_soc, battery_voltage, battery_capacity_ah,
    )
    if remaining_hours is None:
        # Shouldn't happen -- already checked above -- but stay on the
        # "uncertainty -> eco" side if it somehow does.
        return ECO
    if remaining_hours == float("inf"):
        # PV output already covers (or exceeds) the load -- no
        # meaningful discharge to project against, so there's nothing
        # keeping this from being reactive (the "treading water" case).
        return REACTIVE

    return REACTIVE if remaining_hours >= hours_until_sunrise + safety_margin_hours else ECO


class CameraPowerPolicy:
    """Background-thread wrapper around decide_camera_mode(): samples
    link.robot_state.RobotState.power_status() every sample_interval
    seconds to build a trailing average, re-evaluates the decision every
    projection_interval seconds, and -- only when the decision actually
    changes -- POSTs the new mode to camera/stream_server.py's /power_mode
    endpoint.

    self.mode is only updated on a SUCCESSFUL notification. If the camera
    process is unreachable (not running yet, mid-restart, ...) the POST
    fails, self.mode is left as it was, and the SAME desired mode is
    simply retried on the next tick -- rather than being silently
    "forgotten" because this process's own idea of the mode moved on
    without the camera ever hearing about it."""

    def __init__(
        self,
        state,
        projection_interval=PROJECTION_INTERVAL_S,
        sample_interval=SAMPLE_INTERVAL_S,
        battery_capacity_ah=BATTERY_CAPACITY_AH,
        safety_margin_hours=SAFETY_MARGIN_HOURS,
        db_path=None,
        low_pv_threshold_w=LOW_PV_THRESHOLD_W,
        low_pv_sustained_records=LOW_PV_SUSTAINED_RECORDS,
    ):
        self.state = state
        self.projection_interval = projection_interval
        self.sample_interval = sample_interval
        self.battery_capacity_ah = battery_capacity_ah
        self.safety_margin_hours = safety_margin_hours
        self.low_pv_threshold_w = low_pv_threshold_w
        self.low_pv_sustained_records = low_pv_sustained_records
        # Same DB the rest of this process already logs to (see
        # link.power_history.resolve_db_path) -- the new sustained-low-PV
        # day rule reads the last few logged rows from it. Not opened
        # here -- fetch_recent_pv_readings() connects lazily per call,
        # same as every other power_history reader/writer.
        self.db_path = db_path or power_history.resolve_db_path()

        self._samples = []  # [(load_power, pv_power), ...] since the last decision tick
        self.mode = None  # None until the first successfully-applied decision
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self):
        last_tick = time.monotonic()
        while self._running:
            self._sample_once()
            if time.monotonic() - last_tick >= self.projection_interval:
                self._decide_and_apply()
                last_tick = time.monotonic()
            time.sleep(self.sample_interval)

    def _sample_once(self):
        try:
            status = self.state.power_status()
        except Exception:
            log.exception("CameraPowerPolicy: power_status() raised while sampling")
            return
        if not status.get("available"):
            return
        load_power = status.get("load_power")
        pv_power = status.get("pv_power")
        if load_power is not None and pv_power is not None:
            self._samples.append((load_power, pv_power))

    def _average_samples(self):
        samples, self._samples = self._samples, []
        if not samples:
            return None, None
        avg_load = sum(s[0] for s in samples) / len(samples)
        avg_pv = sum(s[1] for s in samples) / len(samples)
        return avg_load, avg_pv

    def _decide_and_apply(self):
        avg_load, avg_pv = self._average_samples()
        try:
            power = self.state.power_status()
            position = self.state.status()
        except Exception:
            log.exception("CameraPowerPolicy: failed to read state while deciding")
            return

        mode = decide_camera_mode(
            available=power.get("available"),
            battery_soc=power.get("battery_soc"),
            battery_voltage=power.get("battery_voltage"),
            avg_load_power=avg_load,
            avg_pv_power=avg_pv,
            lat=position.get("current_lat"),
            lon=position.get("current_lon"),
            sustained_low_pv=self._check_sustained_low_pv(),
            battery_capacity_ah=self.battery_capacity_ah,
            safety_margin_hours=self.safety_margin_hours,
        )

        if mode != self.mode:
            if self._notify_camera(mode):
                self.mode = mode
            # else: leave self.mode as-is -- the same desired mode is
            # recomputed and retried on the next tick (see class
            # docstring), so nothing else to do here on failure.

    def _check_sustained_low_pv(self) -> bool:
        """Reads the last LOW_PV_SUSTAINED_RECORDS rows from
        power_history (the actual logged DB, per the user's explicit
        "en BD" -- NOT this class's own short in-memory `_samples`
        buffer) and runs is_sustained_low_pv() on them. Best-effort, same
        spirit as the rest of this class's state reads: a DB error (disk
        issue, brand new install with no rows yet) logs and is treated
        as "not sustained" -- this is an extra ECO trigger on top of the
        main projection, not the primary uncertainty-handling path, so a
        read failure here shouldn't itself force ECO."""
        try:
            readings = power_history.fetch_recent_pv_readings(
                self.db_path, n=self.low_pv_sustained_records,
            )
        except Exception:
            log.exception("CameraPowerPolicy: failed to read power_history for low-PV check")
            return False
        return is_sustained_low_pv(
            readings,
            threshold_w=self.low_pv_threshold_w,
            required_records=self.low_pv_sustained_records,
        )

    def _notify_camera(self, mode) -> bool:
        """POSTs {"mode": mode} to camera/stream_server.py's /power_mode
        endpoint. Never raises -- logs and returns False on any failure,
        same "camera is optional, this must never crash the caller"
        spirit as link.robot_state's _request_snapshot()/
        _request_recording()."""
        host = os.environ.get("CAMERA_HOST", CAMERA_HOST_DEFAULT)
        port = int(os.environ.get("CAMERA_PORT", CAMERA_PORT_DEFAULT))
        path = os.environ.get("CAMERA_POWER_MODE_PATH", CAMERA_POWER_MODE_PATH_DEFAULT)
        url = f"http://{host}:{port}{path}"
        body = json.dumps({"mode": mode}).encode("utf-8")
        request = urllib.request.Request(
            url, data=body, method="POST", headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=CAMERA_POWER_MODE_TIMEOUT) as resp:
                if resp.status != 200:
                    log.warning("CameraPowerPolicy: /power_mode returned HTTP %s", resp.status)
                    return False
            return True
        except urllib.error.HTTPError as exc:
            log.warning("CameraPowerPolicy: /power_mode HTTP error %s", exc.code)
            return False
        except urllib.error.URLError as exc:
            log.warning("CameraPowerPolicy: camera process unreachable (%s)", exc.reason)
            return False
