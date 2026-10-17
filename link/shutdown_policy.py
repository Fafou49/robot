"""Pi #1 self-shutdown policy.

Explicit user request (2026-10-12), verbatim:

    "je veux un arret automatique de PI1 1h avant la fin de la batterie
    pour eteindre proprement [...] Si l'arret programme par la Pi1
    elle meme est calcule pendant la nuit, on eteins des la tombee de
    la nuit pour garder de la batterie pour le redemarrage."

    (I want Pi1 to automatically shut down 1h before the battery runs
    out, for a clean shutdown [...] if the shutdown Pi1 itself computes
    is calculated during the night, shut down right at nightfall
    instead, to preserve battery for the restart)

This module is the direct sibling of link.camera_power_policy: same
"pure decision function, separate I/O-performing background-thread
wrapper" split, same PowerAverager-based trailing average (see
link.power_projection) -- but its own, SEPARATE averager instance, not
shared state with CameraPowerPolicy (this project's established
"independent, decoupled background threads" convention).

One deliberate inversion from the camera policy's philosophy, because
the two mistakes have very different costs: for the camera,
"uncertainty -> eco" (wrongly staying eco just wastes a frame or two);
here, wrongly triggering a real poweroff is far more consequential than
wrongly NOT triggering one, so every uncertain/missing-data case below
resolves to "do NOT schedule a shutdown" (None), the mirror image of
the camera policy's "uncertainty -> eco".

The single "is anything wrong" condition this module schedules against
is deliberately the SAME one the shutdown-alert banner mockup was built
around and the one link.camera_power_policy's day-branch already uses:
will the battery's reserve, discharging at the current averaged net
rate, last until the next sunrise plus a safety margin? Reusing this
one condition (rather than a bare "battery will hit 0% in N hours")
means the real auto-shutdown only ever arms itself in exactly the
scenario the banner is about -- "can't reach the next charging day" --
not on some harmless, ordinary daytime dip that the sun is about to fix
anyway.

Honesty note: SHUTDOWN_LEAD_HOURS=1.0 is the user's own explicit value.
BATTERY_CAPACITY_AH and SAFETY_MARGIN_HOURS are link.power_projection's
(see that module's docstring for their provenance) -- reused here
rather than re-specified, so there is one answer to "how big is the
battery" / "how much margin" across the whole project, not two that
could silently drift apart.
"""
import logging
import threading
import time

from link.power_projection import (
    BATTERY_CAPACITY_AH,
    PROJECTION_INTERVAL_S,
    SAFETY_MARGIN_HOURS,
    SAMPLE_INTERVAL_S,
    PowerAverager,
    project_remaining_hours,
)
from link.sunrise import next_sunrise_utc, next_sunset_utc

log = logging.getLogger(__name__)

# The user's own explicit value: "1h avant la fin de la batterie".
SHUTDOWN_LEAD_HOURS = 1.0


def compute_scheduled_shutdown_ts(
    *,
    available,
    battery_soc,
    battery_voltage,
    avg_load_power,
    avg_pv_power,
    lat,
    lon,
    now_ts=None,
    battery_capacity_ah=BATTERY_CAPACITY_AH,
    lead_hours=SHUTDOWN_LEAD_HOURS,
    safety_margin_hours=SAFETY_MARGIN_HOURS,
):
    """Pure decision function -- see this module's docstring. Never
    raises, never blocks (next_sunrise_utc()/next_sunset_utc() are pure
    math, same as link.camera_power_policy's usage).

    Returns None when no shutdown is scheduled -- either because the
    inputs are missing/uncertain (the "uncertainty -> do NOT shut down"
    side of this module), or because the projected reserve is fine
    (will reach the next sunrise with the safety margin intact, or PV
    already covers the load outright). Otherwise returns a Unix
    timestamp: the single authoritative "Pi1 will shut itself down at
    this time" answer, computed as follows:

      1. Tracer/power data not available, or no GPS fix (no lat/lon),
         or day/night can't be determined at all (polar day/night) ->
         None (mirrors link.camera_power_policy's own first 3 checks,
         same reasoning -- can't tell, so don't act).
      2. Missing SoC / battery voltage / averaged power samples -> None.
      3. Project the remaining hours of reserve at the current averaged
         net discharge rate (link.power_projection.project_remaining_hours).
         If PV already covers the load (no net discharge) -> None,
         nothing to schedule.
      4. If that projection says the reserve WILL last until the next
         sunrise plus safety_margin_hours -> None, no alarm condition,
         nothing to schedule.
      5. Otherwise (won't make it to the next sunrise): this is the
         alarm condition --
           - if it is currently NIGHT right now, the nightfall override
             applies: don't wait for the "naturally" computed time
             (which could be hours away, deep in the night, burning
             reserve needed for tomorrow's restart) -- return now_ts,
             shut down immediately.
           - otherwise (currently day), return the naturally computed
             time: lead_hours before the battery is projected to be
             empty at the current rate -- never earlier than now_ts,
             in case that projected moment has already passed.
    """
    if not available:
        return None
    if lat is None or lon is None:
        return None

    now = now_ts if now_ts is not None else time.time()
    sunrise_ts = next_sunrise_utc(lat, lon, after_ts=now)
    sunset_ts = next_sunset_utc(lat, lon, after_ts=now)
    if sunrise_ts is None or sunset_ts is None:
        # Polar day/night at this latitude, or some other edge case
        # next_sunrise_utc()/next_sunset_utc() can't resolve.
        return None

    # Same convention as link.camera_power_policy.decide_camera_mode:
    # whichever solar event comes next tells us which side of it we're
    # currently on.
    is_night = sunrise_ts < sunset_ts

    remaining_hours = project_remaining_hours(
        avg_load_power, avg_pv_power, battery_soc, battery_voltage, battery_capacity_ah,
    )
    if remaining_hours is None:
        return None
    if remaining_hours == float("inf"):
        # PV output already covers (or exceeds) the load -- nothing to
        # schedule.
        return None

    hours_until_sunrise = (sunrise_ts - now) / 3600.0
    will_survive_the_night = remaining_hours >= hours_until_sunrise + safety_margin_hours
    if will_survive_the_night:
        return None

    if is_night:
        # Nightfall override -- see this function's docstring, point 5.
        return now

    naturally_scheduled_ts = now + (remaining_hours - lead_hours) * 3600.0
    return max(now, naturally_scheduled_ts)


def should_shutdown_now(scheduled_shutdown_ts, now_ts=None):
    """Thin boolean wrapper: True once `now_ts` has reached
    `scheduled_shutdown_ts`. False (never shuts down) when nothing is
    scheduled at all."""
    if scheduled_shutdown_ts is None:
        return False
    now = now_ts if now_ts is not None else time.time()
    return now >= scheduled_shutdown_ts


class ShutdownPolicy:
    """Background-thread wrapper around compute_scheduled_shutdown_ts():
    samples link.robot_state.RobotState.power_status() every
    sample_interval seconds to build its OWN trailing average (a
    separate link.power_projection.PowerAverager instance -- not shared
    with CameraPowerPolicy), re-evaluates every projection_interval
    seconds, writes the result to RobotState (so it can be reported in
    the PWR sentence -- see link.robot_state.set_shutdown_eta()), and --
    exactly once -- invokes `on_shutdown` the first time
    should_shutdown_now() becomes true.

    `on_shutdown` is expected to be link.server.ControlServer._shutdown_pi
    (the already-existing, already-tested shutdown mechanism -- this
    class never runs a shutdown command itself, same as the gamepad's
    BTN_START handler, which wires the very same method as its own
    on_shutdown callback)."""

    def __init__(
        self,
        state,
        on_shutdown,
        projection_interval=PROJECTION_INTERVAL_S,
        sample_interval=SAMPLE_INTERVAL_S,
        battery_capacity_ah=BATTERY_CAPACITY_AH,
        lead_hours=SHUTDOWN_LEAD_HOURS,
        safety_margin_hours=SAFETY_MARGIN_HOURS,
    ):
        self.state = state
        self.on_shutdown = on_shutdown
        self.projection_interval = projection_interval
        self.sample_interval = sample_interval
        self.battery_capacity_ah = battery_capacity_ah
        self.lead_hours = lead_hours
        self.safety_margin_hours = safety_margin_hours

        self._averager = PowerAverager()
        self._running = False
        self._thread = None
        # Guard so a real poweroff is requested at most once, even if
        # this policy's loop ticks again before the process actually
        # exits (sudo poweroff is not instantaneous).
        self._shutdown_triggered = False

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
            log.exception("ShutdownPolicy: power_status() raised while sampling")
            return
        if not status.get("available"):
            return
        self._averager.sample(status.get("load_power"), status.get("pv_power"))

    def _decide_and_apply(self):
        avg_load, avg_pv = self._averager.average()
        try:
            power = self.state.power_status()
            position = self.state.status()
        except Exception:
            log.exception("ShutdownPolicy: failed to read state while deciding")
            return

        scheduled_ts = compute_scheduled_shutdown_ts(
            available=power.get("available"),
            battery_soc=power.get("battery_soc"),
            battery_voltage=power.get("battery_voltage"),
            avg_load_power=avg_load,
            avg_pv_power=avg_pv,
            lat=position.get("current_lat"),
            lon=position.get("current_lon"),
            battery_capacity_ah=self.battery_capacity_ah,
            lead_hours=self.lead_hours,
            safety_margin_hours=self.safety_margin_hours,
        )

        try:
            self.state.set_shutdown_eta(scheduled_ts)
        except Exception:
            log.exception("ShutdownPolicy: failed to record shutdown_eta_ts on state")

        if self._shutdown_triggered:
            return

        if should_shutdown_now(scheduled_ts):
            self._shutdown_triggered = True
            log.warning(
                "ShutdownPolicy: scheduled self-shutdown reached (scheduled_ts=%s) -- "
                "triggering clean shutdown now",
                scheduled_ts,
            )
            try:
                self.on_shutdown()
            except Exception:
                log.exception("ShutdownPolicy: on_shutdown callback raised")
