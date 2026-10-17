"""Shared battery-reserve projection primitives.

Extracted (2026-10-12) out of link.camera_power_policy so the new
link.shutdown_policy can use the exact same "how many hours of reserve
are left, at the current averaged discharge rate" math without
duplicating it or sharing mutable state with the camera policy.

This module intentionally contains NO policy decisions (no REACTIVE/ECO,
no shutdown-or-not) -- just the two reusable building blocks:

  - PowerAverager: a tiny trailing-average accumulator. Each policy that
    needs one constructs its OWN instance (CameraPowerPolicy keeps its
    existing hand-rolled _samples/_sample_once/_average_samples
    untouched, to avoid any regression risk to its existing test suite;
    ShutdownPolicy below uses a fresh PowerAverager of its own). This
    mirrors the project's established "separate, decoupled background
    threads, no shared mutable state between them" convention.
  - project_remaining_hours(): the pure linear-discharge projection
    formula, unchanged from what used to be inlined at the end of
    link.camera_power_policy.decide_camera_mode().

BATTERY_CAPACITY_AH and SAFETY_MARGIN_HOURS are re-exported here with
the same values link.camera_power_policy already used (see that
module's own docstring for their provenance/honesty notes); it imports
them back from here so there is a single source of truth.
"""

# Reused as-is from pages/power.html's own existing battery capacity
# constant for this robot -- see link.camera_power_policy's docstring.
BATTERY_CAPACITY_AH = 35
# Conservative, simple default -- see link.camera_power_policy's docstring.
SAFETY_MARGIN_HOURS = 1.0

# How often a PowerAverager-backed policy re-evaluates its decision --
# matches link.camera_power_policy's PROJECTION_INTERVAL_S.
PROJECTION_INTERVAL_S = 5 * 60
# How often it samples power_status() to feed that average -- matches
# link.camera_power_policy's SAMPLE_INTERVAL_S.
SAMPLE_INTERVAL_S = 15


class PowerAverager:
    """Tiny trailing-average accumulator: sample(load, pv) as readings
    come in, average() drains the buffer and returns (avg_load, avg_pv)
    -- or (None, None) if nothing was sampled since the last average()
    call. Not thread-safe by itself; callers that sample and average
    from different threads must serialize around it (the existing
    CameraPowerPolicy pattern this mirrors doesn't need to, since it
    only ever touches _samples from its own single background thread --
    same assumption holds here)."""

    def __init__(self):
        self._samples = []  # [(load_power, pv_power), ...]

    def sample(self, load_power, pv_power):
        if load_power is not None and pv_power is not None:
            self._samples.append((load_power, pv_power))

    def average(self):
        samples, self._samples = self._samples, []
        if not samples:
            return None, None
        avg_load = sum(s[0] for s in samples) / len(samples)
        avg_pv = sum(s[1] for s in samples) / len(samples)
        return avg_load, avg_pv


def project_remaining_hours(
    avg_load_power,
    avg_pv_power,
    battery_soc,
    battery_voltage,
    battery_capacity_ah=BATTERY_CAPACITY_AH,
):
    """Pure projection, unchanged math from the former inline formula in
    link.camera_power_policy.decide_camera_mode():

      remaining_ah = soc% of battery_capacity_ah
      discharge_current_a = net_discharge_w / battery_voltage
      remaining_hours = remaining_ah / discharge_current_a

    Returns:
      - None if any required input is missing/invalid (battery_soc,
        battery_voltage <= 0, avg_load_power, or avg_pv_power) -- the
        caller's "uncertainty" branch.
      - float('inf') if PV output already covers (or exceeds) the load
        -- no meaningful discharge to project against (the "treading
        water" case).
      - otherwise, the projected remaining hours of reserve at the
        current averaged net discharge rate.
    """
    if battery_soc is None or battery_voltage is None or battery_voltage <= 0:
        return None
    if avg_load_power is None or avg_pv_power is None:
        return None

    net_discharge_w = max(0.0, avg_load_power - avg_pv_power)
    if net_discharge_w < 0.01:
        return float("inf")

    remaining_ah = (max(0.0, battery_soc) / 100.0) * battery_capacity_ah
    discharge_current_a = net_discharge_w / battery_voltage
    return remaining_ah / discharge_current_a
