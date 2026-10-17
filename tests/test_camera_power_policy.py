"""Tests for link/camera_power_policy.py: decide_camera_mode()'s pure
branch logic (no I/O, no mocking needed at all) and CameraPowerPolicy's
sampling/averaging/notification-retry logic, manually constructed and
driven by hand -- never a real background thread, never a real HTTP
call -- same "manually construct it, never call .start(), drive it by
hand" approach as tests/test_led_control.py's own LedController tests.

Run with:
    pytest tests/test_camera_power_policy.py
"""
import pytest

from link import camera_power_policy as cpp


# --- decide_camera_mode(): pure branch coverage -----------------------------
# A fixed "now" and a fixed (lat, lon) with a known, unambiguous sunrise
# time keeps every test deterministic and independent of the sandbox's
# real wall-clock date. Equator, lon=0 -- link.sunrise's own invariant
# (see tests/test_sunrise.py) puts sunrise at ~356 minutes (~5:56) after
# UTC midnight on any date; these tests use midnight UTC as `now_ts`, so
# "hours_until_sunrise" is always close to ~5.93h regardless of which
# date actually runs them.
LAT, LON = 0.0, 0.0
NOW_TS = 1_800_000_000.0  # 2027-01-15 08:00:00 UTC -- an arbitrary fixed instant;
# next_sunrise_utc(0.0, 0.0, after_ts=NOW_TS) lands at ~2027-01-16 06:05:55 UTC,
# i.e. hours_until_sunrise ~= 22.10h from NOW_TS (confirmed directly against
# link.sunrise.next_sunrise_utc, not re-derived by hand here).


def _decide(**overrides):
    kwargs = dict(
        available=True,
        battery_charging_current=0.0,
        battery_soc=80.0,
        battery_voltage=24.0,
        avg_load_power=20.0,
        avg_pv_power=0.0,
        lat=LAT,
        lon=LON,
        now_ts=NOW_TS,
    )
    kwargs.update(overrides)
    return cpp.decide_camera_mode(**kwargs)


def test_unavailable_power_data_means_eco():
    assert _decide(available=False) == cpp.ECO


def test_charging_means_reactive_regardless_of_projection():
    # battery_soc/avg_load_power below would, on their own, project well
    # short of sunrise -- charging overrides that unconditionally.
    assert _decide(
        battery_charging_current=1.5, battery_soc=1.0, avg_load_power=1000.0,
    ) == cpp.REACTIVE


def test_zero_charging_current_is_not_treated_as_charging():
    # Exactly 0.0 (not actually charging) must fall through to the
    # projection, not be treated as a truthy "is charging" signal.
    assert _decide(battery_charging_current=0.0, battery_soc=1.0, avg_load_power=1000.0) == cpp.ECO


def test_missing_gps_fix_means_eco():
    assert _decide(lat=None, lon=None) == cpp.ECO
    assert _decide(lat=LAT, lon=None) == cpp.ECO


def test_missing_battery_soc_means_eco():
    assert _decide(battery_soc=None) == cpp.ECO


def test_missing_battery_voltage_means_eco():
    assert _decide(battery_voltage=None) == cpp.ECO


def test_zero_battery_voltage_means_eco():
    # Would divide by zero in the discharge-current projection otherwise.
    assert _decide(battery_voltage=0.0) == cpp.ECO


def test_missing_averaged_power_means_eco():
    assert _decide(avg_load_power=None) == cpp.ECO
    assert _decide(avg_pv_power=None) == cpp.ECO


def test_pv_covering_the_load_means_reactive():
    # avg_pv_power >= avg_load_power -- no net discharge to project
    # against, nothing stopping reactive mode.
    assert _decide(avg_load_power=20.0, avg_pv_power=25.0) == cpp.REACTIVE
    assert _decide(avg_load_power=20.0, avg_pv_power=20.0) == cpp.REACTIVE


def test_plenty_of_reserve_projects_reactive():
    # Full battery, tiny load -- comfortably outlasts sunrise.
    assert _decide(battery_soc=100.0, avg_load_power=5.0, avg_pv_power=0.0) == cpp.REACTIVE


def test_thin_reserve_projects_eco():
    # Nearly empty battery, heavy load -- will not last until sunrise.
    assert _decide(battery_soc=2.0, avg_load_power=50.0, avg_pv_power=0.0) == cpp.ECO


def test_safety_margin_tips_a_borderline_case_to_eco():
    # Pick battery_soc so remaining_hours (~22.5h) lands just ABOVE
    # hours_until_sunrise (~22.10h, see NOW_TS's comment above) but just
    # BELOW hours_until_sunrise + margin (~23.10h) -- must resolve to
    # ECO, not REACTIVE, i.e. the margin is actually enforced and isn't
    # a no-op.
    # discharge_current_a = 20/24 = 0.8333 A. remaining_ah for
    # remaining_hours=22.5 = 22.5*0.8333 = 18.75 Ah -> battery_soc =
    # 18.75/35*100 ~= 53.571%.
    mode_with_margin = _decide(battery_soc=53.571, avg_load_power=20.0, avg_pv_power=0.0, safety_margin_hours=1.0)
    assert mode_with_margin == cpp.ECO
    # With the margin removed entirely, that same reserve clears the
    # (now margin-less) bar.
    mode_without_margin = _decide(battery_soc=53.571, avg_load_power=20.0, avg_pv_power=0.0, safety_margin_hours=0.0)
    assert mode_without_margin == cpp.REACTIVE


def test_polar_night_latitude_with_no_resolvable_sunrise_means_eco(monkeypatch):
    # next_sunrise_utc() returning None (polar day/night on both days it
    # checks) must resolve to ECO, not raise and not guess REACTIVE.
    monkeypatch.setattr(cpp, "next_sunrise_utc", lambda lat, lon, after_ts=None: None)
    assert _decide() == cpp.ECO


# --- CameraPowerPolicy: sampling / averaging --------------------------------

class _FakeState:
    """Minimal stand-in for link.robot_state.RobotState -- CameraPowerPolicy
    only ever calls .power_status() and .status()."""

    def __init__(self, power=None, position=None):
        self._power = power or {"available": False}
        self._position = position or {"current_lat": None, "current_lon": None}

    def set_power(self, **kwargs):
        self._power = kwargs

    def set_position(self, lat, lon):
        self._position = {"current_lat": lat, "current_lon": lon}

    def power_status(self):
        return self._power

    def status(self):
        return self._position


def _make_policy(**kwargs):
    state = _FakeState()
    policy = cpp.CameraPowerPolicy(state, **kwargs)
    return state, policy


def test_sample_once_ignores_an_unavailable_reading():
    state, policy = _make_policy()
    state.set_power(available=False, load_power=99.0, pv_power=1.0)
    policy._sample_once()
    assert policy._samples == []


def test_sample_once_ignores_a_reading_missing_load_or_pv_power():
    state, policy = _make_policy()
    state.set_power(available=True, load_power=None, pv_power=1.0)
    policy._sample_once()
    state.set_power(available=True, load_power=5.0, pv_power=None)
    policy._sample_once()
    assert policy._samples == []


def test_sample_once_collects_an_available_reading():
    state, policy = _make_policy()
    state.set_power(available=True, load_power=10.0, pv_power=2.0)
    policy._sample_once()
    assert policy._samples == [(10.0, 2.0)]


def test_average_samples_returns_the_mean_and_clears_the_buffer():
    state, policy = _make_policy()
    policy._samples = [(10.0, 0.0), (20.0, 4.0)]
    avg_load, avg_pv = policy._average_samples()
    assert avg_load == pytest.approx(15.0)
    assert avg_pv == pytest.approx(2.0)
    assert policy._samples == []


def test_average_samples_returns_none_none_when_empty():
    state, policy = _make_policy()
    avg_load, avg_pv = policy._average_samples()
    assert (avg_load, avg_pv) == (None, None)


# --- CameraPowerPolicy: decide-and-apply / notification retry ---------------

class _FakeNotifier:
    """Replaces CameraPowerPolicy._notify_camera for the tests below --
    records every mode it was asked to send and returns a scripted
    success/failure per call, so these tests never touch a real socket."""

    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def __call__(self, mode):
        self.calls.append(mode)
        return self.results.pop(0) if self.results else True


def test_decide_and_apply_applies_the_first_decision_on_success(monkeypatch):
    state, policy = _make_policy()
    state.set_power(
        available=True, battery_charging_current=1.0, battery_soc=80.0,
        battery_voltage=24.0, load_power=20.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    notifier = _FakeNotifier([True])
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    assert policy.mode is None
    policy._decide_and_apply()
    assert policy.mode == cpp.REACTIVE
    assert notifier.calls == [cpp.REACTIVE]


def test_decide_and_apply_does_not_notify_when_the_mode_is_unchanged(monkeypatch):
    state, policy = _make_policy()
    state.set_power(
        available=True, battery_charging_current=1.0, battery_soc=80.0,
        battery_voltage=24.0, load_power=20.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    policy.mode = cpp.REACTIVE  # already reactive from a previous tick
    notifier = _FakeNotifier([True])
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    policy._decide_and_apply()
    assert notifier.calls == []  # never called -- nothing changed
    assert policy.mode == cpp.REACTIVE


def test_decide_and_apply_leaves_mode_unset_when_notification_fails(monkeypatch):
    state, policy = _make_policy()
    state.set_power(
        available=True, battery_charging_current=1.0, battery_soc=80.0,
        battery_voltage=24.0, load_power=20.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    notifier = _FakeNotifier([False])  # camera process unreachable
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    policy._decide_and_apply()
    assert notifier.calls == [cpp.REACTIVE]
    assert policy.mode is None  # NOT updated -- failure, not forgotten


def test_decide_and_apply_retries_the_same_mode_on_the_next_tick_after_a_failure(monkeypatch):
    # The core "retry, never forget" guarantee from the module's own
    # docstring: a failed POST must not stop the SAME desired mode from
    # being retried on the very next tick.
    state, policy = _make_policy()
    state.set_power(
        available=True, battery_charging_current=1.0, battery_soc=80.0,
        battery_voltage=24.0, load_power=20.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    notifier = _FakeNotifier([False, True])  # fails once, then succeeds
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    policy._decide_and_apply()
    assert policy.mode is None
    policy._decide_and_apply()
    assert policy.mode == cpp.REACTIVE
    assert notifier.calls == [cpp.REACTIVE, cpp.REACTIVE]


def test_decide_and_apply_uses_the_averaged_samples_since_the_last_tick(monkeypatch):
    state, policy = _make_policy()
    # Heavy instantaneous load/pv in power_status() itself must NOT be
    # what the decision uses -- only the averaged samples collected via
    # _sample_once() in between ticks matter for avg_load_power/avg_pv_power.
    state.set_power(
        available=True, battery_charging_current=0.0, battery_soc=2.0,
        battery_voltage=24.0, load_power=999.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    # ...but the averaged samples show a light, PV-covered load -- should
    # project REACTIVE despite the low SoC and the scary instantaneous
    # load_power reading above.
    policy._samples = [(5.0, 10.0), (5.0, 10.0)]
    notifier = _FakeNotifier([True])
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    policy._decide_and_apply()
    assert policy.mode == cpp.REACTIVE


def test_decide_and_apply_falls_back_to_eco_when_no_samples_were_collected(monkeypatch):
    state, policy = _make_policy()
    state.set_power(
        available=True, battery_charging_current=0.0, battery_soc=80.0,
        battery_voltage=24.0, load_power=20.0, pv_power=0.0,
    )
    state.set_position(LAT, LON)
    # No _sample_once() calls at all this tick -- avg_load/avg_pv are None.
    notifier = _FakeNotifier([True])
    monkeypatch.setattr(policy, "_notify_camera", notifier)

    policy._decide_and_apply()
    assert policy.mode == cpp.ECO
