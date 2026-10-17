"""Tests for link/led_control.py: the sysfs write helpers (against a fake
sysfs tree under tmp_path, never the real /sys/class/leds/) and
LedController's pure timing/transition logic -- no real background
thread involved (same "manually construct it, never call .start(), drive
it by hand" approach test_link_server.py's own power_history tests use
to avoid racing a real thread's timing), no server/socket involved,
same style as tests/test_solar_map.py for this project's other
standalone-logic modules.

Run with:
    pytest tests/test_led_control.py
"""
import time

import pytest

from link import led_control


# --- _write_led_file / turn_led_off / turn_led_on (fake sysfs tree) --------

@pytest.fixture()
def fake_leds_root(tmp_path, monkeypatch):
    """Points link.led_control.LEDS_SYSFS_ROOT at a throwaway directory
    with led0/led1 subdirectories already created (mirroring a real
    /sys/class/leds/ tree enough for open(path, "w") to succeed) --
    real hardware is never touched by these tests."""
    (tmp_path / "led0").mkdir()
    (tmp_path / "led1").mkdir()
    monkeypatch.setattr(led_control, "LEDS_SYSFS_ROOT", str(tmp_path))
    return tmp_path


def test_write_led_file_writes_the_value(fake_leds_root):
    ok = led_control._write_led_file("led0", "brightness", 0)
    assert ok is True
    assert (fake_leds_root / "led0" / "brightness").read_text() == "0"


def test_write_led_file_returns_false_and_does_not_raise_on_a_missing_path(fake_leds_root):
    # "led9" was never created under fake_leds_root -- same shape as a
    # wrong ACT_LED_NAME/PWR_LED_NAME on the real hardware, or no
    # /sys/class/leds/ at all on a dev machine. Must degrade to a
    # logged warning (see link.led_control's own docstring), never raise.
    ok = led_control._write_led_file("led9", "brightness", 0)
    assert ok is False


def test_turn_led_off_clears_trigger_and_zeroes_brightness(fake_leds_root):
    led_control.turn_led_off("led0")
    assert (fake_leds_root / "led0" / "trigger").read_text() == "none"
    assert (fake_leds_root / "led0" / "brightness").read_text() == "0"


def test_turn_led_on_restores_trigger_and_sets_brightness_one(fake_leds_root):
    led_control.turn_led_on("led1", "default-on")
    assert (fake_leds_root / "led1" / "trigger").read_text() == "default-on"
    assert (fake_leds_root / "led1" / "brightness").read_text() == "1"


# --- LedController: timer/transition logic ----------------------------------
# Explicit user request (2026-10-10): "si le panneau solaire ne capte
# qu'une tension inférieure à 15Volt pendant plus de 10 minutes alors les
# deux Leds doivent s'éteindre et lorsque le soleil revient alors les
# leds redeviennent actives" -- threshold_v/duration_s below are passed
# explicitly (rather than relying on the module's own 15.0V/600s
# defaults) purely so these tests stay readable with small round numbers
# and aren't silently broken by a future default change; the defaults
# themselves are covered by test_led_controller_default_thresholds_match_
# the_user_request below.

class _FakeState:
    """Minimal stand-in for link.robot_state.RobotState -- LedController
    only ever calls .power_status(), so nothing else needs to exist."""

    def __init__(self, available=False, pv_voltage=None):
        self._status = {"available": available, "pv_voltage": pv_voltage}

    def set(self, available, pv_voltage):
        self._status = {"available": available, "pv_voltage": pv_voltage}

    def power_status(self):
        return self._status


def _make_controller(monkeypatch, fake_leds_root, threshold_v=15.0, duration_s=600.0):
    state = _FakeState()
    controller = led_control.LedController(
        state, threshold_v=threshold_v, duration_s=duration_s,
    )
    return state, controller


def test_led_controller_default_thresholds_match_the_user_request():
    # 15V / 10 minutes, explicit user request -- see this module's
    # docstring and led_control.py's own LOW_VOLTAGE_THRESHOLD_V/
    # LOW_VOLTAGE_DURATION_S.
    assert led_control.LOW_VOLTAGE_THRESHOLD_V == 15.0
    assert led_control.LOW_VOLTAGE_DURATION_S == 10 * 60


def test_led_controller_ignores_ticks_while_tracer_is_unavailable(monkeypatch, fake_leds_root):
    # No real reading at all (RS485 cable unplugged, Tracer unpowered) --
    # must neither start nor advance the low-voltage timer, and never
    # touch the LEDs. See LedController's own docstring for why "wait,
    # don't guess" is the deliberate choice here.
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    state.set(available=False, pv_voltage=5.0)  # a real "low" value, but unavailable
    controller._check_once()
    assert controller._low_since is None
    assert controller._leds_off is False
    assert not (fake_leds_root / "led0" / "trigger").exists()


def test_led_controller_starts_the_timer_on_a_low_reading_but_does_not_act_yet(monkeypatch, fake_leds_root):
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)  # below the 15V threshold
    controller._check_once()
    assert controller._low_since == 1000.0
    assert controller._leds_off is False  # not yet -- duration hasn't elapsed
    assert not (fake_leds_root / "led0" / "trigger").exists()


def test_led_controller_does_not_turn_leds_off_before_the_duration_elapses(monkeypatch, fake_leds_root):
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()  # starts the timer at t=1000

    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 599)  # 1s short of 600s
    controller._check_once()
    assert controller._leds_off is False
    assert not (fake_leds_root / "led0" / "trigger").exists()


def test_led_controller_turns_both_leds_off_once_the_duration_elapses(monkeypatch, fake_leds_root):
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()  # starts the timer at t=1000

    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 600)  # exactly the threshold
    controller._check_once()
    assert controller._leds_off is True
    assert (fake_leds_root / "led0" / "trigger").read_text() == "none"
    assert (fake_leds_root / "led0" / "brightness").read_text() == "0"
    assert (fake_leds_root / "led1" / "trigger").read_text() == "none"
    assert (fake_leds_root / "led1" / "brightness").read_text() == "0"


def test_led_controller_only_writes_to_sysfs_once_on_the_off_transition(monkeypatch, fake_leds_root):
    # Once the LEDs are off, further low-voltage ticks must not keep
    # re-writing the same sysfs files on every single check (harmless on
    # real hardware, but pointless I/O and log noise over a whole night).
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 600)
    controller._check_once()
    (fake_leds_root / "led0" / "brightness").write_text("sentinel")  # mark it

    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 700)
    state.set(available=True, pv_voltage=10.0)  # still low
    controller._check_once()
    assert (fake_leds_root / "led0" / "brightness").read_text() == "sentinel"  # untouched


def test_led_controller_turns_leds_back_on_once_voltage_recovers(monkeypatch, fake_leds_root):
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 600)
    controller._check_once()
    assert controller._leds_off is True  # off, as in the test above

    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 610)
    state.set(available=True, pv_voltage=18.0)  # the sun is back
    controller._check_once()
    assert controller._leds_off is False
    assert controller._low_since is None
    assert (fake_leds_root / "led0" / "trigger").read_text() == led_control.ACT_LED_RESTORE_TRIGGER
    assert (fake_leds_root / "led0" / "brightness").read_text() == "1"
    assert (fake_leds_root / "led1" / "trigger").read_text() == led_control.PWR_LED_RESTORE_TRIGGER
    assert (fake_leds_root / "led1" / "brightness").read_text() == "1"


def test_led_controller_recovery_resets_the_timer_even_before_leds_were_off(monkeypatch, fake_leds_root):
    # A reading back at/above the threshold always resets _low_since,
    # whether or not the duration had already elapsed -- a brief dip
    # that recovers well under 10 minutes must not leave a stale timer
    # lying around to combine with some LATER, unrelated dip.
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()
    assert controller._low_since == 1000.0

    monkeypatch.setattr(time, "monotonic", lambda: 1030.0)
    state.set(available=True, pv_voltage=16.0)
    controller._check_once()
    assert controller._low_since is None
    assert controller._leds_off is False
    assert not (fake_leds_root / "led0" / "trigger").exists()  # never touched -- was never off


def test_led_controller_an_unavailable_tick_does_not_reset_an_in_progress_timer(monkeypatch, fake_leds_root):
    # Same "wait, don't guess" rule as the ignores-while-unavailable test
    # above, from the other direction: a brief Tracer dropout in the
    # MIDDLE of an already-running low-voltage timer must not reset it
    # back to None (that would effectively restart the 10-minute clock
    # over a momentary RS485 hiccup, not an actual return of daylight).
    state, controller = _make_controller(monkeypatch, fake_leds_root)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=10.0)
    controller._check_once()
    assert controller._low_since == 1000.0

    monkeypatch.setattr(time, "monotonic", lambda: 1100.0)
    state.set(available=False, pv_voltage=None)  # Tracer briefly unavailable
    controller._check_once()
    assert controller._low_since == 1000.0  # unchanged, not reset to None

    monkeypatch.setattr(time, "monotonic", lambda: 1000.0 + 600)
    state.set(available=True, pv_voltage=9.0)  # back, and still low
    controller._check_once()
    assert controller._leds_off is True  # the original timer still counted


def test_led_controller_right_at_the_threshold_voltage_does_not_count_as_low(monkeypatch, fake_leds_root):
    # pv_voltage < threshold_v, strictly -- a reading exactly AT the
    # threshold is "fine", matching STATUS_ZONES-style falling/rising
    # breakpoints elsewhere in this project (robot-webserver's power.html)
    # being inclusive on the "good" side.
    state, controller = _make_controller(monkeypatch, fake_leds_root, threshold_v=15.0)
    monkeypatch.setattr(time, "monotonic", lambda: 1000.0)
    state.set(available=True, pv_voltage=15.0)
    controller._check_once()
    assert controller._low_since is None
