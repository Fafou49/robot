"""Tests for link/voltage_safety.py: is_voltage_critical()/
decide_voltage_safety_action()/update_auto_lockout()'s pure branch logic
(no I/O, no mocking needed), and VoltageSafety's sampling/deciding/
rumble/force-idle wiring, manually constructed and driven by hand --
never a real background thread, never a real gamepad -- same approach
as tests/test_shutdown_policy.py.

Run with:
    pytest tests/test_voltage_safety.py
"""
import pytest

from link import voltage_safety as vs
from link.robot_state import CommandError, RobotState

CRITICAL = vs.CRITICAL_VOLTAGE_THRESHOLD_V  # 11.5
RECOVERY = vs.RECOVERY_VOLTAGE_THRESHOLD_V  # 12.0


# --- is_voltage_critical() --------------------------------------------------

def test_is_voltage_critical_true_at_or_below_the_threshold():
    assert vs.is_voltage_critical(CRITICAL) is True
    assert vs.is_voltage_critical(CRITICAL - 0.1) is True


def test_is_voltage_critical_false_above_the_threshold():
    assert vs.is_voltage_critical(CRITICAL + 0.1) is False


def test_is_voltage_critical_false_with_no_reading():
    # Uncertainty -> do not act, same posture as link.shutdown_policy.
    assert vs.is_voltage_critical(None) is False


# --- decide_voltage_safety_action() -----------------------------------------

def test_not_critical_never_rumbles_or_forces_idle():
    assert vs.decide_voltage_safety_action(13.0, "MANUAL", 200, 0) == (False, False)
    assert vs.decide_voltage_safety_action(13.0, "AUTO", 0, 0) == (False, False)
    assert vs.decide_voltage_safety_action(None, "MANUAL", 200, 0) == (False, False)


def test_manual_critical_with_a_genuine_push_rumbles_but_does_not_force_idle():
    assert vs.decide_voltage_safety_action(11.0, "MANUAL", 200, 0) == (True, False)
    assert vs.decide_voltage_safety_action(11.0, "MANUAL", 0, -50) == (True, False)


def test_manual_critical_with_both_sticks_neutral_forces_idle_and_stops_rumble():
    assert vs.decide_voltage_safety_action(11.0, "MANUAL", 0, 0) == (False, True)


def test_auto_critical_forces_idle_immediately_with_no_rumble():
    # No joystick to warn through in AUTO -- confirmed with the user.
    assert vs.decide_voltage_safety_action(11.0, "AUTO", 0, 0) == (False, True)


def test_idle_mode_critical_does_nothing_more():
    # Already stopped -- nothing left to warn about or force.
    assert vs.decide_voltage_safety_action(11.0, "IDLE", 0, 0) == (False, False)


def test_exactly_at_the_threshold_counts_as_critical():
    assert vs.decide_voltage_safety_action(CRITICAL, "MANUAL", 100, 0) == (True, False)


# --- update_auto_lockout() --------------------------------------------------

def test_lockout_engages_on_a_critical_reading():
    assert vs.update_auto_lockout(False, CRITICAL) is True
    assert vs.update_auto_lockout(False, CRITICAL - 0.5) is True


def test_lockout_clears_once_recovered_past_the_margin():
    assert vs.update_auto_lockout(True, RECOVERY) is False
    assert vs.update_auto_lockout(True, RECOVERY + 0.5) is False


def test_lockout_holds_steady_in_the_hysteresis_band():
    # Between the two thresholds -- neither critical nor recovered --
    # the flag is left exactly as it was, whichever way it was set.
    midpoint = (CRITICAL + RECOVERY) / 2
    assert vs.update_auto_lockout(True, midpoint) is True
    assert vs.update_auto_lockout(False, midpoint) is False


def test_lockout_unaffected_by_a_missing_reading():
    assert vs.update_auto_lockout(True, None) is True
    assert vs.update_auto_lockout(False, None) is False


# --- VoltageSafety: sampling / deciding / rumble / force-idle wiring -------

class _FakeGamepad:
    """Minimal stand-in for link.gamepad_handler.GamepadReader --
    VoltageSafety only ever calls .start_rumble()/.stop_rumble()."""

    def __init__(self):
        self.calls = []

    def start_rumble(self, strong=True):
        self.calls.append(("start", strong))

    def stop_rumble(self):
        self.calls.append(("stop",))


def _make_policy(gamepad=None):
    state = RobotState()
    policy = vs.VoltageSafety(state, gamepad=gamepad if gamepad is not None else _FakeGamepad())
    return state, policy


def test_check_once_rumbles_while_manual_and_critical_with_a_push():
    state, policy = _make_policy()
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=11.0, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=11.0, load_current=5.0, load_power=55.0,
        battery_soc=20.0, battery_temp=25.0, controller_temp=25.0,
    )
    state.set_mode("MANUAL")
    state.drive(150, 150)

    policy._check_once()

    assert policy.gamepad.calls == [("start", True)]
    assert state.mode == "MANUAL"  # not forced to idle -- still pushing


def test_check_once_stops_rumble_and_forces_idle_once_neutral():
    state, policy = _make_policy()
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=11.0, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=11.0, load_current=5.0, load_power=55.0,
        battery_soc=20.0, battery_temp=25.0, controller_temp=25.0,
    )
    state.set_mode("MANUAL")
    state.drive(150, 150)
    policy._check_once()  # starts rumbling
    policy.gamepad.calls.clear()

    state.drive(0, 0)  # joystick released to neutral
    policy._check_once()

    assert policy.gamepad.calls == [("stop",)]
    assert state.mode == "IDLE"


def test_check_once_forces_idle_in_auto_with_no_rumble():
    state, policy = _make_policy()
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=11.0, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=11.0, load_current=5.0, load_power=55.0,
        battery_soc=20.0, battery_temp=25.0, controller_temp=25.0,
    )
    state.set_mode("AUTO")

    policy._check_once()

    assert policy.gamepad.calls == []
    assert state.mode == "IDLE"


def test_check_once_locks_and_later_unlocks_the_auto_lockout_on_state():
    state, policy = _make_policy()
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=11.0, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=11.0, load_current=5.0, load_power=55.0,
        battery_soc=20.0, battery_temp=25.0, controller_temp=25.0,
    )
    policy._check_once()
    with pytest.raises(CommandError) as exc_info:
        state.set_mode("AUTO")
    assert exc_info.value.code == "26"

    # Recharged past the recovery margin -- lockout clears.
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=12.5, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=12.5, load_current=1.0, load_power=12.5,
        battery_soc=60.0, battery_temp=25.0, controller_temp=25.0,
    )
    policy._check_once()
    state.set_mode("AUTO")  # no longer raises
    assert state.mode == "AUTO"


def test_check_once_ignores_an_unavailable_reading():
    # power_available stays None/not set -- same "uncertainty -> do not
    # act" posture is tested at the pure-function level above; this
    # just confirms the wrapper wires it through without crashing.
    state, policy = _make_policy()
    policy._check_once()
    assert policy.gamepad.calls == []
    assert state.mode == "IDLE"


def test_check_once_works_with_no_gamepad_connected():
    state, policy = _make_policy(gamepad=None)
    policy.gamepad = None
    state.update_power_reading(
        available=True, pv_voltage=13.0, pv_current=1.0, pv_power=13.0,
        battery_voltage=11.0, battery_charging_current=0.0, battery_charging_power=0.0,
        load_voltage=11.0, load_current=5.0, load_power=55.0,
        battery_soc=20.0, battery_temp=25.0, controller_temp=25.0,
    )
    state.set_mode("MANUAL")
    state.drive(150, 150)
    policy._check_once()  # must not raise with gamepad=None
    assert state.mode == "MANUAL"


def test_check_once_survives_a_broken_state():
    class _BrokenState:
        def power_status(self):
            raise RuntimeError("tracer is on fire")

        def status(self):
            return {"mode": "IDLE", "left_pwm": 0, "right_pwm": 0}

    policy = vs.VoltageSafety(_BrokenState(), gamepad=_FakeGamepad())
    policy._check_once()  # must not raise
