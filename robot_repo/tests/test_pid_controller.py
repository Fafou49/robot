"""
Unit tests for PIDController (pid/pid_controller.py).

This is the only module in the project with no hardware dependency
(no GPIO, no serial port), so it is the easiest place to start adding
automated tests. Run with:

    pip install -r requirements.txt
    pytest
"""
import time

from pid.pid_controller import PIDController


def test_zero_error_gives_zero_output():
    pid = PIDController(setpoint=0.0, kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(measured_value=0.0)
    assert output == 0.0


def test_proportional_term_scales_with_error():
    pid = PIDController(setpoint=10.0, kp=2.0, ki=0.0, kd=0.0)
    # error = 10 - 0 = 10, kd/ki are 0 so output ~= kp * error
    output = pid.update(measured_value=0.0)
    assert output == 2.0 * 10.0


def test_set_setpoint_updates_target():
    pid = PIDController(setpoint=0.0, kp=1.0, ki=0.0, kd=0.0)
    pid.set_setpoint(5.0)
    assert pid.setpoint == 5.0


def test_integral_accumulates_over_successive_calls():
    pid = PIDController(setpoint=1.0, kp=0.0, ki=1.0, kd=0.0)
    pid.update(measured_value=0.0)
    time.sleep(0.01)
    second_output = pid.update(measured_value=0.0)
    # With constant positive error and ki>0, the integral term keeps growing.
    assert second_output > 0.0


# --- reset() (added 2026-09-07 for link/autopilot.py) -----------------------

def test_reset_clears_integral_and_previous_error():
    pid = PIDController(setpoint=1.0, kp=0.0, ki=1.0, kd=1.0)
    pid.update(measured_value=0.0)
    time.sleep(0.01)
    pid.update(measured_value=0.0)  # integral and previous_error now nonzero/set
    assert pid.integral != 0
    assert pid.previous_error is not None

    pid.reset()
    assert pid.integral == 0
    assert pid.previous_error is None


def test_reset_avoids_a_derivative_kick_on_the_next_update():
    # Same reasoning as the existing "no derivative kick on first call"
    # behavior (previous_error starts as None, not 0) -- reset() should
    # put the controller back in that exact state, not just zero the
    # integral.
    pid = PIDController(setpoint=0.0, kp=0.0, ki=0.0, kd=10.0)
    pid.update(measured_value=100.0)  # large first error, no kick (previous_error was None)
    pid.reset()
    output = pid.update(measured_value=-100.0)  # large jump right after reset
    assert output == 0.0  # derivative term must not fire off a stale previous_error
