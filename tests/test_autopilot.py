"""Tests for link.autopilot -- pure math (haversine_distance_m,
bearing_deg, heading_error_for_pid) plus Autopilot.compute()/reset(),
none of which touch hardware, GPS, or GPIO."""
import time

import pytest

from link.autopilot import (
    Autopilot,
    bearing_deg,
    haversine_distance_m,
    heading_error_for_pid,
)


# --- haversine_distance_m ----------------------------------------------------

def test_haversine_distance_same_point_is_zero():
    assert haversine_distance_m(47.391533, -0.739, 47.391533, -0.739) == pytest.approx(0.0, abs=1e-6)


def test_haversine_distance_known_short_hop():
    # Two points ~95m apart (same pair used in tests/test_link_server.py's
    # route-advance test).
    d = haversine_distance_m(47.391533, -0.739, 47.392200, -0.7382)
    assert d == pytest.approx(95, abs=15)


# --- bearing_deg --------------------------------------------------------------

def test_bearing_due_north():
    assert bearing_deg(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_bearing_due_east():
    assert bearing_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=1e-6)


def test_bearing_due_south():
    assert bearing_deg(0.0, 0.0, -1.0, 0.0) == pytest.approx(180.0, abs=1e-6)


def test_bearing_due_west():
    assert bearing_deg(0.0, 0.0, 0.0, -1.0) == pytest.approx(270.0, abs=1e-6)


def test_bearing_is_always_in_0_360_range():
    assert 0.0 <= bearing_deg(10.0, 10.0, 5.0, -5.0) < 360.0


# --- heading_error_for_pid ----------------------------------------------------
# See link/autopilot.py's docstring for the sign derivation: target
# clockwise of current heading (need to turn right) -> NEGATIVE value, so
# that PIDController's setpoint(0)-measured error comes out positive,
# which the mixing formula (left = speed+angular, right = speed-angular)
# turns into "spin left wheel faster" == turn right.

def test_heading_error_target_clockwise_is_negative():
    # current=0 (north), target=90 (east) -- target is 90 deg clockwise.
    assert heading_error_for_pid(0.0, 90.0) == pytest.approx(-90.0)


def test_heading_error_target_counterclockwise_is_positive():
    # current=90 (east), target=0 (north) -- target is 90 deg counter-clockwise.
    assert heading_error_for_pid(90.0, 0.0) == pytest.approx(90.0)


def test_heading_error_wraps_around_0_360():
    # current=350, target=10 -- target is 20 deg clockwise (wrapping past 360).
    assert heading_error_for_pid(350.0, 10.0) == pytest.approx(-20.0)


def test_heading_error_already_aligned_is_zero():
    assert heading_error_for_pid(45.0, 45.0) == pytest.approx(0.0)


# --- Autopilot.compute() ------------------------------------------------------

def test_compute_far_and_misaligned_drives_forward_and_turns_right():
    # Far away (100m) and target 90 deg clockwise of current heading --
    # both wheels should get a real command, and the robot should be
    # steered right (left wheel faster than right).
    ap = Autopilot()
    left, right = ap.compute(distance_m=100.0, heading_error_deg=heading_error_for_pid(0.0, 90.0))
    assert left > right  # left faster than right -> turns right, matching the target


def test_compute_far_and_aligned_drives_straight():
    ap = Autopilot()
    left, right = ap.compute(distance_m=50.0, heading_error_deg=0.0)
    assert left == right
    assert left > 0  # far away and pointed the right way -> drive forward


def test_compute_at_target_with_zero_heading_error_is_near_zero():
    ap = Autopilot()
    left, right = ap.compute(distance_m=0.0, heading_error_deg=0.0)
    assert left == 0
    assert right == 0


def test_compute_output_is_clamped_to_pwm_range():
    ap = Autopilot()
    # Absurdly large distance/heading error -- output must never exceed
    # the +/-255 range MotorDriver/RobotState expect.
    left, right = ap.compute(distance_m=1_000_000.0, heading_error_deg=-179.0)
    assert -255 <= left <= 255
    assert -255 <= right <= 255


def test_set_gains_updates_the_right_loop():
    ap = Autopilot()
    ap.set_gains("D", 2.0, 0.1, 0.3)
    ap.set_gains("A", 0.7, 0.0, 0.9)
    assert (ap._pid_distance.kp, ap._pid_distance.ki, ap._pid_distance.kd) == (2.0, 0.1, 0.3)
    assert (ap._pid_angle.kp, ap._pid_angle.ki, ap._pid_angle.kd) == (0.7, 0.0, 0.9)


def test_set_gains_rejects_unknown_loop():
    ap = Autopilot()
    with pytest.raises(ValueError):
        ap.set_gains("Z", 1.0, 0.0, 0.0)


def test_reset_clears_both_loops():
    ap = Autopilot()
    ap.compute(distance_m=50.0, heading_error_deg=30.0)
    time.sleep(0.01)
    ap.compute(distance_m=45.0, heading_error_deg=25.0)
    assert ap._pid_distance.previous_error is not None

    ap.reset()
    assert ap._pid_distance.previous_error is None
    assert ap._pid_distance.integral == 0
    assert ap._pid_angle.previous_error is None
    assert ap._pid_angle.integral == 0
