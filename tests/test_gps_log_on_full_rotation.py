"""Tests for motor_control.gps_log_on_full_rotation.

Only is_full_rotation() is tested here: it's pure logic with no hardware
involved, so it can actually run in this sandbox. The rest of the module
(main(), the shared ConditionGPSLogger from gps_condition_logger.py) needs
evdev/pygame/gpiod (Remote's dependencies) and pyserial, none of which are
installable here (no PyPI access) -- same honesty note as
tests/test_gps_reader.py and tests/test_gps_log_on_full_throttle.py. Run
this for real on the Pi once those are installed, with a gamepad and GPS
receiver connected, before relying on the full-rotation logging behavior.
"""
from motor_control.gps_log_on_full_rotation import FULL_SPEED, is_full_rotation


def test_is_full_rotation_true_left_forward_right_backward():
    assert is_full_rotation(255, -255) is True


def test_is_full_rotation_true_left_backward_right_forward():
    assert is_full_rotation(-255, 255) is True


def test_is_full_rotation_false_same_direction_full_throttle():
    # Both motors forward at full speed is straight-line driving (see
    # is_full_throttle in gps_log_on_full_throttle.py), not a rotation.
    assert is_full_rotation(255, 255) is False
    assert is_full_rotation(-255, -255) is False


def test_is_full_rotation_false_when_not_at_full_speed():
    assert is_full_rotation(200, -200) is False
    assert is_full_rotation(255, -200) is False
    assert is_full_rotation(200, -255) is False


def test_is_full_rotation_false_when_stopped_or_partial():
    assert is_full_rotation(0, 0) is False
    assert is_full_rotation(255, 0) is False
    assert is_full_rotation(0, -255) is False


def test_is_full_rotation_uses_the_255_constant():
    assert FULL_SPEED == 255
    assert is_full_rotation(FULL_SPEED, -FULL_SPEED) is True


def test_is_full_rotation_custom_speed():
    # speed is overridable, mainly so tests don't have to hardcode 255
    # twice -- the real script always calls it with the default.
    assert is_full_rotation(200, -200, speed=200) is True
    assert is_full_rotation(199, -200, speed=200) is False
