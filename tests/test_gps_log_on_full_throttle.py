"""Tests for motor_control.gps_log_on_full_throttle.

Only is_full_throttle() is tested here: it's pure logic with no hardware
involved, so it can actually run in this sandbox. The rest of the module
(FullThrottleGPSLogger, main()) needs evdev/pygame/gpiod (Remote's
dependencies) and pyserial, none of which are installable here (no PyPI
access) -- same honesty note as tests/test_gps_reader.py. Run this whole
module's manual smoke-check (see bottom of file) on the Pi once those are
installed, with a gamepad and GPS receiver connected, before relying on
the full-throttle logging behavior.
"""
from motor_control.gps_log_on_full_throttle import FULL_THROTTLE, is_full_throttle


def test_is_full_throttle_true_when_both_at_max():
    assert is_full_throttle(255, 255) is True


def test_is_full_throttle_false_when_only_one_at_max():
    assert is_full_throttle(255, 200) is False
    assert is_full_throttle(100, 255) is False


def test_is_full_throttle_false_when_neither_at_max():
    assert is_full_throttle(0, 0) is False
    assert is_full_throttle(-255, -255) is False  # full reverse doesn't count, only full forward


def test_is_full_throttle_uses_the_255_constant():
    assert FULL_THROTTLE == 255
    assert is_full_throttle(FULL_THROTTLE, FULL_THROTTLE) is True


def test_is_full_throttle_custom_threshold():
    # threshold is overridable, mainly so tests don't have to hardcode 255
    # twice -- the real script always calls it with the default.
    assert is_full_throttle(200, 200, threshold=200) is True
    assert is_full_throttle(199, 200, threshold=200) is False
