"""Tests for motor_control.check_gamepad._verdict() -- the one piece of
pure logic in that diagnostic script (no hardware, no evdev needed): the
plain-language verdict derived from the min/max right-stick PWM values
observed during one run. The rest of the script (finding a controller,
printing live values) needs a real evdev/controller and isn't exercised
here -- same honesty note as the rest of this project's hardware-facing
code.
"""
from motor_control.check_gamepad import _verdict


def test_verdict_no_movement_at_all():
    assert "No movement detected" in _verdict(0, 0)


def test_verdict_reached_both_extremes():
    assert "looks functional" in _verdict(-255, 255)


def test_verdict_reached_near_full_scale_counts_as_functional():
    # A worn stick or a slightly generous deadzone might not hit the
    # exact -255/+255 endpoints -- "near enough" still counts.
    assert "looks functional" in _verdict(-250, 248)


def test_verdict_moved_but_under_travels():
    assert "under-traveling" in _verdict(-40, 35)


def test_verdict_reached_only_one_side():
    # Full range on one side, barely anything on the other -- still a
    # problem worth flagging, not a pass.
    assert "under-traveling" in _verdict(-255, 10)


def test_verdict_custom_near_full_scale_threshold():
    assert "under-traveling" in _verdict(-150, 150, near_full_scale=200)
    assert "looks functional" in _verdict(-150, 150, near_full_scale=100)
