"""Tests for motor_control.check_rumble. Only _support_message() is pure
logic (no hardware) -- main() drives evdev/GamepadReader calls directly for
the live vibration test and is not exercised here, same "pure logic
tested, hardware-facing main() documented as untested" pattern as
check_gamepad.py and dump_gamepad_axes.py in this project."""
from motor_control.check_rumble import _support_message


def test_support_message_when_ff_rumble_is_supported():
    message = _support_message(True)
    assert "supported" in message
    assert "NOT" not in message


def test_support_message_when_ff_rumble_is_not_supported():
    message = _support_message(False)
    assert "NOT supported" in message
    # Must point at the known real-world Bluetooth-vs-USB caveat, not just
    # say "no" -- that's the whole point of this script over a bare bool.
    assert "USB" in message
