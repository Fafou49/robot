"""Tests for motor_control.dump_gamepad_axes._axis_name() -- the one
piece of pure logic in that diagnostic script (no hardware needed): the
reverse lookup from a raw evdev axis code to a human name. The rest of
the script (finding a controller, dumping live events) needs a real
evdev/controller and isn't exercised here -- same honesty note as the
rest of this project's hardware-facing code.
"""
import types

from motor_control.dump_gamepad_axes import _axis_name

# A small fake ecodes namespace, deliberately using DIFFERENT numbers
# than the real Linux ones so these tests can't accidentally pass just
# because they happen to match reality -- only the name->code mapping
# matters to _axis_name().
_FAKE_ECODES = types.SimpleNamespace(
    ABS_X=100, ABS_Y=101, ABS_Z=102, ABS_RX=103, ABS_RY=104, ABS_RZ=105,
    ABS_HAT0X=106, ABS_HAT0Y=107, ABS_THROTTLE=108, ABS_BRAKE=109, ABS_GAS=110,
)


def test_axis_name_resolves_known_codes():
    assert _axis_name(101, _FAKE_ECODES) == "ABS_Y"
    assert _axis_name(104, _FAKE_ECODES) == "ABS_RY"
    assert _axis_name(105, _FAKE_ECODES) == "ABS_RZ"


def test_axis_name_returns_none_for_an_unknown_code():
    assert _axis_name(9999, _FAKE_ECODES) is None


def test_axis_name_handles_a_missing_attribute_on_the_ecodes_module():
    # A stripped-down ecodes namespace, missing some names entirely --
    # must not raise, just skip those in the search.
    sparse = types.SimpleNamespace(ABS_Y=101, ABS_RY=104)
    assert _axis_name(101, sparse) == "ABS_Y"
    assert _axis_name(999, sparse) is None
