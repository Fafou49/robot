"""Tests for motor_control.dump_gamepad_buttons._button_name() -- the one
piece of pure logic in that diagnostic script (no hardware needed): the
reverse lookup from a raw evdev button code to a human name. The rest of
the script (finding a controller, dumping live events) needs a real
evdev/controller and isn't exercised here -- same honesty note as the
rest of this project's hardware-facing code.
"""
import types

from motor_control.dump_gamepad_buttons import _button_name

# A small fake ecodes namespace, deliberately using DIFFERENT numbers
# than the real Linux ones so these tests can't accidentally pass just
# because they happen to match reality -- only the name->code mapping
# matters to _button_name(). BTN_A/BTN_SOUTH share a code on purpose,
# mirroring how the real evdev module aliases them.
_FAKE_ECODES = types.SimpleNamespace(
    BTN_A=200, BTN_B=201, BTN_X=202, BTN_Y=203,
    BTN_SOUTH=200, BTN_EAST=201, BTN_NORTH=203, BTN_WEST=202,
    BTN_START=210, BTN_SELECT=211, BTN_MODE=212,
)


def test_button_name_resolves_known_codes():
    assert _button_name(203, _FAKE_ECODES) == "BTN_Y"
    assert _button_name(210, _FAKE_ECODES) == "BTN_START"


def test_button_name_prefers_the_standard_name_over_its_alias():
    # BTN_A and BTN_SOUTH share code 200 in the fake namespace above (as
    # they do in real evdev) -- KNOWN_BUTTON_NAMES lists BTN_A first, so
    # that's the name that should come back, not its generic alias.
    assert _button_name(200, _FAKE_ECODES) == "BTN_A"


def test_button_name_returns_none_for_an_unknown_code():
    assert _button_name(9999, _FAKE_ECODES) is None


def test_button_name_handles_a_missing_attribute_on_the_ecodes_module():
    # A stripped-down ecodes namespace, missing some names entirely --
    # must not raise, just skip those in the search.
    sparse = types.SimpleNamespace(BTN_Y=203, BTN_START=210)
    assert _button_name(203, sparse) == "BTN_Y"
    assert _button_name(999, sparse) is None
