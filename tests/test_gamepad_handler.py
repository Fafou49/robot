"""Tests for link.gamepad_handler.

_normalize_axis() and _pwm_from_axis() are pure logic (no hardware) and
always run. GamepadReader needs evdev, which isn't installable in this
sandbox (no PyPI access) -- same honesty note as the rest of this
project's hardware-facing code: evdev is stubbed out below just enough to
exercise GamepadReader's control flow (device discovery, degrade-
gracefully paths, and event dispatch against a *fake* device that yields
a scripted sequence of events) -- this does NOT mean it has been run
against a real controller.
"""
import sys
import types

import pytest

# Stub evdev with a minimal but realistic ecodes namespace -- real evdev
# event codes for a standard Xbox controller (xpad driver), used
# symbolically (ecodes.BTN_A etc.) everywhere in gamepad_handler.py and
# here, never hardcoded, so the exact numeric values only need to be
# internally consistent.
_ecodes_stub = types.SimpleNamespace(
    EV_KEY=1, EV_ABS=3,
    ABS_Y=1, ABS_RY=4,
    BTN_A=304, BTN_B=305, BTN_X=307, BTN_Y=308, BTN_START=315,
)
_evdev_stub = types.ModuleType("evdev")
_evdev_stub.ecodes = _ecodes_stub
_evdev_stub.list_devices = lambda: []
_evdev_stub.InputDevice = lambda path: None
sys.modules["evdev"] = _evdev_stub
sys.modules["evdev.ecodes"] = _ecodes_stub

from link.gamepad_handler import (  # noqa: E402
    GamepadReader, _normalize_axis, _pwm_from_axis, robot_state_button_handler,
    robot_state_drive_handler,
)

ecodes = _ecodes_stub


# --- _normalize_axis / _pwm_from_axis (pure logic) --------------------------

def test_normalize_axis_center_is_zero():
    # A typical evdev signed 16-bit axis range.
    assert _normalize_axis(0, -32768, 32767) == pytest.approx(0.0, abs=1e-3)


def test_normalize_axis_extremes_are_clamped_to_unit_range():
    assert _normalize_axis(-32768, -32768, 32767) == pytest.approx(-1.0, abs=1e-3)
    assert _normalize_axis(32767, -32768, 32767) == pytest.approx(1.0, abs=1e-3)


def test_normalize_axis_handles_unsigned_range_too():
    # Some drivers report axes as 0..255 instead of a signed range.
    assert _normalize_axis(0, 0, 255) == pytest.approx(-1.0, abs=1e-3)
    assert _normalize_axis(255, 0, 255) == pytest.approx(1.0, abs=1e-3)
    assert _normalize_axis(127.5, 0, 255) == pytest.approx(0.0, abs=1e-3)


def test_normalize_axis_degenerate_range_returns_zero():
    # min == max: division by zero avoided, just report centered.
    assert _normalize_axis(5, 5, 5) == 0.0


def test_pwm_from_axis_deadzone():
    assert _pwm_from_axis(0.0) == 0
    assert _pwm_from_axis(0.05) == 0    # inside the default deadzone
    assert _pwm_from_axis(-0.05) == 0


def test_pwm_from_axis_sign_is_inverted():
    # Stick pushed fully "up" (evdev/SDL convention: negative value) must
    # produce a POSITIVE (forward) pwm.
    assert _pwm_from_axis(-1.0) == 255
    assert _pwm_from_axis(1.0) == -255


def test_pwm_from_axis_scales_linearly():
    assert _pwm_from_axis(-0.5) == 128 or _pwm_from_axis(-0.5) == 127  # round() boundary


# --- GamepadReader: device discovery / graceful degradation ----------------

class _FakeDevice:
    def __init__(self, name, path, capabilities, events=()):
        self.name = name
        self.path = path
        self._capabilities = capabilities
        self._events = list(events)

    def capabilities(self, absinfo=False):
        if absinfo:
            return self._capabilities.get("absinfo", {})
        return {k: v for k, v in self._capabilities.items() if k != "absinfo"}

    def read_loop(self):
        return iter(self._events)


def test_find_device_skips_non_gamepad_devices(monkeypatch):
    keyboard = _FakeDevice("Some Keyboard", "/dev/input/event3", {ecodes.EV_KEY: [30, 31]})
    gamepad = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RY], ecodes.EV_KEY: [ecodes.BTN_A, ecodes.BTN_B]},
    )
    monkeypatch.setattr(sys.modules["evdev"], "list_devices", lambda: [keyboard.path, gamepad.path])
    monkeypatch.setattr(
        sys.modules["evdev"], "InputDevice",
        lambda path: {keyboard.path: keyboard, gamepad.path: gamepad}[path],
    )

    reader = GamepadReader(on_drive=lambda l, r: None)
    found = reader._find_device()
    assert found is gamepad


def test_find_device_returns_none_when_nothing_matches(monkeypatch):
    keyboard = _FakeDevice("Some Keyboard", "/dev/input/event3", {ecodes.EV_KEY: [30, 31]})
    monkeypatch.setattr(sys.modules["evdev"], "list_devices", lambda: [keyboard.path])
    monkeypatch.setattr(sys.modules["evdev"], "InputDevice", lambda path: keyboard)

    reader = GamepadReader(on_drive=lambda l, r: None)
    assert reader._find_device() is None


def test_run_blocking_degrades_without_evdev(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "_EVDEV_AVAILABLE", False)
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader.run_blocking()  # must return immediately, not raise or hang


# --- GamepadReader: event dispatch ------------------------------------------

class _Event:
    def __init__(self, type_, code, value):
        self.type = type_
        self.code = code
        self.value = value


def test_read_events_dispatches_axis_and_button_events(monkeypatch):
    drives = []
    buttons = []

    AbsInfo = types.SimpleNamespace  # just needs .min/.max attributes
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {
            ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RY],
            ecodes.EV_KEY: [ecodes.BTN_A],
            "absinfo": {
                ecodes.ABS_Y: [(ecodes.ABS_Y, AbsInfo(min=-32768, max=32767))][0][1],
                ecodes.ABS_RY: AbsInfo(min=-32768, max=32767),
            },
        },
        events=[
            _Event(ecodes.EV_ABS, ecodes.ABS_Y, -32768),   # full forward on left stick
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 1),         # press
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 2),         # hold-repeat, must be ignored
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 0),         # release
        ],
    )
    # capabilities(absinfo=True) must return {EV_ABS: [(code, info), ...]}
    device._capabilities["absinfo"] = {ecodes.EV_ABS: [
        (ecodes.ABS_Y, AbsInfo(min=-32768, max=32767)),
        (ecodes.ABS_RY, AbsInfo(min=-32768, max=32767)),
    ]}

    def _capabilities(absinfo=False):
        if absinfo:
            return device._capabilities["absinfo"]
        return {k: v for k, v in device._capabilities.items() if k != "absinfo"}
    device.capabilities = _capabilities

    reader = GamepadReader(on_drive=lambda l, r: drives.append((l, r)),
                            on_button=lambda code, pressed: buttons.append((code, pressed)))
    reader._running = True
    reader._read_events(device)

    assert drives == [(255, 0)]  # full-forward left stick -> +255 left, right untouched (0)
    assert buttons == [(ecodes.BTN_A, True), (ecodes.BTN_A, False)]


# --- robot_state_button_handler ---------------------------------------------

class _FakeState:
    def __init__(self):
        self.modes = []
        self.stop_count = 0
        self.drives = []

    def set_mode(self, mode):
        self.modes.append(mode)

    def stop(self):
        self.stop_count += 1

    def drive(self, left_pwm, right_pwm):
        self.drives.append((left_pwm, right_pwm))


def test_button_handler_maps_buttons_to_state_calls():
    # BTN_A arms AUTO (the "go to the next point" button, see
    # link/robot_state.py); BTN_B and BTN_START are both a full stop
    # (state.stop(), same as STP) -- redundant on purpose, see
    # robot_state_button_handler()'s docstring.
    state = _FakeState()
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_A, True)
    handler(ecodes.BTN_B, True)
    handler(ecodes.BTN_START, True)
    assert state.modes == ["AUTO"]
    assert state.stop_count == 2


def test_button_handler_ignores_release_events():
    state = _FakeState()
    handler = robot_state_button_handler(state)
    handler(ecodes.BTN_A, False)
    assert state.modes == []
    assert state.stop_count == 0


# --- robot_state_drive_handler -----------------------------------------------

def test_drive_handler_forces_manual_on_genuine_stick_push():
    state = _FakeState()
    handler = robot_state_drive_handler(state)

    handler(200, 0)  # left stick pushed forward -- a genuine, nonzero push
    assert state.modes == ["MANUAL"]
    assert state.drives == [(200, 0)]


def test_drive_handler_does_not_force_manual_on_idle_zero_output():
    # An idle/centered stick still fires on_drive on every ABS event
    # (analog noise) -- _pwm_from_axis maps those to (0, 0), and that
    # must NOT cancel an active AUTO drive (see this function's
    # docstring for why).
    state = _FakeState()
    handler = robot_state_drive_handler(state)

    handler(0, 0)
    assert state.modes == []
    assert state.drives == [(0, 0)]


def test_drive_handler_forces_manual_when_only_one_side_is_nonzero():
    state = _FakeState()
    handler = robot_state_drive_handler(state)

    handler(0, -128)  # e.g. only the right stick pushed
    assert state.modes == ["MANUAL"]
    assert state.drives == [(0, -128)]
