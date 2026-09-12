"""Tests for link.gamepad_handler.

_normalize_axis() and _pwm_from_axis() are pure logic (no hardware) and
always run. GamepadReader needs evdev, which isn't installable in this
sandbox (no PyPI access) -- same honesty note as the rest of this
project's hardware-facing code: evdev is stubbed out (see tests/
conftest.py, loaded by pytest before this or any other test module here)
just enough to exercise GamepadReader's control flow (device discovery,
degrade-gracefully paths, and event dispatch against a *fake* device that
yields a scripted sequence of events) -- this does NOT mean it has been
run against a real controller. The rumble (force-feedback) tests further
down are the same story: the fake device's upload_effect()/write()/
erase_effect() just record what was called and don't touch any real FF
hardware state, so those tests exercise start_rumble()/stop_rumble()'s
own control flow (pulse retriggering, idempotency, reconnect handling),
not whether a real Xbox controller actually buzzes.
"""
import sys
import time
import types

import pytest

from tests.conftest import ecodes  # noqa: E402 -- see conftest.py's own docstring
from link.gamepad_handler import (  # noqa: E402
    GamepadReader, _normalize_axis, _pwm_from_axis, _supports_ff_rumble,
    robot_state_button_handler, robot_state_drive_handler,
)


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
        # Force-feedback call log, for the rumble tests further down:
        # each entry is ("upload", effect), ("write", effect_id, value),
        # or ("erase", effect_id) -- recorded, not actually acted on,
        # same "fake just enough to exercise control flow" spirit as the
        # rest of this stub.
        self.ff_calls = []
        self._next_effect_id = 1

    def capabilities(self, absinfo=False):
        if absinfo:
            return self._capabilities.get("absinfo", {})
        return {k: v for k, v in self._capabilities.items() if k != "absinfo"}

    def read_loop(self):
        return iter(self._events)

    def upload_effect(self, effect):
        effect_id = self._next_effect_id
        self._next_effect_id += 1
        self.ff_calls.append(("upload", effect))
        return effect_id

    def erase_effect(self, effect_id):
        self.ff_calls.append(("erase", effect_id))

    def write(self, event_type, code, value):
        assert event_type == ecodes.EV_FF
        self.ff_calls.append(("write", code, value))


def test_find_device_skips_non_gamepad_devices(monkeypatch):
    keyboard = _FakeDevice("Some Keyboard", "/dev/input/event3", {ecodes.EV_KEY: [30, 31]})
    gamepad = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RZ], ecodes.EV_KEY: [ecodes.BTN_A, ecodes.BTN_B]},
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


def test_read_events_dispatches_left_and_right_stick_and_button_events(monkeypatch):
    # Right stick uses ABS_RZ here, NOT the "standard" xpad mapping's
    # ABS_RY -- this project's actual controller/receiver was confirmed
    # (2026-09-11, motor_control/dump_gamepad_axes.py) to report it that
    # way; see DEFAULT_RIGHT_Y_CODE's comment in link/gamepad_handler.py
    # (an earlier fix briefly used ABS_Z -- the left trigger's code in
    # the standard mapping -- based on a misread, corrected to ABS_RZ).
    # This test exists specifically to catch a regression back to
    # ABS_RY (or ABS_Z): with either of those, no event in this scripted
    # sequence would ever match self.right_y_code, and `drives` would
    # never show a nonzero right value no matter how the right stick
    # moved.
    drives = []
    buttons = []

    AbsInfo = types.SimpleNamespace  # just needs .min/.max attributes
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {
            ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RZ],
            ecodes.EV_KEY: [ecodes.BTN_A],
        },
        events=[
            _Event(ecodes.EV_ABS, ecodes.ABS_Y, -32768),   # full forward on left stick
            _Event(ecodes.EV_ABS, ecodes.ABS_RZ, 32767),   # full "up" on right stick
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 1),         # press
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 2),         # hold-repeat, must be ignored
            _Event(ecodes.EV_KEY, ecodes.BTN_A, 0),         # release
        ],
    )
    # capabilities(absinfo=True) must return {EV_ABS: [(code, info), ...]}
    device._capabilities["absinfo"] = {ecodes.EV_ABS: [
        (ecodes.ABS_Y, AbsInfo(min=-32768, max=32767)),
        (ecodes.ABS_RZ, AbsInfo(min=-32768, max=32767)),
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

    assert drives == [
        (255, 0),     # left stick moves first, right stick still centered
        (255, -255),  # then the right stick moves too (left value retained)
    ]
    assert buttons == [(ecodes.BTN_A, True), (ecodes.BTN_A, False)]


# --- robot_state_button_handler ---------------------------------------------

class _FakeState:
    def __init__(self, nav_target=None, mode="IDLE"):
        self.modes = []
        self.stop_count = 0
        self.drives = []
        # has_nav_target() reads this -- set it in a test to simulate a
        # NAV point or GPS route already having been sent.
        self.nav_target = nav_target
        # is_manual() reads this -- set it in a test to simulate the
        # robot already being in a given mode (e.g. "MANUAL" or "AUTO").
        # set_mode() below also updates it, same as the real RobotState,
        # so a test that arms AUTO via set_mode and then checks is_manual
        # sees a consistent picture.
        self.mode = mode

    def set_mode(self, mode):
        self.modes.append(mode)
        self.mode = mode

    def stop(self):
        self.stop_count += 1

    def drive(self, left_pwm, right_pwm):
        self.drives.append((left_pwm, right_pwm))

    def has_nav_target(self):
        return self.nav_target is not None

    def is_manual(self):
        return self.mode == "MANUAL"


def test_button_handler_maps_buttons_to_state_calls():
    # BTN_Y re-arms AUTO (2026-09-12: moved from BTN_A, see
    # robot_state_button_handler()'s docstring) when a target is already
    # set; BTN_B is a full stop (state.stop(), same as STP); BTN_START
    # also stops, then additionally calls on_shutdown() -- link/server.py
    # wires this to actually power off the Raspberry Pi.
    state = _FakeState(nav_target=("4723.492", "N", "00044.340", "W"))
    shutdown_calls = []
    handler = robot_state_button_handler(state, on_shutdown=lambda: shutdown_calls.append(True))

    handler(ecodes.BTN_Y, True)
    handler(ecodes.BTN_B, True)
    handler(ecodes.BTN_START, True)
    assert state.modes == ["AUTO"]
    assert state.stop_count == 2
    assert shutdown_calls == [True]


def test_button_handler_refuses_to_arm_auto_with_no_target():
    # Pressing Y with neither a NAV point nor a GPS route sent yet must
    # not switch into AUTO -- see has_nav_target()'s own docstring for why
    # (nothing to drive toward -- it would just sit there every GPS fix).
    state = _FakeState(nav_target=None)
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_Y, True)
    assert state.modes == []


def test_button_handler_shutdown_is_optional():
    # link/server.py always passes on_shutdown, but nothing requires
    # every caller to: BTN_START must still stop the robot even with none
    # given, and must not raise trying to call it.
    state = _FakeState()
    handler = robot_state_button_handler(state)  # no on_shutdown

    handler(ecodes.BTN_START, True)
    assert state.stop_count == 1


def test_button_handler_ignores_release_events():
    state = _FakeState(nav_target=("4723.492", "N", "00044.340", "W"))
    handler = robot_state_button_handler(state)
    handler(ecodes.BTN_Y, False)
    assert state.modes == []
    assert state.stop_count == 0


def test_button_handler_button_codes_are_configurable():
    # 2026-09-12: if this project's actual controller/receiver reports Y
    # or Start under a different evdev code than assumed (the same class
    # of quirk already hit for the right stick's axis -- see
    # DEFAULT_RIGHT_Y_CODE), arm_auto_btn/stop_btn/shutdown_btn (or the
    # matching GAMEPAD_*_BTN env vars link/server.py reads) let it be
    # fixed without touching this module. Here BTN_X stands in for
    # whatever the real "arm AUTO" button turns out to be.
    state = _FakeState(nav_target=("4723.492", "N", "00044.340", "W"))
    handler = robot_state_button_handler(state, arm_auto_btn="BTN_X")

    handler(ecodes.BTN_X, True)
    assert state.modes == ["AUTO"]

    # The old default (BTN_Y) must NOT still trigger it once reassigned.
    handler(ecodes.BTN_Y, True)
    assert state.modes == ["AUTO"]


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
    assert state.drives == []


def test_drive_handler_idle_zero_output_does_not_touch_the_motors_outside_manual():
    # 2026-09-12 bug fix: outside MANUAL (AUTO here), an idle stick's
    # (0, 0) must not reach state.drive() at all -- it used to, and that
    # silently zeroed whatever PWM the autopilot had just computed on the
    # last GPS fix, which looked exactly like "AUTO mode never engages".
    state = _FakeState(mode="AUTO")
    handler = robot_state_drive_handler(state)

    handler(0, 0)
    assert state.modes == []
    assert state.drives == []


def test_drive_handler_still_zeroes_motors_on_release_during_manual_drive():
    # Releasing the stick after a genuine MANUAL drive must still stop
    # the motors normally -- only AUTO/IDLE are protected from idle
    # (0, 0) noise, not an actual ongoing manual drive.
    state = _FakeState(mode="MANUAL")
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


# --- GamepadReader: rumble (force feedback) ---------------------------------

def test_start_rumble_is_a_silent_no_op_with_no_controller_connected(monkeypatch):
    # No _device set at all -- must not raise or hang, just idle until
    # stop_rumble() is called (same "degrade without crashing" convention
    # as the rest of this class).
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    reader = GamepadReader(on_drive=lambda l, r: None)

    reader.start_rumble()
    time.sleep(0.05)
    reader.stop_rumble()
    reader._rumble_thread.join(timeout=1)
    assert not reader._rumble_thread.is_alive()


def test_rumble_uploads_once_and_replays_the_same_effect_while_active(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.start_rumble()
    time.sleep(0.15)  # ~15 refresh cycles at RUMBLE_REFRESH_S=0.01
    reader.stop_rumble()
    reader._rumble_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    plays = [c for c in device.ff_calls if c[0] == "write" and c[2] == 1]
    stops = [c for c in device.ff_calls if c[0] == "write" and c[2] == 0]
    erases = [c for c in device.ff_calls if c[0] == "erase"]

    # One upload, reused for every pulse -- not re-uploaded each time.
    assert len(uploads) == 1
    # Retriggered repeatedly while active (generous lower bound to avoid
    # timing flakiness, see the module docstring's timing-test caveat).
    assert len(plays) >= 3
    # Cleanly stopped and torn down exactly once after stop_rumble().
    assert stops == [("write", 1, 0)]
    assert erases == [("erase", 1)]


def test_start_rumble_is_idempotent(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.start_rumble()
    first_thread = reader._rumble_thread
    reader.start_rumble()  # already rumbling -- must not spawn a second thread
    assert reader._rumble_thread is first_thread

    reader.stop_rumble()
    first_thread.join(timeout=1)


def test_rumble_targets_the_controller_once_one_connects(monkeypatch):
    # start_rumble() called before any controller is found (e.g. right at
    # script startup, before Remote.fonction1()'s gamepad thread has found
    # a device yet) -- once GamepadReader._read_events() sets self._device,
    # the already-running rumble loop must pick it up on its own.
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    monkeypatch.setattr(gh, "RUMBLE_IDLE_POLL_S", 0.01)
    reader = GamepadReader(on_drive=lambda l, r: None)

    reader.start_rumble()
    time.sleep(0.02)  # no device yet -- nothing to assert on

    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader._device = device
    time.sleep(0.1)
    reader.stop_rumble()
    reader._rumble_thread.join(timeout=1)

    assert any(c[0] == "upload" for c in device.ff_calls)
    assert any(c == ("write", 1, 1) for c in device.ff_calls)


def test_start_rumble_weak_uses_the_weak_magnitude(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.start_rumble(strong=False)
    time.sleep(0.05)
    reader.stop_rumble()
    reader._rumble_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    assert len(uploads) == 1
    assert uploads[0][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_WEAK_MAGNITUDE


def test_set_intensity_re_uploads_a_new_effect_when_it_changes(monkeypatch):
    # An evdev FF effect's magnitude is baked in at upload time -- a
    # change in target intensity must erase the stale effect and upload a
    # fresh one at the new magnitude, not just keep replaying the old one.
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.start_rumble(strong=True)
    time.sleep(0.05)
    reader.set_intensity(strong=False)
    time.sleep(0.05)
    reader.stop_rumble()
    reader._rumble_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    erases = [c for c in device.ff_calls if c[0] == "erase"]
    # One upload for the initial strong effect, a second once
    # set_intensity() switched to weak.
    assert len(uploads) == 2
    assert uploads[0][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_STRONG_MAGNITUDE
    assert uploads[1][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_WEAK_MAGNITUDE
    # One erase for the stale strong effect when switching, one more for
    # final teardown at stop_rumble().
    assert len(erases) == 2


def test_start_rumble_on_an_active_vibration_updates_intensity_in_place(monkeypatch):
    # Calling start_rumble() again while already rumbling (e.g.
    # gps_log_on_full_maneuvers.py's active-condition counter re-arming
    # rumble on a 1->2 overlap) must update the intensity, not spawn a
    # second thread.
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},  # must advertise FF_RUMBLE, or _rumble_loop now refuses to upload at all
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.start_rumble(strong=True)
    time.sleep(0.05)
    first_thread = reader._rumble_thread
    reader.start_rumble(strong=False)
    assert reader._rumble_thread is first_thread
    time.sleep(0.05)
    reader.stop_rumble()
    first_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    assert uploads[-1][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_WEAK_MAGNITUDE


# --- _supports_ff_rumble / the "vibrations don't work" fix (2026-09-11) -----

def test_supports_ff_rumble_true_when_device_advertises_it():
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},
    )
    assert _supports_ff_rumble(device) is True


def test_supports_ff_rumble_false_when_ev_ff_missing_entirely():
    # This is the exact shape a controller/connection with NO force-feedback
    # support at all reports -- no EV_FF key in capabilities() whatsoever.
    device = _FakeDevice("Xbox Wireless Controller", "/dev/input/event7", {})
    assert _supports_ff_rumble(device) is False


def test_supports_ff_rumble_false_when_ev_ff_present_but_not_rumble():
    # A device could in principle expose EV_FF for some other effect type
    # without FF_RUMBLE specifically -- still not enough for this feature.
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [999]},
    )
    assert _supports_ff_rumble(device) is False


def test_rumble_loop_skips_upload_and_warns_once_when_device_lacks_ff_rumble(monkeypatch, caplog):
    # The bug this test guards against: before this fix, a device with no
    # FF_RUMBLE support made _rumble_loop retry upload_effect() forever,
    # failing (or worse, "succeeding" against a fake in tests) with zero
    # log output -- indistinguishable from "everything is fine". Now it
    # must never even attempt an upload, and must say so exactly once.
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice("Xbox Wireless Controller", "/dev/input/event7", {})  # no EV_FF at all
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    with caplog.at_level("WARNING", logger="link.gamepad_handler"):
        reader.start_rumble()
        time.sleep(0.15)  # several loop iterations at RUMBLE_REFRESH_S=0.01
        reader.stop_rumble()
        reader._rumble_thread.join(timeout=1)

    assert device.ff_calls == []  # never even tried to upload/write/erase
    unsupported_warnings = [
        r for r in caplog.records if "does not advertise FF_RUMBLE" in r.message
    ]
    assert len(unsupported_warnings) == 1  # logged once, not once per loop iteration
