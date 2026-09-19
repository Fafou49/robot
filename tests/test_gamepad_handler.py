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


def test_pwm_from_axis_deadzone_covers_documented_joystick_noise():
    # 2026-09-19: AXIS_DEADZONE was widened from ~20 to 30 raw PWM units
    # after a field report that BTN_A ("arm AUTO") appeared to do nothing --
    # see AXIS_DEADZONE's own comment in link/gamepad_handler.py for the
    # full root-cause explanation (idle-stick noise up to +/-30 units was
    # forcing MANUAL mode back on immediately after every AUTO arm).
    assert _pwm_from_axis(29 / 255) == 0
    assert _pwm_from_axis(-29 / 255) == 0
    assert abs(_pwm_from_axis(35 / 255)) > 0


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


def test_find_device_recognizes_the_older_joystick_button_set(monkeypatch):
    # 2026-09-18 fix: a controller/receiver reporting every button under
    # the OLDER, pre-"gamepad" joystick set (BTN_TRIGGER instead of
    # BTN_A) used to never be found at all -- see
    # GAMEPAD_IDENTIFYING_BUTTONS's comment in link/gamepad_handler.py.
    # This is the exact real-world failure mode that would make BTN_START
    # (and every other button) silently unreachable, not just one
    # mislabeled button.
    old_style_gamepad = _FakeDevice(
        "Generic USB Joystick", "/dev/input/event9",
        {ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RZ], ecodes.EV_KEY: [ecodes.BTN_TRIGGER, ecodes.BTN_TL]},
    )
    monkeypatch.setattr(sys.modules["evdev"], "list_devices", lambda: [old_style_gamepad.path])
    monkeypatch.setattr(sys.modules["evdev"], "InputDevice", lambda path: old_style_gamepad)

    reader = GamepadReader(on_drive=lambda l, r: None)
    assert reader._find_device() is old_style_gamepad


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
    def __init__(self, nav_target=None, mode="IDLE", is_recording=False,
                 raise_on_camera=None, raise_on_waypoint=None):
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
        # is_recording -- read directly by record_btn's dispatch (2026-09-18)
        # to decide whether to send REC_START or REC_STOP next, same as
        # the real RobotState.
        self.is_recording = is_recording
        self.camera_commands = []
        self.waypoints_saved = 0
        # Simulates camera_command()/save_waypoint() raising (e.g. no
        # camera running, no GPS fix yet) -- both must be caught inside
        # _on_button, never propagated (see that function's docstring).
        self._raise_on_camera = raise_on_camera
        self._raise_on_waypoint = raise_on_waypoint

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

    def camera_command(self, action):
        self.camera_commands.append(action)
        if self._raise_on_camera is not None:
            raise self._raise_on_camera
        if action == "REC_START":
            self.is_recording = True
        elif action == "REC_STOP":
            self.is_recording = False

    def save_waypoint(self):
        if self._raise_on_waypoint is not None:
            raise self._raise_on_waypoint
        self.waypoints_saved += 1
        return "/fake/waypoints.txt"


def test_button_handler_maps_buttons_to_state_calls():
    # 2026-09-18 remap: BTN_A re-arms AUTO (moved from BTN_Y); BTN_B
    # toggles video recording (CAM,REC_START/REC_STOP); BTN_X saves the
    # current GPS fix as a waypoint; BTN_Y takes a camera snapshot
    # (CAM,SNAP); BTN_START stops the robot and additionally calls
    # on_shutdown() -- link/server.py wires this to actually power off
    # the Raspberry Pi. There is no default stop_btn any more (see
    # test_button_handler_no_default_stop_button below).
    state = _FakeState(nav_target=("4723.492", "N", "00044.340", "W"))
    shutdown_calls = []
    handler = robot_state_button_handler(state, on_shutdown=lambda: shutdown_calls.append(True))

    handler(ecodes.BTN_A, True)
    handler(ecodes.BTN_B, True)   # REC_START (not recording yet)
    handler(ecodes.BTN_B, True)   # REC_STOP (toggles back)
    handler(ecodes.BTN_X, True)
    handler(ecodes.BTN_Y, True)
    handler(ecodes.BTN_START, True)

    assert state.modes == ["AUTO"]
    assert state.camera_commands == ["REC_START", "REC_STOP", "SNAP"]
    assert state.waypoints_saved == 1
    assert state.stop_count == 1  # only BTN_START stops now, not BTN_B
    assert shutdown_calls == [True]


def test_button_handler_refuses_to_arm_auto_with_no_target():
    # Pressing arm_auto (BTN_A) with neither a NAV point nor a GPS route
    # sent yet must not switch into AUTO -- see has_nav_target()'s own
    # docstring for why (nothing to drive toward -- it would just sit
    # there every GPS fix).
    state = _FakeState(nav_target=None)
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_A, True)
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
    handler(ecodes.BTN_A, False)
    assert state.modes == []
    assert state.stop_count == 0


def test_button_handler_button_codes_are_configurable():
    # 2026-09-12/2026-09-18: if this project's actual controller/receiver
    # reports a button under a different evdev code than assumed (the
    # same class of quirk already hit for the right stick's axis -- see
    # DEFAULT_RIGHT_Y_CODE), arm_auto_btn/record_btn/save_waypoint_btn/
    # snapshot_btn/stop_btn/shutdown_btn (or the matching GAMEPAD_*_BTN
    # env vars link/server.py reads) let it be fixed without touching
    # this module. Here BTN_Y stands in for whatever the real "arm AUTO"
    # button turns out to be.
    state = _FakeState(nav_target=("4723.492", "N", "00044.340", "W"))
    handler = robot_state_button_handler(state, arm_auto_btn="BTN_Y")

    handler(ecodes.BTN_Y, True)
    assert state.modes == ["AUTO"]

    # The default (BTN_A) must NOT still trigger it once reassigned.
    handler(ecodes.BTN_A, True)
    assert state.modes == ["AUTO"]


def test_button_handler_no_default_stop_button():
    # 2026-09-18: stop_btn defaults to None (no button bound) now that
    # BTN_B records video instead of stopping -- a deliberate
    # simplification, see the docstring. Pressing BTN_B (the old stop
    # button) must record, never stop; and with stop_btn left at its
    # None default, nothing at all reaches state.stop() except BTN_START.
    state = _FakeState()
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_B, True)
    assert state.stop_count == 0
    assert state.camera_commands == ["REC_START"]


def test_button_handler_stop_btn_can_be_rebound():
    # A dedicated stop button can still be wired back up explicitly (the
    # docstring suggests a shoulder button/bumper) -- BTN_X stands in for
    # one here (distinct from save_waypoint_btn's own default so the two
    # don't collide in this test).
    state = _FakeState()
    handler = robot_state_button_handler(state, save_waypoint_btn=None, stop_btn="BTN_X")

    handler(ecodes.BTN_X, True)
    assert state.stop_count == 1


def test_button_handler_record_toggles_based_on_is_recording():
    # record_btn reads state.is_recording to decide which command to
    # send next -- starting already-recording must send REC_STOP first.
    state = _FakeState(is_recording=True)
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_B, True)
    assert state.camera_commands == ["REC_STOP"]


def test_button_handler_record_failure_is_caught_not_raised():
    # camera_command() can raise (camera script not running, no frame
    # yet) -- must be logged, never propagated, or an uncaught exception
    # here would kill the whole GamepadReader thread, taking every other
    # button and both sticks down with it (see the docstring).
    state = _FakeState(raise_on_camera=RuntimeError("camera down"))
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_B, True)  # must not raise
    handler(ecodes.BTN_Y, True)  # must not raise (snapshot uses the same path)


def test_button_handler_save_waypoint_failure_is_caught_not_raised():
    # Same "never crash the thread" reasoning as record/snapshot above --
    # e.g. no GPS fix yet.
    state = _FakeState(raise_on_waypoint=RuntimeError("no fix yet"))
    handler = robot_state_button_handler(state)

    handler(ecodes.BTN_X, True)  # must not raise
    assert state.waypoints_saved == 0


def test_button_handler_warns_on_duplicate_button_codes(caplog):
    # Two actions accidentally sharing one evdev code (a copy-paste in
    # the GAMEPAD_*_BTN environment variables, most likely) should log
    # one clear warning at setup time rather than silently shadowing one
    # of them.
    with caplog.at_level("WARNING", logger="link.gamepad_handler"):
        robot_state_button_handler(_FakeState(), arm_auto_btn="BTN_A", snapshot_btn="BTN_A")

    duplicate_warnings = [r for r in caplog.records if "SAME evdev code" in r.message]
    assert len(duplicate_warnings) == 1


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


def test_read_events_and_drive_handler_together_survive_the_documented_joystick_noise():
    # End-to-end regression for the 2026-09-19 AXIS_DEADZONE fix: wires a
    # REAL GamepadReader._read_events() (not just _pwm_from_axis in
    # isolation) straight into robot_state_drive_handler(), and feeds it a
    # single raw ABS event of exactly the magnitude reported as idle-stick
    # noise on the field controller (see AXIS_DEADZONE's comment).
    #
    # raw=3727 on a standard signed 16-bit axis (-32768..32767) normalizes
    # to ~0.11376 -- just BELOW the new deadzone (30/255 ~= 0.11765) but
    # ABOVE the old one (0.08). Before this fix, this exact event would
    # have produced a nonzero PWM, reached on_drive(), and forced AUTO
    # straight back to MANUAL -- undoing an arm_auto_btn press within
    # milliseconds, since ABS events fire continuously. After the fix, it
    # must normalize to (0, 0) and leave an active AUTO mode untouched.
    state = _FakeState(mode="AUTO")
    drive_handler = robot_state_drive_handler(state)

    AbsInfo = types.SimpleNamespace
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {
            ecodes.EV_ABS: [ecodes.ABS_Y, ecodes.ABS_RZ],
            ecodes.EV_KEY: [ecodes.BTN_A],
        },
        events=[_Event(ecodes.EV_ABS, ecodes.ABS_Y, 3727)],  # documented noise level
    )
    device._capabilities["absinfo"] = {ecodes.EV_ABS: [
        (ecodes.ABS_Y, AbsInfo(min=-32768, max=32767)),
        (ecodes.ABS_RZ, AbsInfo(min=-32768, max=32767)),
    ]}

    def _capabilities(absinfo=False):
        if absinfo:
            return device._capabilities["absinfo"]
        return {k: v for k, v in device._capabilities.items() if k != "absinfo"}
    device.capabilities = _capabilities

    reader = GamepadReader(on_drive=drive_handler)
    reader._running = True
    reader._read_events(device)

    assert state.modes == []  # AUTO must NOT have been knocked back to MANUAL
    assert state.drives == []  # and the autopilot's own PWM must be left alone


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


# --- GamepadReader: pulse() (one-shot vibration, 2026-09-18) ---------------

def test_pulse_starts_rumble_and_stops_it_after_duration(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.pulse(strong=True, duration_s=0.05)
    assert reader._rumble_active is True
    time.sleep(0.15)  # well past the 0.05s pulse duration
    assert reader._rumble_active is False
    reader._rumble_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    assert uploads[0][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_STRONG_MAGNITUDE


def test_pulse_weak_uses_the_weak_magnitude(monkeypatch):
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.pulse(strong=False, duration_s=0.05)
    time.sleep(0.15)
    reader._rumble_thread.join(timeout=1)

    uploads = [c for c in device.ff_calls if c[0] == "upload"]
    assert uploads[0][1].effect.ff_rumble_effect.strong_magnitude == gh.RUMBLE_WEAK_MAGNITUDE


def test_pulse_called_again_resets_the_stop_timer(monkeypatch):
    # A second pulse() before the first one's timer fires must cancel and
    # replace it -- the vibration should stop duration_s after the LATEST
    # call, not the first one (see pulse()'s docstring: "the fix quality
    # flaps quickly" is the real-world scenario this protects).
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    device = _FakeDevice(
        "Xbox Wireless Controller", "/dev/input/event7",
        {ecodes.EV_FF: [ecodes.FF_RUMBLE]},
    )
    reader = GamepadReader(on_drive=lambda l, r: None)
    reader._device = device

    reader.pulse(strong=True, duration_s=0.1)
    time.sleep(0.05)
    reader.pulse(strong=False, duration_s=0.1)  # resets the timer -- should still be active at +0.08s
    time.sleep(0.08)
    assert reader._rumble_active is True  # would be False already without the reset
    time.sleep(0.15)
    assert reader._rumble_active is False
    reader._rumble_thread.join(timeout=1)


def test_pulse_is_safe_with_no_controller_connected(monkeypatch):
    # Same "degrade without crashing" convention as start_rumble()/
    # stop_rumble() themselves.
    import link.gamepad_handler as gh
    monkeypatch.setattr(gh, "RUMBLE_REFRESH_S", 0.01)
    reader = GamepadReader(on_drive=lambda l, r: None)

    reader.pulse(strong=True, duration_s=0.05)  # must not raise
    time.sleep(0.1)
    reader._rumble_thread.join(timeout=1)


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
