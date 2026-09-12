"""Shared pytest setup for this test suite: stubs the `evdev` and `gpiod`
packages in sys.modules when the real libraries aren't installed -- same
honesty note as the rest of this project's hardware-facing code: neither
could be installed in the sandbox this was written in (no PyPI access
there), so link/gamepad_handler.py's and motor_control/motor_driver.py's
own tests exercise them against fake devices instead of the real
libraries. If a real library IS importable (e.g. on the Pi, with
requirements.txt installed), this does nothing for it and every test just
uses it directly.

Lives in conftest.py rather than inline in one test file because pytest
loads conftest.py exactly once, before collecting ANY test module in this
directory -- unlike a stub set up inside one test file, which only takes
effect for OTHER files that import the stubbed package's dependents if
that stubbing file happens to be collected first. Relying on alphabetical
collection order for that worked by accident for a while:
- evdev: test_gamepad_handler.py's own inline stub (where it used to live,
  2026-09-11 and earlier) sorted before every file that needed
  link.gamepad_handler -- until tests/test_check_gamepad.py, sorting
  earlier, started importing it for real, permanently caching the
  "real evdev genuinely absent, nothing stubbed" version of that module.
- gpiod: test_motor_driver.py's own inline stub (moved here 2026-09-11,
  same day, once this exact failure mode repeated) sorted before every
  OTHER file that needed motor_control.motor_driver -- until
  tests/test_link_server.py, sorting earlier (`link` < `motor_driver`),
  started importing it for real via link.server -> motor_control.
  motor_driver, permanently caching motor_driver._GPIOD_AVAILABLE=False
  and breaking test_motor_driver.py's "working fake chip" tests, which
  need MotorDriver.start() to actually succeed against the stub.
Python only executes a module's top-level code once, on first import, so
in both cases the fix is the same: stub the dependency once here, before
collection even starts, removing the ordering dependency for good rather
than hoping alphabetical order keeps saving it a third time.
"""
import sys
import types

try:
    import evdev  # noqa: F401 -- only probing whether the real library is installed
    _REAL_EVDEV_AVAILABLE = True
except ImportError:
    _REAL_EVDEV_AVAILABLE = False

try:
    import gpiod  # noqa: F401 -- only probing whether the real library is installed
    _REAL_GPIOD_AVAILABLE = True
except ImportError:
    _REAL_GPIOD_AVAILABLE = False


class _FFRumble:
    def __init__(self, strong_magnitude, weak_magnitude):
        self.strong_magnitude = strong_magnitude
        self.weak_magnitude = weak_magnitude


class _FFTrigger:
    def __init__(self, button, interval):
        self.button = button
        self.interval = interval


class _FFReplay:
    def __init__(self, length, delay):
        self.length = length
        self.delay = delay


class _FFEffectType:
    def __init__(self, ff_rumble_effect=None):
        self.ff_rumble_effect = ff_rumble_effect


class _FFEffect:
    def __init__(self, effect_type, effect_id, direction, trigger, replay, effect):
        self.effect_type = effect_type
        self.effect_id = effect_id
        self.direction = direction
        self.trigger = trigger
        self.replay = replay
        self.effect = effect


# Real evdev event codes for a standard Xbox controller (xpad driver),
# used symbolically (ecodes.BTN_A etc.) everywhere in gamepad_handler.py
# and in tests/test_gamepad_handler.py, never hardcoded -- so the exact
# numeric values below only need to be internally consistent, not match
# the kernel's real ones.
ecodes = types.SimpleNamespace(
    EV_KEY=1, EV_ABS=3, EV_FF=21,
    ABS_Y=1, ABS_Z=2, ABS_RY=4, ABS_RZ=5,
    BTN_A=304, BTN_B=305, BTN_X=307, BTN_Y=308, BTN_START=315,
    FF_RUMBLE=80,
)

ff = types.SimpleNamespace(
    Rumble=_FFRumble, Trigger=_FFTrigger, Replay=_FFReplay,
    EffectType=_FFEffectType, Effect=_FFEffect,
)

if not _REAL_EVDEV_AVAILABLE:
    _evdev_stub = types.ModuleType("evdev")
    _evdev_stub.ecodes = ecodes
    _evdev_stub.ff = ff
    _evdev_stub.list_devices = lambda: []
    _evdev_stub.InputDevice = lambda path: None
    sys.modules["evdev"] = _evdev_stub
    sys.modules["evdev.ecodes"] = ecodes
    sys.modules["evdev.ff"] = ff


# Stub gpiod (and gpiod.line, since motor_control/motor_driver.py does
# `from gpiod.line import Direction, Value`) just enough to import
# motor_driver.py without the real package installed. Left at this
# module-level default, Chip(...) returns a plain object with no
# request_lines() method, so MotorDriver.start() hits its "could not open
# GPIO chip" except branch -- i.e. the degrade-gracefully path. Any test
# that wants a *working* fake chip (e.g. tests/test_motor_driver.py)
# overrides sys.modules["gpiod"].Chip itself via monkeypatch, same as
# tests/test_gamepad_handler.py overrides bits of the evdev stub above.
if not _REAL_GPIOD_AVAILABLE:
    _gpiod_line_stub = types.ModuleType("gpiod.line")
    _gpiod_line_stub.Direction = types.SimpleNamespace(OUTPUT="OUTPUT")
    _gpiod_line_stub.Value = types.SimpleNamespace(ACTIVE="ACTIVE", INACTIVE="INACTIVE")

    _gpiod_stub = types.ModuleType("gpiod")
    _gpiod_stub.line = _gpiod_line_stub
    _gpiod_stub.LineSettings = lambda **kw: kw
    _gpiod_stub.Chip = lambda *a, **kw: object()

    sys.modules["gpiod"] = _gpiod_stub
    sys.modules["gpiod.line"] = _gpiod_line_stub
