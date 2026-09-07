"""Tests for motor_control.motor_driver.

_sense_line_values() is pure logic (no hardware) and always runs. The
MotorDriver class needs gpiod, which isn't installable in this sandbox
(no PyPI access) -- same honesty note as the rest of this project's
hardware-facing code: gpiod is stubbed out below just enough to exercise
MotorDriver's control flow (degrade-gracefully paths, and the PWM loop
driving a *fake* set of GPIO lines that just record every set_value()
call) -- this does NOT mean it has been run against real hardware.
"""
import sys
import time
import types

import pytest

# Stub gpiod (and gpiod.line, since motor_driver.py does
# `from gpiod.line import Direction, Value`) so importing motor_driver
# here doesn't require the real package to be installed. Each test that
# wants a *working* fake chip overrides sys.modules["gpiod"].Chip itself;
# left as the module-level default, Chip(...) returns a plain object with
# no request_lines() method, so MotorDriver.start() hits its "could not
# open GPIO chip" except branch -- i.e. the degrade-gracefully path.
_line_stub = types.ModuleType("gpiod.line")
_line_stub.Direction = types.SimpleNamespace(OUTPUT="OUTPUT")
_line_stub.Value = types.SimpleNamespace(ACTIVE="ACTIVE", INACTIVE="INACTIVE")
sys.modules["gpiod.line"] = _line_stub

_gpiod_stub = types.ModuleType("gpiod")
_gpiod_stub.line = _line_stub
_gpiod_stub.LineSettings = lambda **kw: kw
_gpiod_stub.Chip = lambda *a, **kw: object()
sys.modules["gpiod"] = _gpiod_stub

from motor_control.motor_driver import (  # noqa: E402
    MOTOR1_SENS1, MOTOR1_SENS2, MOTOR2_SENS1, MOTOR2_SENS2,
    MotorDriver, _sense_line_values,
)


# --- _sense_line_values (pure logic) ----------------------------------------

def test_neutral_within_deadzone():
    for duty in (0, 20, -20, 15, -15):
        assert _sense_line_values(duty, counter=0) == (False, False)
        assert _sense_line_values(duty, counter=254) == (False, False)


def test_positive_duty_drives_sens1_only():
    # duty=100: sens1 active while counter <= 100, inactive after.
    assert _sense_line_values(100, counter=0) == (True, False)
    assert _sense_line_values(100, counter=100) == (True, False)
    assert _sense_line_values(100, counter=101) == (False, False)


def test_negative_duty_drives_sens2_only():
    assert _sense_line_values(-100, counter=0) == (False, True)
    assert _sense_line_values(-100, counter=100) == (False, True)
    assert _sense_line_values(-100, counter=101) == (False, False)


def test_max_duty_active_for_the_whole_period():
    assert _sense_line_values(255, counter=0) == (True, False)
    assert _sense_line_values(255, counter=254) == (True, False)


def test_deadzone_boundary_is_exclusive():
    # duty_cycle must be STRICTLY greater than the deadzone to count as
    # "driving" -- exactly at the boundary is still neutral.
    assert _sense_line_values(21, counter=0) == (True, False)
    assert _sense_line_values(-21, counter=0) == (False, True)


# --- MotorDriver -------------------------------------------------------------

def test_drive_updates_values_even_before_start():
    driver = MotorDriver(gpiochip="/dev/fake0")
    driver.drive(150, -150)
    assert (driver.left_pwm, driver.right_pwm) == (150, -150)


def test_start_degrades_gracefully_when_chip_open_fails(monkeypatch):
    def _raise(*a, **kw):
        raise OSError("no such device")
    monkeypatch.setattr(sys.modules["gpiod"], "Chip", _raise)

    driver = MotorDriver(gpiochip="/dev/fake0")
    thread = driver.start()
    assert thread is None
    # Still safe to call -- just has no physical effect.
    driver.drive(100, 100)
    assert (driver.left_pwm, driver.right_pwm) == (100, 100)


class _FakeLines:
    """Stands in for the LineRequest object gpiod.Chip.request_lines()
    returns -- records every set_value() call so the PWM loop's actual
    GPIO-toggling behavior can be asserted on."""

    def __init__(self):
        self.calls = []
        self.values = {}

    def set_value(self, offset, value):
        self.calls.append((offset, value))
        self.values[offset] = value


def test_drive_loop_toggles_expected_lines(monkeypatch):
    fake_lines = _FakeLines()

    class _FakeChip:
        def request_lines(self, consumer, config):
            assert set(config.keys()) == {MOTOR1_SENS1, MOTOR1_SENS2, MOTOR2_SENS1, MOTOR2_SENS2}
            return fake_lines

    monkeypatch.setattr(sys.modules["gpiod"], "Chip", lambda path: _FakeChip())

    driver = MotorDriver(gpiochip="/dev/fake0")
    driver.drive(255, -255)  # full forward left, full reverse right, before start
    thread = driver.start()
    assert thread is not None

    # Let the loop run through at least one full 255-counter period.
    time.sleep(0.3)
    driver.shutdown()
    thread.join(timeout=2)

    # Left motor (sens1) should have been driven ACTIVE at least once,
    # right motor (sens2) too -- confirms the loop is actually reading
    # left_pwm/right_pwm and driving the corresponding lines, not just
    # sitting idle.
    left_active_calls = [v for (offset, v) in fake_lines.calls if offset == MOTOR1_SENS1]
    right_active_calls = [v for (offset, v) in fake_lines.calls if offset == MOTOR2_SENS2]
    assert "ACTIVE" in left_active_calls
    assert "ACTIVE" in right_active_calls
    # The *other* line for each motor must never have been driven active
    # -- full-forward-left/full-reverse-right must not also toggle
    # sens2/sens1 for those motors.
    assert all(v == "INACTIVE" for (offset, v) in fake_lines.calls if offset == MOTOR1_SENS2)
    assert all(v == "INACTIVE" for (offset, v) in fake_lines.calls if offset == MOTOR2_SENS1)


def test_stop_zeroes_pwm():
    driver = MotorDriver(gpiochip="/dev/fake0")
    driver.drive(200, 200)
    driver.stop()
    assert (driver.left_pwm, driver.right_pwm) == (0, 0)
