"""Standalone gamepad-driven manual robot control -- kept independent from
link/server.py's TCP command protocol on purpose. Used directly by the
motor_control/gps_log_on_*.py field-test scripts, which run on the robot
(Pi #1) with no webserver/TCP server involved at all: just a gamepad and
a GPS receiver, logging step-response data for specific maneuvers.

Rewritten (2026-09-07) to share its internals with link/server.py's own
gamepad integration instead of duplicating them: gamepad reading now
comes from link.gamepad_handler.GamepadReader (evdev-based; previously
this class had its own ~150-line pygame/evdev loop, mixed in with the
GPIO code below), and GPIO/PWM now comes from motor_control.motor_driver.
MotorDriver (previously this class had its own private copy of that too,
nearly identical to -- and for a while out of sync with -- motor_control/
pwm.py's). Reusing both here means a bug fixed once (like the 2026-09-06
PWM-thread crash and libgpiod v1->v2 fixes, back when this file still had
its own copies of that code) only needs fixing in one place going
forward.

Public interface is UNCHANGED from before this refactor: `verrou`,
`dutyCycleLeft`, `dutyCycleRight`, and `fonction1()` are exactly what
motor_control/gps_condition_logger.py depends on. ONE thing IS different
for the three scripts that use this module as an entry point
(gps_log_on_full_throttle.py / gps_log_on_full_rotation.py /
gps_log_on_full_maneuvers.py): they used to detect "evdev/pygame/gpiod
missing" by catching an ImportError from `from motor_control.
remote_control import Remote` itself, because the old version imported
those libraries unguarded at module level. This module now guards its own
hardware imports internally (matching link/gps_reader.py's pattern), so
that ImportError never happens any more -- Remote is always importable,
even with no gamepad/GPIO libraries present. The three scripts were
updated to check REMOTE_HARDWARE_AVAILABLE (below) instead, which is the
accurate replacement for what they used to infer from the import
succeeding or not.
"""
import threading

from link.gamepad_handler import GamepadReader
from motor_control.motor_driver import MotorDriver
from motor_control.motor_driver import _GPIOD_AVAILABLE as _MOTOR_GPIOD_AVAILABLE

try:
    from evdev import ecodes
    _EVDEV_AVAILABLE = True
except ImportError:  # pragma: no cover -- exercised whenever evdev isn't
    # installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    ecodes = None
    _EVDEV_AVAILABLE = False

# True only when BOTH the gamepad (evdev) and the motors (gpiod) can
# actually work -- what the gps_log_on_*.py scripts check before doing
# anything, since a field-test run needs both a controller and real
# motors to make sense at all.
REMOTE_HARDWARE_AVAILABLE = _EVDEV_AVAILABLE and _MOTOR_GPIOD_AVAILABLE


class Remote:
    """Reads an Xbox controller (left/right stick Y -> left/right motor
    duty cycle) and drives the two motors accordingly for as long as
    fonction1() is running. BTN_START stops the motors and returns from
    fonction1() -- same meaning as the original pygame-based version's
    Start button, which ended the whole program."""

    def __init__(self):
        self.verrou = threading.Lock()
        self.dutyCycleLeft = 0
        self.dutyCycleRight = 0
        self.motor_driver = MotorDriver()
        self._gamepad = GamepadReader(on_drive=self._on_drive, on_button=self._on_button)

    def _on_drive(self, left_pwm, right_pwm):
        with self.verrou:
            self.dutyCycleLeft = left_pwm
            self.dutyCycleRight = right_pwm
        self.motor_driver.drive(left_pwm, right_pwm)

    def _on_button(self, code, pressed):
        if not (pressed and _EVDEV_AVAILABLE and code == ecodes.BTN_START):
            return
        self.motor_driver.drive(0, 0)
        with self.verrou:
            self.dutyCycleLeft = 0
            self.dutyCycleRight = 0
        self._gamepad.stop()

    def fonction1(self):
        """Blocks for the lifetime of the script: starts the motor driver
        and the gamepad reader, then reads the gamepad forever (or until
        BTN_START is pressed) -- same historical behavior as the pygame-
        based version this replaces (wait for a controller, then read it
        forever), just built on the shared GamepadReader/MotorDriver now
        instead of a private copy of both."""
        self.motor_driver.start()
        self._gamepad.run_blocking()
