"""GPS/NMEA logger triggered by full-speed rotation in place.

Based on motor_control/remote_control.py's gamepad-driven motor control
(GPIO PWM via gpiod, joystick reading via evdev/pygame) -- this script
reuses that exact Remote class unchanged (same gamepad, same motors) and
adds a second, independent background thread (motor_control.
gps_condition_logger.ConditionGPSLogger) that logs NMEA sentences from
the GPS receiver (see gps_condition_logger.py's _is_gps_sentence() --
2026-09-11 fix: only genuine GPS frames are kept, not the RTCM/DGPS
correction traffic gps/dgps_transfer.py writes to this same serial
device), but ONLY for as long as the two motors are pushed
to full speed in OPPOSITE directions (dutyCycleLeft/dutyCycleRight ==
+255/-255 or -255/+255) -- i.e. spinning in place at maximum speed.
Useful to define the robot's step response on rotation, the counterpart
to gps_log_on_full_throttle.py's straight-line case: correlate GPS
course/position with what the robot does while pivoting at maximum PWM,
without sifting through a full trip's worth of GPS data.

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.gps_log_on_full_rotation

Logged lines land in motor_control/full_rotation_gps.log (created next to
this file), one per row, each prefixed with a wall-clock timestamp, with
FULL_ROTATION_START/FULL_ROTATION_END marker lines around each period at
full-speed rotation. Nothing is written outside of a +255/-255 (or
-255/+255) pair -- an empty (or marker-only) log file after a session
just means that exact pivot was never reached, not a bug. Note that GPS
position barely moves during a pure pivot (the robot spins near a fixed
point) -- it's the GPS *course/heading* field that is the interesting
signal here, not position, unlike gps_log_on_full_throttle.py.

While a full-rotation period is being logged, the gamepad also vibrates
continuously (link.gamepad_handler.GamepadReader.start_rumble(), wired
below via ConditionGPSLogger's on_transition callback) -- physical
confirmation for the driver that this exact pivot is being recorded,
without needing to glance at a screen. Stops the instant the pivot drops
below full speed.

The vibration is also strong or weak depending on GPS fix quality
(on_gps_quality below, driven by the GGA sentence's quality field --
see motor_control/gps_condition_logger.py's DGPS_QUALITY): strong while
the current fix is DGPS-corrected, weak otherwise -- a live warning, felt
while driving, that this stretch of the log has a degraded fix.

Honesty note (same caveat as link/gps_reader.py): pyserial, evdev and
gpiod (Remote's own dependencies) could not be installed in the sandbox
this was written in (no PyPI access there), so the serial-reading loop
(in gps_condition_logger.py) and the Remote integration below were
written carefully against their documented APIs but have NOT been run
against real hardware. motor_control.remote_control.Remote guards its own
hardware imports internally (so importing it here always succeeds, even
without evdev/gpiod) -- REMOTE_HARDWARE_AVAILABLE (checked in main()
below) is the accurate signal for whether it can actually do anything,
letting this module still be *imported* -- and its one piece of pure
logic, is_full_rotation(), actually unit-tested -- on a machine without
that hardware/those libraries. Run this for real on the Pi, with a
gamepad and GPS receiver connected, before relying on it.
"""
import os

from motor_control.gps_condition_logger import ConditionGPSLogger
from motor_control.remote_control import REMOTE_HARDWARE_AVAILABLE, Remote

# Both motors must be at exactly this magnitude -- not just close to it
# -- and in opposite directions, to count as "full-speed rotation". 255
# is the hard maximum PWM value Remote's joystick-to-duty-cycle
# conversion can ever produce (int(axis * 255) at an axis reading of
# +/-1.0), so this is the true maximum, not a threshold picked ad hoc.
FULL_SPEED = 255

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "full_rotation_gps.log")


def is_full_rotation(left, right, speed=FULL_SPEED):
    """Pure predicate, no hardware involved: True once the two duty
    cycles are at +/-`speed` (default FULL_SPEED) in OPPOSITE
    directions -- either (+speed, -speed) or (-speed, +speed) -- i.e.
    the robot pivoting in place at maximum speed, one side forward and
    the other backward. Same-direction full throttle (see
    is_full_throttle() in gps_log_on_full_throttle.py) does NOT count as
    rotation here, and neither does a pivot at less than full speed.
    Kept standalone (rather than inlined in the logger loop) so it can
    be unit-tested without a real Remote/gamepad/GPIO chip."""
    return (left == speed and right == -speed) or (left == -speed and right == speed)


def main():
    if not REMOTE_HARDWARE_AVAILABLE:
        raise SystemExit(
            "evdev/gpiod not installed -- this script needs the same "
            "gamepad/GPIO dependencies as motor_control/remote_control.py. "
            "Run `pip install -r requirements.txt` on the robot (Pi #1)."
        )
    remote = Remote()

    def _on_transition(trigger_name, triggered):
        # Buzz the gamepad for as long as (and only while) this exact
        # maneuver is being logged -- see the module docstring above.
        # Seed the intensity with whatever fix quality is already known
        # so the very first pulse is already right (see
        # gps_log_on_full_throttle.py's identical comment).
        if triggered:
            remote.gamepad.start_rumble(strong=gps_logger.last_is_dgps)
        else:
            remote.gamepad.stop_rumble()

    def _on_gps_quality(is_dgps):
        # Called only while this maneuver is actively being logged.
        remote.gamepad.set_intensity(strong=is_dgps)

    gps_logger = ConditionGPSLogger(
        remote, is_full_rotation, LOG_PATH, trigger_name="FULL_ROTATION",
        on_transition=_on_transition, on_gps_quality=_on_gps_quality,
    )
    gps_logger.start()
    # Remote.fonction1() is itself a `while True` (waits for the gamepad,
    # then reads it forever, exactly like running remote_control.py
    # directly) -- this call blocks here for the lifetime of the script,
    # with the GPS logger thread running alongside it in the background.
    remote.fonction1()


if __name__ == "__main__":
    main()
