"""GPS/NMEA logger triggered by full-speed rotation in place.

Based on motor_control/remote_control.py's gamepad-driven motor control
(GPIO PWM via gpiod, joystick reading via evdev/pygame) -- this script
reuses that exact Remote class unchanged (same gamepad, same motors) and
adds a second, independent background thread (motor_control.
gps_condition_logger.ConditionGPSLogger) that logs raw NMEA sentences
from the GPS receiver, but ONLY for as long as the two motors are pushed
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

Honesty note (same caveat as link/gps_reader.py): pyserial, evdev, pygame
and gpiod (Remote's own dependencies) could not be installed in the
sandbox this was written in (no PyPI access there), so the serial-reading
loop (in gps_condition_logger.py) and the Remote integration below were
written carefully against their documented APIs but have NOT been run
against real hardware. Both imports are wrapped in try/except (same
pattern as link/gps_reader.py) precisely so this module can still be
*imported* -- and its one piece of pure logic, is_full_rotation(),
actually unit-tested -- on a machine without that hardware/those
libraries. Run this for real on the Pi, with a gamepad and GPS receiver
connected, before relying on it.
"""
import os

from motor_control.gps_condition_logger import ConditionGPSLogger

try:
    from motor_control.remote_control import Remote
    _REMOTE_AVAILABLE = True
except ImportError:  # pragma: no cover -- evdev/pygame/gpiod missing.
    Remote = None
    _REMOTE_AVAILABLE = False

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
    if not _REMOTE_AVAILABLE:
        raise SystemExit(
            "evdev/pygame/gpiod not installed -- this script needs the same "
            "gamepad/GPIO dependencies as motor_control/remote_control.py. "
            "Run `pip install -r requirements.txt` on the robot (Pi #1)."
        )
    remote = Remote()
    gps_logger = ConditionGPSLogger(
        remote, is_full_rotation, LOG_PATH, trigger_name="FULL_ROTATION"
    )
    gps_logger.start()
    # Remote.fonction1() is itself a `while True` (waits for the gamepad,
    # then reads it forever, exactly like running remote_control.py
    # directly) -- this call blocks here for the lifetime of the script,
    # with the GPS logger thread running alongside it in the background.
    remote.fonction1()


if __name__ == "__main__":
    main()
