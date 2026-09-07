"""GPS/NMEA logger triggered by full-throttle driving in a straight line.

Based on motor_control/remote_control.py's gamepad-driven motor control
(GPIO PWM via gpiod, joystick reading via evdev/pygame) -- this script
reuses that exact Remote class unchanged (same gamepad, same motors) and
adds a second, independent background thread (motor_control.
gps_condition_logger.ConditionGPSLogger) that logs raw NMEA sentences
from the GPS receiver, but ONLY for as long as both motors are pushed to
full throttle in the same direction (dutyCycleLeft == dutyCycleRight ==
255) -- i.e. driving straight ahead at maximum speed. Useful to define
the robot's step response on translation: correlate GPS speed/position
with what it does at maximum forward PWM (top speed, straight-line
drift...) without sifting through a full trip's worth of GPS data.

See gps_log_on_full_rotation.py for the equivalent on rotation (motors
pushed in opposite directions).

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.gps_log_on_full_throttle

Logged lines land in motor_control/full_throttle_gps.log (created next to
this file), one per row, each prefixed with a wall-clock timestamp, with
FULL_THROTTLE_START/FULL_THROTTLE_END marker lines around each period at
full throttle. Nothing is written while at least one motor is below 255
-- an empty (or marker-only) log file after a session just means full
throttle was never reached, not a bug.

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
logic, is_full_throttle(), actually unit-tested -- on a machine without
that hardware/those libraries. Run this for real on the Pi, with a
gamepad and GPS receiver connected, before relying on it.
"""
import os

from motor_control.gps_condition_logger import ConditionGPSLogger
from motor_control.remote_control import REMOTE_HARDWARE_AVAILABLE, Remote

# Both motors must be at exactly this value -- not just close to it -- to
# count as "full throttle". 255 is the hard maximum PWM value Remote's
# joystick-to-duty-cycle conversion can ever produce (int(axis * 255) at
# an axis reading of +/-1.0), so this is the true maximum, not a
# threshold picked ad hoc.
FULL_THROTTLE = 255

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "full_throttle_gps.log")


def is_full_throttle(left, right, threshold=FULL_THROTTLE):
    """Pure predicate, no hardware involved: True once both duty cycles
    have reached `threshold` (default FULL_THROTTLE) IN THE SAME
    direction, i.e. driving straight ahead at maximum speed. Kept
    standalone (rather than inlined in the logger loop) so it can be
    unit-tested without a real Remote/gamepad/GPIO chip."""
    return left == threshold and right == threshold


def main():
    if not REMOTE_HARDWARE_AVAILABLE:
        raise SystemExit(
            "evdev/gpiod not installed -- this script needs the same "
            "gamepad/GPIO dependencies as motor_control/remote_control.py. "
            "Run `pip install -r requirements.txt` on the robot (Pi #1)."
        )
    remote = Remote()
    gps_logger = ConditionGPSLogger(
        remote, is_full_throttle, LOG_PATH, trigger_name="FULL_THROTTLE"
    )
    gps_logger.start()
    # Remote.fonction1() is itself a `while True` (waits for the gamepad,
    # then reads it forever, exactly like running remote_control.py
    # directly) -- this call blocks here for the lifetime of the script,
    # with the GPS logger thread running alongside it in the background.
    remote.fonction1()


if __name__ == "__main__":
    main()
