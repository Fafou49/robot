"""GPS/NMEA logger for BOTH full-throttle (translation) and full-speed
rotation maneuvers, in a single run -- for defining the robot's full set
of step-response curves without deciding in advance which maneuver
you're about to perform, and without running gps_log_on_full_throttle.py
and gps_log_on_full_rotation.py side by side (which would fight over the
same GPS serial port -- see below).

Based on motor_control/remote_control.py's gamepad-driven motor control
(GPIO PWM via gpiod, joystick reading via evdev/pygame) -- this script
reuses that exact Remote class unchanged, exactly like the two
single-maneuver scripts.

Why one script instead of running the other two together: each of them
opens its own connection to the GPS serial port. Running both at once
would mean two independent readers on the SAME port, which most serial
ports don't support cleanly -- lines get unpredictably split between the
two readers instead of each one seeing the full stream, so neither log
would be reliable. This script reads the port ONCE (via
motor_control.gps_condition_logger.MultiConditionGPSLogger, watching
both conditions against every line) so each maneuver's log stays
complete, independent of the other.

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.gps_log_on_full_maneuvers

Logs land in the very same two files the individual scripts already use
-- motor_control/full_throttle_gps.log and
motor_control/full_rotation_gps.log -- with the same
FULL_THROTTLE_START/END and FULL_ROTATION_START/END markers, so nothing
downstream that already reads those files needs to change; only how
they get populated (one process instead of two) does.

IMPORTANT: don't run this at the same time as gps_log_on_full_throttle.py
or gps_log_on_full_rotation.py (or as each other) -- all three open the
same GPS serial port, and only one process can hold it at a time. The
two single-maneuver scripts are kept as-is (unchanged) for recording
just one maneuver in isolation, if that's ever preferred over this one.

Honesty note (same caveat as the rest of this project's hardware-facing
code): pyserial, evdev and gpiod could not be installed in the sandbox
this was written in (no PyPI access there), so the serial-reading loop
(in gps_condition_logger.py) and the Remote integration below were
written carefully against their documented APIs but have NOT been run
against real hardware. motor_control.remote_control.Remote guards its own
hardware imports internally (so importing it here always succeeds, even
without evdev/gpiod) -- REMOTE_HARDWARE_AVAILABLE (checked in main()
below) is the accurate signal for whether it can actually do anything.
Run this for real on the Pi, with a gamepad and GPS receiver connected,
before relying on it.
"""
from motor_control.gps_condition_logger import MultiConditionGPSLogger
from motor_control.gps_log_on_full_rotation import is_full_rotation
from motor_control.gps_log_on_full_rotation import LOG_PATH as ROTATION_LOG_PATH
from motor_control.gps_log_on_full_throttle import is_full_throttle
from motor_control.gps_log_on_full_throttle import LOG_PATH as THROTTLE_LOG_PATH
from motor_control.remote_control import REMOTE_HARDWARE_AVAILABLE, Remote


def main():
    if not REMOTE_HARDWARE_AVAILABLE:
        raise SystemExit(
            "evdev/gpiod not installed -- this script needs the same "
            "gamepad/GPIO dependencies as motor_control/remote_control.py. "
            "Run `pip install -r requirements.txt` on the robot (Pi #1)."
        )
    remote = Remote()
    gps_logger = MultiConditionGPSLogger(remote, [
        (is_full_throttle, THROTTLE_LOG_PATH, "FULL_THROTTLE"),
        (is_full_rotation, ROTATION_LOG_PATH, "FULL_ROTATION"),
    ])
    gps_logger.start()
    # Remote.fonction1() is itself a `while True` (waits for the gamepad,
    # then reads it forever, exactly like running remote_control.py
    # directly) -- this call blocks here for the lifetime of the script,
    # with the GPS logger thread running alongside it in the background.
    remote.fonction1()


if __name__ == "__main__":
    main()
