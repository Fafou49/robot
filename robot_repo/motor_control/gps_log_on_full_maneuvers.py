"""GPS/NMEA logger for BOTH full-throttle (translation) and full-speed
rotation maneuvers, in a single run -- for defining the robot's full set
of step-response curves without deciding in advance which maneuver
you're about to perform, and without running gps_log_on_full_throttle.py
and gps_log_on_full_rotation.py side by side (which would fight over the
same GPS serial port -- see below).

Like the two single-maneuver scripts, only genuine GPS NMEA frames are
logged, not the RTCM/DGPS correction traffic gps/dgps_transfer.py writes
to this same serial device (see gps_condition_logger.py's
_is_gps_sentence(), 2026-09-11 fix) -- MultiConditionGPSLogger applies
that filter the same way regardless of how many conditions it watches.

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

While EITHER maneuver is being logged, the gamepad also vibrates
continuously (link.gamepad_handler.GamepadReader.start_rumble(), wired
below via MultiConditionGPSLogger's on_transition callback) -- physical
confirmation for the driver that something is being recorded right now,
without needing to glance at a screen. The two conditions are tracked
independently but share one rumble: an active-condition counter only
stops the vibration once BOTH have ended, so going straight from one
maneuver into the other (no gap in between) doesn't cause a spurious
buzz/pause/buzz -- see main() below.

The vibration is also strong or weak depending on GPS fix quality
(on_gps_quality below, driven by the GGA sentence's quality field --
see motor_control/gps_condition_logger.py's DGPS_QUALITY): strong while
the current fix is DGPS-corrected, weak otherwise, for whichever
maneuver is currently being recorded (quality is a single shared GPS
stream, not per-maneuver).

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

    # Counts how many of the two conditions are currently active (0, 1,
    # or 2) so the rumble only stops once neither is triggered any more
    # -- see the module docstring above for why a plain per-condition
    # start/stop would risk a spurious gap between back-to-back maneuvers.
    active_conditions = {"count": 0}

    def _on_transition(trigger_name, triggered):
        active_conditions["count"] += 1 if triggered else -1
        if active_conditions["count"] > 0:
            # Seed the intensity with whatever fix quality is already
            # known so the very first pulse of a fresh recording is
            # already right (see gps_log_on_full_throttle.py's identical
            # comment) -- harmless to call again on a 1->2 overlap too.
            remote.gamepad.start_rumble(strong=gps_logger.last_is_dgps)
        else:
            remote.gamepad.stop_rumble()

    def _on_gps_quality(is_dgps):
        # Called only while at least one of the two maneuvers is actively
        # being logged.
        remote.gamepad.set_intensity(strong=is_dgps)

    gps_logger = MultiConditionGPSLogger(remote, [
        (is_full_throttle, THROTTLE_LOG_PATH, "FULL_THROTTLE"),
        (is_full_rotation, ROTATION_LOG_PATH, "FULL_ROTATION"),
    ], on_transition=_on_transition, on_gps_quality=_on_gps_quality)
    gps_logger.start()
    # Remote.fonction1() is itself a `while True` (waits for the gamepad,
    # then reads it forever, exactly like running remote_control.py
    # directly) -- this call blocks here for the lifetime of the script,
    # with the GPS logger thread running alongside it in the background.
    remote.fonction1()


if __name__ == "__main__":
    main()
