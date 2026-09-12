"""Standalone diagnostic: prints the gamepad's stick values live, focused
on the RIGHT stick, so you can wiggle it and immediately see whether it
responds -- meant to be run right before a field-test session (like the
one this was written for) to catch a dead/miscalibrated stick before
driving anywhere, rather than discovering it mid-maneuver.

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.check_gamepad

What to do once it's running: push the RIGHT stick fully up, fully down,
fully left, fully right, then let go and leave it centered. Ctrl+C to
stop -- a summary is printed on exit (the min/max PWM values actually
seen on the right stick) with a plain verdict:

- reached close to -255 and +255 at some point -> stick looks functional
- stayed near 0 the whole time -> dead, or you forgot to move it -- rerun
  and actually push it to each extreme this time
- moved but never got close to the extremes -> under-traveling/drifted
  potentiometer, worth a closer look before relying on it tomorrow

Deliberately independent from motor_control.remote_control.Remote:
this only reads the gamepad (link.gamepad_handler.GamepadReader), no
motors, no GPIO, no GPS -- safe to run with the robot up on blocks or its
motor driver disconnected, purely to check the controller itself. Reuses
GamepadReader as-is (same device discovery/reconnect handling as every
other script that reads the gamepad in this project) rather than talking
to evdev directly, so there's exactly one place that logic lives.

Honesty note (same caveat as the rest of this project's hardware-facing
code): evdev could not be installed in the sandbox this was written in
(no PyPI access there). link.gamepad_handler.GamepadReader itself is
exercised against a stubbed device in tests/test_gamepad_handler.py, but
this specific script -- being a thin, mostly-print wrapper around it --
has NOT been run against a real controller. Run it for real before
trusting its verdict.
"""
import logging
import sys

from link.gamepad_handler import _EVDEV_AVAILABLE, GamepadReader

# PWM magnitudes at or above this count as "reached the extreme" for the
# end-of-run verdict -- not 255 itself, since a worn stick or a slightly
# generous deadzone might top out at, say, 250 without that meaning
# anything is actually wrong.
NEAR_FULL_SCALE = 200


def _verdict(right_min, right_max, near_full_scale=NEAR_FULL_SCALE):
    """Pure function, no hardware involved: given the min/max right-stick
    PWM values observed during one run, returns the plain-language
    verdict line to print. Kept standalone (rather than inlined in
    main()) so it's unit-tested without a real controller -- same
    reasoning as is_full_throttle()/is_full_rotation() elsewhere in this
    project."""
    if right_min == 0 and right_max == 0:
        return ("-> No movement detected at all on the right stick: either "
                "it's dead/disconnected, or it was never actually moved "
                "during this run -- rerun and push it to each extreme.")
    if right_min > -near_full_scale or right_max < near_full_scale:
        return ("-> Right stick moved but never got near its full range "
                "(expected close to -255/+255) -- looks under-traveling or "
                "miscalibrated, worth a closer look before tomorrow.")
    return "-> Right stick looks functional: reached both extremes."


def main():
    if not _EVDEV_AVAILABLE:
        sys.exit(
            "evdev not installed -- this script needs the same gamepad "
            "dependency as link/gamepad_handler.py. Run "
            "`pip install -r requirements.txt` on the robot (Pi #1)."
        )

    # So GamepadReader's own log.info("gamepad connected: ...")/
    # log.warning("no gamepad found...") lines are actually visible --
    # without a handler configured, Python's logging module only shows
    # WARNING and above via its bare last-resort handler, which would
    # silently swallow the (useful, here) "gamepad connected" line.
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("Looking for an Xbox-style controller (plug it in now if it "
          "isn't already -- this keeps retrying)...")
    print("Push the RIGHT stick to each extreme, then let it recenter. "
          "Ctrl+C when done.\n")

    last_printed = {"left": None, "right": None}
    seen = {"right_min": 0, "right_max": 0}

    def _on_drive(left_pwm, right_pwm):
        if right_pwm != last_printed["right"]:
            print(f"  RIGHT stick -> pwm={right_pwm:+d}")
            last_printed["right"] = right_pwm
            seen["right_min"] = min(seen["right_min"], right_pwm)
            seen["right_max"] = max(seen["right_max"], right_pwm)
        if left_pwm != last_printed["left"]:
            # Shown for context only (e.g. to notice the sticks are
            # swapped) -- this script's verdict is about the right stick.
            print(f"  (left stick, for reference) -> pwm={left_pwm:+d}")
            last_printed["left"] = left_pwm

    reader = GamepadReader(on_drive=_on_drive)
    try:
        reader.run_blocking()
    except KeyboardInterrupt:
        pass

    print("\nStopped.")
    right_min, right_max = seen["right_min"], seen["right_max"]
    print(f"Right stick PWM range seen this run: {right_min} to {right_max} "
          f"(full range is -255 to +255)")
    print(_verdict(right_min, right_max))


if __name__ == "__main__":
    main()
