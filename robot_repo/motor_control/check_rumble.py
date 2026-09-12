"""Standalone diagnostic: finds the gamepad, reports plainly whether it
advertises FF_RUMBLE (force-feedback vibration) support at all, and if it
does, actually drives a real, feelable vibration for a few seconds at each
intensity -- meant to answer, once and for all, the question a "vibrations
don't work" report always raises: is this a genuine hardware/connection
limitation, or a bug in the code?

Why this exists: written 2026-09-11 after a report that the vibration
feature (link.gamepad_handler.GamepadReader.start_rumble(), wired into the
motor_control/gps_log_on_full_*.py field-test scripts) doesn't work on the
robot. Before this, that code silently swallowed every failure -- including
a controller/connection that simply doesn't support FF_RUMBLE at all, a
real possibility this project's own docs already flagged for Bluetooth --
with zero diagnostic trail. link/gamepad_handler.py's _rumble_loop() now
logs a clear warning in that case instead of retrying forever in silence,
but that warning only appears once you're already running a full field-test
script; this script isolates the question with nothing else going on, and
if support IS there, lets you feel both intensities directly with no GPS
condition needing to be triggered first.

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.check_rumble

What it does, in order:
1. Finds an Xbox-style controller (same discovery as everywhere else in
   this project -- an absolute axis plus BTN_A).
2. Reports its FF_RUMBLE support plainly:
   - NOT supported -> tells you directly that vibration cannot work on
     this controller/connection as configured, and to try it wired (USB)
     instead of Bluetooth before assuming the code is broken -- the same
     guidance link/gamepad_handler.py's own log warning gives, but seen
     here immediately, on its own, no field-test script required.
   - supported -> proceeds to the live test below.
3. If supported: runs a strong vibration for STRONG_DURATION_S seconds,
   stops, then a weak vibration for WEAK_DURATION_S seconds, then stops --
   so you can feel the actual difference in intensity used by
   gps_condition_logger.py's DGPS/non-DGPS distinction, not just confirm
   "something buzzed".

Deliberately independent from motor_control.remote_control.Remote and from
motor_control.check_gamepad: this only touches rumble, nothing else -- no
motors, no GPS, no stick reading -- so a "vibrations don't work" report can
be chased down without anything else in the loop.

Honesty note (same caveat as the rest of this project's hardware-facing
code): evdev could not be installed in the sandbox this was written in (no
PyPI access there). link.gamepad_handler._supports_ff_rumble() itself is
unit-tested against a stubbed device in tests/test_gamepad_handler.py, but
this specific script -- a thin wrapper driving GamepadReader/evdev calls
directly for the live vibration test -- has NOT been run against a real
controller. Run it for real before trusting its verdict, same as
check_gamepad.py and dump_gamepad_axes.py.
"""
import logging
import sys
import time

from link.gamepad_handler import (
    _EVDEV_AVAILABLE,
    GamepadReader,
    _supports_ff_rumble,
)

STRONG_DURATION_S = 2.0
WEAK_DURATION_S = 2.0


def _support_message(supported):
    """Pure function, no hardware involved: the plain-language verdict
    line for a given FF_RUMBLE support result. Kept standalone (rather
    than inlined in main()) so it's unit-tested without a real
    controller -- same reasoning as check_gamepad.py's _verdict()."""
    if supported:
        return "-> FF_RUMBLE is supported: this controller/connection can vibrate."
    return (
        "-> FF_RUMBLE is NOT supported on this controller/connection: "
        "vibration cannot work here no matter what the code does. If this "
        "controller is connected over Bluetooth, try it wired (USB) instead "
        "-- FF_RUMBLE support under Linux's xpad driver is solid over USB "
        "but can be inconsistent over Bluetooth depending on kernel/driver "
        "version. If it's already wired and still unsupported, this "
        "specific controller/receiver just doesn't do force feedback."
    )


def main():
    if not _EVDEV_AVAILABLE:
        sys.exit(
            "evdev not installed -- this script needs the same gamepad "
            "dependency as link/gamepad_handler.py. Run "
            "`pip install -r requirements.txt` on the robot (Pi #1)."
        )

    # So GamepadReader's own log.info/log.warning lines (including
    # _rumble_loop's new "does not advertise FF_RUMBLE" warning) are
    # actually visible -- see check_gamepad.py's main() for why this is
    # needed at all.
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    print("Looking for an Xbox-style controller (plug it in now if it "
          "isn't already -- this keeps retrying)...\n")

    reader = GamepadReader(on_drive=lambda l, r: None)
    device = None
    while device is None:
        device = reader._find_device()
        if device is None:
            time.sleep(reader.retry_interval)

    print(f"Found: {device.name} ({device.path})")
    supported = _supports_ff_rumble(device)
    print(_support_message(supported))

    if not supported:
        return

    print(f"\nVibrating STRONG for {STRONG_DURATION_S:.0f}s ...")
    reader._device = device
    reader.start_rumble(strong=True)
    time.sleep(STRONG_DURATION_S)
    reader.stop_rumble()
    if reader._rumble_thread is not None:
        reader._rumble_thread.join(timeout=1)

    print(f"Vibrating WEAK for {WEAK_DURATION_S:.0f}s ...")
    reader.start_rumble(strong=False)
    time.sleep(WEAK_DURATION_S)
    reader.stop_rumble()
    if reader._rumble_thread is not None:
        reader._rumble_thread.join(timeout=1)

    print("\nDone. If you felt both pulses (and the second one noticeably "
          "weaker), the feature works end-to-end -- a report of \"vibrations "
          "don't work\" in a real field-test script would then point at "
          "something else (e.g. the GPS-logging condition never actually "
          "triggering start_rumble() in the first place, rather than "
          "rumble itself). If you felt nothing despite FF_RUMBLE being "
          "reported as supported above, that's worth reporting -- it would "
          "mean a real bug rather than a hardware limitation.")


if __name__ == "__main__":
    main()
