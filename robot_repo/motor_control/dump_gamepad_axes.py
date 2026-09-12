"""Standalone diagnostic: dumps every raw EV_ABS axis event straight from
evdev, by numeric code and (when it matches a well-known name) a human
name -- unlike link/gamepad_handler.py's normal reading, which only ever
reacts to whichever two axis codes it's told to listen for (ABS_Y/ABS_RY
by default -- see DEFAULT_LEFT_Y_CODE/DEFAULT_RIGHT_Y_CODE), this listens
to EVERY absolute axis with no filtering at all.

Why this exists: written 2026-09-11 after a report that the LEFT stick
works through GamepadReader but the RIGHT stick does nothing, even
though _read_events()'s code path is identical for both sticks -- the
only difference between them is which axis CODE each one listens for
(see link/gamepad_handler.py, `if event.code in (left_code, right_code)`
then `if event.code == left_code: ... else: ...`). GamepadReader.on_drive
is only ever called for an event whose evdev code equals exactly the
configured left/right code -- if this specific controller/receiver/
driver combination reports the right stick's vertical axis under a
DIFFERENT code than the assumed ABS_RY, every right-stick event is
silently dropped by that filter: no crash, no error, just nothing
happening, which matches exactly what was reported. A known real-world
cause: some controllers/receivers/drivers (especially over Bluetooth, or
with third-party dongles for older Xbox 360 controllers) expose the
right stick's Y-axis as ABS_RZ or ABS_Z instead of the "standard" xpad
mapping's ABS_RY. This script finds out empirically which code the right
stick actually fires, instead of guessing.

Usage (on Pi #1, robot, from the repo root):
    python3 -m motor_control.dump_gamepad_axes

Move the RIGHT stick (and, for comparison, the left one and the
triggers) -- every axis event prints its code number, its name if it's
one of the common ones this script recognizes, and its raw value.
Ctrl+C to stop. Whatever code fires when you move the right stick is
what link.gamepad_handler.GamepadReader's right_y_code (or
DEFAULT_RIGHT_Y_CODE, to fix it for everyone rather than pass it in
each time) needs to be set to.

Outcome (2026-09-11): run for real on this project's actual controller,
this confirmed the right stick fires as ABS_RZ, not ABS_RY -- see
link/gamepad_handler.py's DEFAULT_RIGHT_Y_CODE, updated accordingly.
(The first read of this script's output was ABS_Z -- the left trigger's
code in the standard mapping -- corrected to ABS_RZ once that mix-up was
caught.) This script stays useful beyond that one fix, though: for a
different controller/receiver, or if the mapping ever needs re-checking
(e.g. after a kernel/driver update), it finds out empirically again
instead of assuming the same conclusion still holds.

Honesty note (same caveat as the rest of this project's hardware-facing
code): evdev could not be installed in the sandbox this was written in
(no PyPI access there), so this is written against its documented public
API (list_devices(), InputDevice, capabilities(), read_loop()) but has
NOT been run against a real controller. _axis_name() (the one piece of
pure logic here) is unit-tested; the device discovery/read loop below it
is not.
"""
import sys

try:
    import evdev
    from evdev import ecodes
except ImportError:  # pragma: no cover -- exercised whenever evdev isn't
    # installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    evdev = None
    ecodes = None

# The common analog-stick/trigger axis names worth resolving by hand here
# -- deliberately not relying on evdev's own reverse-lookup tables (their
# exact shape isn't worth depending on for a quick diagnostic): a code
# that isn't one of these just prints as a bare number below, which is
# all that's actually needed to fix DEFAULT_RIGHT_Y_CODE.
KNOWN_AXIS_NAMES = [
    "ABS_X", "ABS_Y", "ABS_Z", "ABS_RX", "ABS_RY", "ABS_RZ",
    "ABS_HAT0X", "ABS_HAT0Y", "ABS_THROTTLE", "ABS_BRAKE", "ABS_GAS",
]


def _axis_name(code, ecodes_module, known_names=KNOWN_AXIS_NAMES):
    """Pure lookup, no hardware involved: reverse-maps a raw evdev ABS
    code to one of KNOWN_AXIS_NAMES by comparing against that name's
    attribute on `ecodes_module`, or None if it isn't one of them (still
    a perfectly usable result -- the raw code number alone is enough to
    fix right_y_code). Takes `ecodes_module` as a parameter (rather than
    importing evdev.ecodes at call time) so it's unit-testable against a
    plain fake namespace, no evdev installation required."""
    for name in known_names:
        if getattr(ecodes_module, name, None) == code:
            return name
    return None


def _find_controller():
    """Same device-discovery predicate as
    link.gamepad_handler.GamepadReader._find_device() (an absolute axis
    plus BTN_A) -- duplicated rather than imported so this script has no
    dependency on GamepadReader at all: the whole point is to look at the
    controller with NO assumptions from that class baked in, in case the
    bug turns out to be there too."""
    for path in evdev.list_devices():
        try:
            candidate = evdev.InputDevice(path)
        except OSError:
            continue
        capabilities = candidate.capabilities()
        has_abs = ecodes.EV_ABS in capabilities
        has_a_button = (
            ecodes.EV_KEY in capabilities
            and ecodes.BTN_A in capabilities[ecodes.EV_KEY]
        )
        if has_abs and has_a_button:
            return candidate
    return None


def main():
    if evdev is None:
        sys.exit(
            "evdev not installed -- this script needs the same gamepad "
            "dependency as link/gamepad_handler.py. Run "
            "`pip install -r requirements.txt` on the robot (Pi #1)."
        )

    device = _find_controller()
    if device is None:
        sys.exit("No Xbox-style controller found (looked for a device "
                  "exposing an absolute axis and BTN_A). Plug it in and retry.")

    print(f"Found: {device.name} ({device.path})")
    print("Move EVERY stick and trigger, one at a time -- watch which "
          "code number moves when you touch the RIGHT stick specifically. "
          "Ctrl+C to stop.\n")

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS:
                name = _axis_name(event.code, ecodes)
                label = f"{name} (code {event.code})" if name else f"code {event.code}"
                print(f"  {label}: {event.value}")
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
