"""Standalone diagnostic: dumps every raw EV_KEY (button) event straight
from evdev, by numeric code and (when it matches a well-known name) a
human name -- the button equivalent of dump_gamepad_axes.py in this same
folder, written for the exact same reason that one was: a report
(2026-09-12) that BTN_Y and BTN_START "don't work as expected" on this
project's actual controller/receiver, and that AUTO mode never actually
engages.

Why this exists: link/gamepad_handler.py's robot_state_button_handler()
only ever reacts to whichever three button codes it's told to listen for
(BTN_Y/BTN_B/BTN_START by default -- see that function's arm_auto_btn/
stop_btn/shutdown_btn parameters, and link/server.py's
GAMEPAD_ARM_AUTO_BTN/GAMEPAD_STOP_BTN/GAMEPAD_SHUTDOWN_BTN environment
overrides). This is the exact same shape of bug already hit once for the
right stick's axis (see dump_gamepad_axes.py's own docstring): if this
specific controller/receiver/driver combination reports "Y" or "Start"
under a DIFFERENT evdev code than the assumed BTN_Y/BTN_START, every
press of that button is silently dropped by the handler's `if code ==
...` check -- no crash, no error, just nothing happening, which matches
exactly what was reported. A known real-world cause: some controllers
(especially wireless/Bluetooth ones, or third-party receivers) expose
"Y" as BTN_NORTH instead of BTN_Y, or "Start" as BTN_MODE, KEY_MENU, or
nothing at all under a plain button code (sometimes folded into a
different report entirely) -- this script finds out empirically which
code actually fires, instead of guessing.

Usage (on Pi #1, robot, from the repo root, with the robot's Python
control scripts NOT running -- both would otherwise fight over the same
/dev/input device, which is harmless but makes the output noisier to
read):
    python3 -m motor_control.dump_gamepad_buttons

Press EVERY button one at a time -- watch which code number fires for Y
and Start specifically. Ctrl+C to stop. Whatever code fires:
  - if it prints a name from KNOWN_BUTTON_NAMES below that matches
    (BTN_Y, BTN_NORTH, BTN_MODE, ...), pass that exact name as
    link/gamepad_handler.py's robot_state_button_handler() arm_auto_btn/
    stop_btn/shutdown_btn parameter (or, without touching code, set the
    GAMEPAD_ARM_AUTO_BTN / GAMEPAD_STOP_BTN / GAMEPAD_SHUTDOWN_BTN
    environment variable link/server.py reads -- see .env.example);
  - if it only prints a bare code number (not recognized here), look
    that number up against evdev's own ecodes module (`python3 -c
    "from evdev import ecodes; print(ecodes.keys[<code>])"` on the Pi,
    once evdev is installed) to find its real name, then use that name
    the same way.

Honesty note (same caveat as the rest of this project's hardware-facing
code): evdev could not be installed in the sandbox this was written in
(no PyPI access there), so this is written against its documented public
API (list_devices(), InputDevice, capabilities(), read_loop()) but has
NOT been run against a real controller. _button_name() (the one piece of
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

# The common gamepad button names worth resolving by hand here --
# deliberately not relying on evdev's own reverse-lookup tables (their
# exact shape isn't worth depending on for a quick diagnostic): a code
# that isn't one of these just prints as a bare number below, which is
# still enough to look it up against evdev.ecodes.keys on the Pi. Order
# matters where two names alias the same code (e.g. BTN_A == BTN_SOUTH):
# the first match in this list is what gets printed, so the "standard"
# Xbox-style name is listed before its generic alias.
KNOWN_BUTTON_NAMES = [
    "BTN_A", "BTN_B", "BTN_X", "BTN_Y",
    "BTN_SOUTH", "BTN_EAST", "BTN_NORTH", "BTN_WEST",
    "BTN_TL", "BTN_TR", "BTN_TL2", "BTN_TR2",
    "BTN_SELECT", "BTN_START", "BTN_MODE",
    "BTN_THUMBL", "BTN_THUMBR",
    "BTN_DPAD_UP", "BTN_DPAD_DOWN", "BTN_DPAD_LEFT", "BTN_DPAD_RIGHT",
    "BTN_C", "BTN_Z",
]


def _button_name(code, ecodes_module, known_names=KNOWN_BUTTON_NAMES):
    """Pure lookup, no hardware involved: reverse-maps a raw evdev EV_KEY
    code to one of KNOWN_BUTTON_NAMES by comparing against that name's
    attribute on `ecodes_module`, or None if it isn't one of them (still
    a perfectly usable result -- the raw code number alone is enough to
    look it up against evdev.ecodes.keys). Takes `ecodes_module` as a
    parameter (rather than importing evdev.ecodes at call time) so it's
    unit-testable against a plain fake namespace, no evdev installation
    required."""
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
    print("Press EVERY button, one at a time -- watch which code number "
          "fires for Y and Start specifically. Release events print too "
          "(value 0) so a stuck/always-on button is obvious. Ctrl+C to "
          "stop.\n")

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_KEY:
                name = _button_name(event.code, ecodes)
                label = f"{name} (code {event.code})" if name else f"code {event.code}"
                state = {0: "released", 1: "pressed", 2: "held"}.get(event.value, event.value)
                print(f"  {label}: {state}")
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
