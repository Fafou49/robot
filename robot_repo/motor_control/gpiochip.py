"""Finds the Raspberry Pi 5's user-facing GPIO chip (RP1) by driver label
instead of a hardcoded /dev/gpiochipN number.

Moved here (2026-09-07) out of motor_control/remote_control.py, which used
to be the only caller -- now that both motor_control/motor_driver.py
(link/server.py's real motor driver) and remote_control.py's Remote class
share the same GPIO-owning logic, this lives in its own module so neither
has to import the other just to get at this one function. Behavior is
unchanged from the original.
"""
import os
import re
import subprocess

# On a Raspberry Pi 5, the 40-pin header's GPIO lines are owned by a
# separate chip (RP1, the "southbridge"), exposed as its own /dev/gpiochipN
# -- but WHICH number depends on the OS/kernel version, not the hardware:
# early Pi 5 images exposed it as gpiochip4, while a kernel/device-tree
# change (mid-2024 onward) moved it back to gpiochip0 for consistency with
# older Pi models, pushing unrelated internal chips to gpiochip10+. So a
# number that's correct today can silently stop being correct after an
# `apt upgrade` -- opening the wrong chip either fails outright (line
# offset out of range, or the device simply doesn't exist) or, worse,
# succeeds on a real but unrelated chip, silently doing nothing to the
# actual motor pins. The robust fix (Raspberry Pi's own recommendation)
# is to identify the chip by its driver *label*, "pinctrl-rp1", rather
# than a hardcoded number.
RP1_GPIOCHIP_LABEL = "pinctrl-rp1"
# Used only if auto-detection below fails (e.g. `gpiodetect` isn't
# installed, or this runs on a non-Pi5 board) -- matches what's been
# confirmed to work on this robot's Pi 5 as of 2026-09. Must be a full
# device path: gpiod v2's Chip() (unlike the older v1 API) does NOT
# resolve a bare name like "gpiochip0" against /dev/ on its own -- it
# passes the string straight to the OS open() call, so "gpiochip0"
# without the leading "/dev/" raises FileNotFoundError (confirmed by
# testing this for real on the robot's Pi 5, 2026-09-06).
FALLBACK_GPIOCHIP = "/dev/gpiochip0"


def detect_rp1_gpiochip(fallback=FALLBACK_GPIOCHIP):
    """Returns the full gpiochip device path (e.g. "/dev/gpiochip0")
    whose driver label is "pinctrl-rp1", by parsing `gpiodetect`'s output
    -- e.g. a line like "gpiochip0 [pinctrl-rp1] (54 lines)" becomes
    "/dev/gpiochip0". Can be overridden at any time with the
    ROBOT_GPIOCHIP environment variable (useful for testing off a real
    Pi 5, or if a future OS image renames the label) -- when set, it is
    used exactly as given (include the "/dev/" prefix yourself). Falls
    back to `fallback` if gpiodetect isn't available or nothing matches
    -- this never raises, so a detection glitch degrades to "try the
    last known-good chip" instead of crashing before anything using it
    even starts."""
    override = os.environ.get("ROBOT_GPIOCHIP")
    if override:
        return override

    try:
        output = subprocess.run(
            ["gpiodetect"], capture_output=True, text=True, timeout=5, check=True
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return fallback

    for line in output.splitlines():
        match = re.match(r"(gpiochip\d+)\s+\[" + re.escape(RP1_GPIOCHIP_LABEL) + r"\]", line)
        if match:
            return f"/dev/{match.group(1)}"
    return fallback
