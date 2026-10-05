"""Pi #1's own CPU temperature -- a local, instant reading (no bus, no
cable, no background thread needed), added to the PWR sentence alongside
the EPever Tracer's solar/battery/load readings so robot-webserver's
/power page can group it with the battery and controller temperatures in
one "Temperatures" panel (2026-10-05; see pages/power_explained.html on
robot-webserver for the full writeup).

Reads the kernel's own thermal sysfs file directly rather than shelling
out to `vcgencmd measure_temp` on every poll: no extra package
(libraspberrypi-bin) to depend on, and no subprocess spawned per reading.
/sys/class/thermal/thermal_zone0/temp exists on any Linux box with a
thermal zone -- every Raspberry Pi OS image included -- and holds the
temperature in millidegrees Celsius as plain text (e.g. "47800" -> 47.8).
"""

import logging

log = logging.getLogger("link.cpu_temp")

THERMAL_ZONE_PATH = "/sys/class/thermal/thermal_zone0/temp"

# Logged at most once per process (not once per PWR request, which would
# spam the log every 3s whenever this reader isn't available) -- same
# "missing hardware is expected, don't be noisy about it" reasoning as
# tracer_reader/gps_reader's own degraded-mode logging.
_warned_once = False


def read_cpu_temperature_c():
    """Returns Pi #1's CPU temperature in degrees Celsius, or None if the
    thermal zone file can't be read -- e.g. developing this off a real
    Raspberry Pi, or a kernel that numbers its thermal zones differently.
    Same best-effort "missing just means missing" degradation as the rest
    of the PWR pipeline; there's no background thread here because, unlike
    the Tracer's RS485 poll, a single sysfs read is cheap enough to just
    do fresh on every PWR request (see link.server's PWR dispatch)."""
    global _warned_once
    try:
        with open(THERMAL_ZONE_PATH, "r") as f:
            millidegrees = int(f.read().strip())
        return millidegrees / 1000.0
    except (OSError, ValueError) as exc:
        if not _warned_once:
            log.warning("could not read CPU temperature from %s: %s", THERMAL_ZONE_PATH, exc)
            _warned_once = True
        return None
