"""Pi #1's own uptime -- how long it's been awake since its last boot, a
local, instant reading (no bus, no cable, no background thread needed),
added to the PWR sentence alongside the new onboard_time field (2026-10-10,
explicit user request) so robot-webserver's /power page can show a small
"lifetime" readout right next to it. Deliberately a SEPARATE field/concept
from onboard_time: uptime comes from the kernel's own monotonic boot clock
(/proc/uptime), so unlike the wall-clock-derived onboard_time it is immune
to the "no RTC battery, clock not synced yet" problem this project's
system_clock_is_plausible() guards against elsewhere (link.power_history)
-- it's simply always correct, synced clock or not, which is exactly why
it's useful as a companion reading next to a wall-clock value that might
not be trustworthy yet.

Reads /proc/uptime directly rather than shelling out to the `uptime`
command: no subprocess per poll, and the file's first field (seconds since
boot, as a plain decimal) is already exactly what's needed -- present on
any Linux box, Raspberry Pi OS included, same "any Linux kernel has this"
reasoning as link.cpu_temp's own sysfs read.
"""

import logging

log = logging.getLogger("link.uptime")

PROC_UPTIME_PATH = "/proc/uptime"

# Same "logged at most once per process, not once per PWR request"
# reasoning as link.cpu_temp._warned_once -- missing /proc/uptime (e.g.
# developing this off a real Linux box) is expected, not worth spamming
# the log over every 3s poll.
_warned_once = False


def read_uptime_s():
    """Returns Pi #1's uptime in whole seconds since boot, or None if
    /proc/uptime can't be read (e.g. a non-Linux dev environment). Same
    best-effort "missing just means missing" degradation as
    link.cpu_temp.read_cpu_temperature_c -- cheap enough to just read
    fresh on every PWR request, no caching/background thread needed."""
    global _warned_once
    try:
        with open(PROC_UPTIME_PATH, "r") as f:
            # First field: seconds since boot (float, e.g. "12345.67
            # 54321.00" -- the second field, idle time summed across all
            # CPUs, isn't needed here).
            uptime_seconds = float(f.read().split()[0])
        return int(uptime_seconds)
    except (OSError, ValueError, IndexError) as exc:
        if not _warned_once:
            log.warning("could not read uptime from %s: %s", PROC_UPTIME_PATH, exc)
            _warned_once = True
        return None
