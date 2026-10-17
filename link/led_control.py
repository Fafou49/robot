"""Turns Pi #1's two onboard status LEDs (ACT/green, PWR/red) off while
the solar panel has been reporting low voltage for a sustained stretch --
i.e. it's dark -- and back on once light returns. Explicit user request
(2026-10-10): "je souhaites que si le panneau solaire ne capte qu'une
tension inférieure à 15Volt pendant plus de 10 minutes alors les deux
Leds doivent s'éteindre et lorsque le soleil revient alors les leds
redeviennent actives" -- two LEDs glowing next to the robot all night is
exactly the kind of thing worth not having.

Runs as its own background thread (LedController below), sampling
RobotState.power_status() on a short interval and driving the LEDs
through sysfs (/sys/class/leds/<name>/{trigger,brightness}) -- the same
interface `echo ... | sudo tee /sys/class/leds/led0/trigger` from a shell
uses, just done here from Python. "led0"/"led1" (rather than the
friendlier "ACT"/"PWR" udev aliases some OS images create) is the one
naming guaranteed to exist on every Raspberry Pi OS image -- see
ACT_LED_NAME/PWR_LED_NAME below if a given Pi's `ls /sys/class/leds/`
shows something else.

Separate module/thread from link.tracer_reader (which only ever WRITES
pv_voltage into RobotState) and link.power_history (which only ever
READS it to decide whether to log) -- this is a third, independent
consumer of the same state, same "RobotState as the single shared hub,
everything else just reads or writes through it" shape already used
throughout this project (see link/robot_state.py's own module docstring).

Honesty note: /sys/class/leds/* is real Raspberry Pi hardware this was
written against carefully (the same trigger/brightness file pair used by
the project's own earlier "turn the status LED off" guidance), but could
not be exercised against an actual Pi's sysfs tree in this sandbox (no
/sys/class/leds/ here at all). _write_led_file()'s try/except around
every sysfs write means a wrong LED name or missing permission degrades
to a logged warning rather than crashing the control server -- that
degradation path IS exercised by this module's own tests (a fake/missing
sysfs root). Confirm the LED names on the real hardware with
`ls /sys/class/leds/` before relying on this, and adjust ACT_LED_NAME/
PWR_LED_NAME via the environment if they differ.
"""
import logging
import os
import threading
import time

log = logging.getLogger("link.led_control")

LEDS_SYSFS_ROOT = os.environ.get("LEDS_SYSFS_ROOT", "/sys/class/leds")

# "led0"/"led1" is the classic, always-present naming (ACT/PWR are
# udev-created aliases that don't exist on every OS image) -- overridable
# per-Pi via the environment, see this module's docstring.
ACT_LED_NAME = os.environ.get("ACT_LED_NAME", "led0")
PWR_LED_NAME = os.environ.get("PWR_LED_NAME", "led1")

# Triggers restored when the LEDs are turned back on: "mmc0" is the stock
# ACT LED behaviour on Raspberry Pi OS (blinks on SD-card activity),
# "default-on" is the stock PWR LED behaviour (steady on, only dims on
# undervoltage) -- also overridable, same reasoning as the names above.
ACT_LED_RESTORE_TRIGGER = os.environ.get("ACT_LED_RESTORE_TRIGGER", "mmc0")
PWR_LED_RESTORE_TRIGGER = os.environ.get("PWR_LED_RESTORE_TRIGGER", "default-on")

LOW_VOLTAGE_THRESHOLD_V = 15.0  # explicit user request (2026-10-10)
LOW_VOLTAGE_DURATION_S = 10 * 60  # 10 minutes, explicit user request
# Fine-grained enough that the 10-minute threshold isn't meaningfully
# delayed (worst case: ~10s late) without busy-polling RobotState.
LED_CHECK_INTERVAL_S = 10.0

_warned_once = {}  # led_name -> True, once a write to it has ever failed


def _write_led_file(led_name, filename, value):
    """Writes one sysfs file for one LED. Degrades to a one-time-per-LED
    warning (not an exception) on failure -- wrong LED name, no sysfs on
    this machine, or missing permission (this process normally runs with
    the access a Pi control service needs, but a dev machine won't have
    this path at all) -- same "best-effort hardware, never crash the
    control server over it" posture as link.gps_reader/link.tracer_reader."""
    path = f"{LEDS_SYSFS_ROOT}/{led_name}/{filename}"
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except OSError as exc:
        if not _warned_once.get(path):
            log.warning("could not write %r to %s: %s", value, path, exc)
            _warned_once[path] = True
        return False


def turn_led_off(led_name):
    """Disables whatever's driving the LED (trigger "none") AND forces
    brightness to 0 -- both steps are needed: leaving the trigger in
    place (e.g. "mmc0") would just turn the LED back on at the next SD
    card access, brightness alone isn't enough to override an active
    trigger."""
    _write_led_file(led_name, "trigger", "none")
    _write_led_file(led_name, "brightness", 0)


def turn_led_on(led_name, restore_trigger):
    """Restores the LED's normal behaviour -- not just "brightness 1" on
    its own, which would leave it permanently lit instead of resuming its
    actual activity/power-status indication."""
    _write_led_file(led_name, "trigger", restore_trigger)
    _write_led_file(led_name, "brightness", 1)


class LedController:
    """Background thread: watches RobotState.power_status()["pv_voltage"]
    and turns both onboard LEDs off once it has read continuously below
    LOW_VOLTAGE_THRESHOLD_V for LOW_VOLTAGE_DURATION_S, back on the
    moment a reading recovers above it. See this module's docstring for
    the explicit user request behind this.

    A tick while the Tracer itself is unavailable (power_status()[
    "available"] is False -- RS485 cable unplugged, Tracer unpowered)
    neither counts toward the low-voltage timer nor resets it: with no
    real voltage reading, guessing "low" would risk turning the LEDs off
    over a disconnected cable rather than an actually dark panel, and
    guessing "fine" would throw away a timer that might have been
    minutes from firing. It simply waits for a real reading either way,
    same spirit as link.power_history's system_clock_is_plausible() guard
    choosing to wait rather than act on an untrustworthy value."""

    def __init__(self, state, check_interval=LED_CHECK_INTERVAL_S,
                 threshold_v=LOW_VOLTAGE_THRESHOLD_V, duration_s=LOW_VOLTAGE_DURATION_S):
        self.state = state
        self.check_interval = check_interval
        self.threshold_v = threshold_v
        self.duration_s = duration_s
        self._running = False
        # None while the latest reading is >= threshold (or unknown);
        # set to the monotonic time the FIRST low reading was seen, once
        # one has -- monotonic rather than wall-clock time since this is
        # purely a "how long has this been going on" duration, with no
        # use for it surviving a process restart and no exposure to
        # Pi #1's own clock-plausibility problem (link.power_history).
        self._low_since = None
        # Tracks whether the LEDs are currently OFF (from this
        # controller's point of view) so _check_once() only writes to
        # sysfs on an actual transition, not on every tick.
        self._leds_off = False

    def start(self):
        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            self._check_once()
            time.sleep(self.check_interval)

    def _check_once(self):
        status = self.state.power_status()
        if not status.get("available"):
            return  # see class docstring -- a tick with no real reading
            # neither counts nor resets the timer.
        pv_voltage = status.get("pv_voltage")
        if pv_voltage is None:
            return
        now = time.monotonic()
        if pv_voltage < self.threshold_v:
            if self._low_since is None:
                self._low_since = now
            elif now - self._low_since >= self.duration_s and not self._leds_off:
                self._set_leds(off=True)
        else:
            # Any reading at or above the threshold is "the sun is back"
            # -- resets the timer immediately (no separate recovery
            # delay was requested) and turns the LEDs back on right away
            # if they were off.
            self._low_since = None
            if self._leds_off:
                self._set_leds(off=False)

    def _set_leds(self, off):
        if off:
            log.info(
                "PV voltage below %.1fV for over %.0f minutes -- turning "
                "both status LEDs off for the night",
                self.threshold_v, self.duration_s / 60,
            )
            turn_led_off(ACT_LED_NAME)
            turn_led_off(PWR_LED_NAME)
        else:
            log.info("PV voltage recovered -- turning status LEDs back on")
            turn_led_on(ACT_LED_NAME, ACT_LED_RESTORE_TRIGGER)
            turn_led_on(PWR_LED_NAME, PWR_LED_RESTORE_TRIGGER)
        self._leds_off = off
