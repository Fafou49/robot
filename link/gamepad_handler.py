"""Generic Xbox-controller reader, shared by the two things in this repo
that need one: link/server.py (feeds a live link.robot_state.RobotState,
wired into the TCP command protocol) and motor_control/remote_control.py's
Remote class (a standalone tool with no RobotState/TCP server involved at
all, used directly by the motor_control/gps_log_on_*.py field-test
scripts). Both used to have their own separate copy of this exact
gamepad-reading logic; mutualized here (2026-09-07) into one evdev-based
implementation driven purely by callbacks, so the two consumers differ
only in what they DO with an axis change or a button press, never in how
one is read off the hardware.

Why evdev over pygame (remote_control.py's previous choice -- still
listed in requirements.txt, now otherwise unused): pygame's joystick
module needs SDL's video subsystem initialized just to read a gamepad,
which can fail in a headless session (SSH, no display) unless
SDL_VIDEODRIVER=dummy is set; evdev reads /dev/input/eventX directly, has
no display dependency, and delivers real button-press EVENTS (a clean
0->1 transition) instead of a "currently held" level the caller has to
diff against the previous frame itself. It also avoids hardcoding a
device path like /dev/input/event5 (remote_control.py's previous
approach, which breaks the day the controller re-enumerates under a
different event number) -- devices are found by capability instead.

Also exposes start_rumble()/stop_rumble() on GamepadReader (2026-09-11):
a continuous controller vibration via evdev's force-feedback (FF) API.
See the RUMBLE_EFFECT_MS/RUMBLE_REFRESH_S comment below for why this is
built out of repeated short pulses rather than one continuous effect.

UPDATE (2026-09-18) -- every vibration call site that existed before this
date has been removed: the gps_log_on_full_*.py field-test scripts used
to buzz continuously for the duration of a full-throttle/full-rotation
maneuver (strong/weak depending on live GPS fix quality, via
motor_control/gps_condition_logger.py's on_transition/on_gps_quality
hooks) -- that wiring is gone from those three scripts (the hooks
themselves still exist in gps_condition_logger.py, unused, in case a
future feature wants them again). In its place, link/server.py now uses
the NEW pulse() method below (a short, fixed-duration buzz rather than a
"for as long as a condition holds" one) to physically confirm a REAL
change during actual robot operation: a strong 0.5s pulse the moment the
live GPS fix becomes DGPS-corrected, a weak 0.5s pulse the moment it
drops back out of DGPS -- see link/gps_reader.py's on_gps_quality
callback and link/server.py's DGPS_PULSE_DURATION_S. start_rumble()/
stop_rumble()/set_intensity() (continuous) remain available for a future
caller that genuinely needs a held vibration; pulse() is what a one-shot
event like this should use instead of hand-rolling a start-then-sleep-
then-stop sequence, since that would block whichever thread calls it.

Honesty note (same caveat as link/gps_reader.py and motor_control.
motor_driver): evdev could not be installed in the sandbox this was
written in (no PyPI access there), so this was written carefully against
its documented public API (list_devices(), InputDevice, capabilities(),
read_loop(), the AbsInfo namedtuple, and for rumble: upload_effect(),
erase_effect(), the ff.Rumble/ff.Effect/ff.Trigger/ff.Replay/ff.EffectType
structures, and writing an EV_FF event to play/stop an effect) but has
NOT been run against a real controller. Run it for real on the Pi before
trusting it further than "the pure axis/PWM math is unit-tested, and the
no-controller-found path degrades without crashing". Rumble specifically
has one more unknown worth testing explicitly before relying on it
tomorrow: FF_RUMBLE support on Xbox controllers under Linux's xpad driver
is solid over USB, but can be inconsistent over Bluetooth depending on
kernel/driver version -- if start_rumble() silently does nothing on the
robot, try the controller wired for the field test rather than assuming
the code is at fault.
"""
import logging
import threading
import time

log = logging.getLogger("link.gamepad_handler")

try:
    import evdev
    from evdev import ecodes, ff
    _EVDEV_AVAILABLE = True
    _EVDEV_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover -- exercised whenever evdev
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    evdev = None
    ecodes = None
    ff = None
    _EVDEV_AVAILABLE = False
    _EVDEV_IMPORT_ERROR = exc

# Xbox controller stick axes. Names (not raw numbers) so they can be
# looked up on the real `ecodes` module at run time -- overridable
# per-instance (see GamepadReader.__init__) in case a particular
# controller/driver combination maps these differently.
#
# DEFAULT_RIGHT_Y_CODE was "ABS_RY" (the "standard" xpad-driver mapping
# for an Xbox controller's right stick) until 2026-09-11, when a report
# that the left stick worked through GamepadReader but the right one did
# nothing at all led to diagnosing it with motor_control/
# dump_gamepad_axes.py: on this project's actual controller/receiver,
# the right stick's vertical axis fires as ABS_RZ instead (a known
# real-world quirk -- ABS_RZ is the "standard" mapping's RIGHT TRIGGER
# axis, but not every controller/receiver/driver combination follows
# that convention, especially over Bluetooth or with third-party
# dongles). _read_events() only ever calls on_drive() for an event whose
# code matches exactly DEFAULT_LEFT_Y_CODE or DEFAULT_RIGHT_Y_CODE, so
# with the old "ABS_RY" value every right-stick event was silently
# dropped -- no crash, no error, just nothing happening, which is
# exactly what was reported. (An earlier fix briefly set this to "ABS_Z"
# -- the LEFT trigger's code in the standard mapping -- based on a
# misread of dump_gamepad_axes.py's output; corrected to "ABS_RZ" once
# that was caught.)
#
# If this ever needs to run with a DIFFERENT controller/receiver that
# follows the standard ABS_RY mapping after all, don't just flip this
# back blindly -- re-run dump_gamepad_axes.py on that specific hardware
# first (or pass right_y_code="ABS_RY" for just that one GamepadReader
# instance instead of changing the default for everyone).
DEFAULT_LEFT_Y_CODE = "ABS_Y"
DEFAULT_RIGHT_Y_CODE = "ABS_RZ"

# Which evdev EV_KEY codes _find_device() (and dump_gamepad_buttons.py's
# own duplicated copy of the same predicate) accepts as "this looks like
# a gamepad" -- see BUTTON MAPPING just below for the two real-world
# families this covers. BTN_A is the modern "gamepad" set's first button
# (BTN_GAMEPAD is the exact same numeric code, just an alias); BTN_TRIGGER
# is the OLDER, pre-"gamepad" joystick set's equivalent (BTN_JOYSTICK is
# again the same code aliased). 2026-09-18: added BTN_TRIGGER here after
# realizing device discovery used to require BTN_A specifically -- a
# controller/receiver that reports itself entirely under the older
# joystick set (plausible, see BUTTON MAPPING below) would then never be
# found AT ALL, which silently disables every single button, START
# included, and looks exactly like "BTN_START doesn't power off the Pi"
# even though the whole controller is plugged in, powered, and working.
# This is a genuine, real bug fix on top of the two BUTTON MAPPING
# hypotheses below (which only explain individual buttons swapping, not
# the controller failing to be found in the first place) -- but like
# them, it can only be confirmed by actually running
# motor_control.dump_gamepad_buttons on the real hardware.
GAMEPAD_IDENTIFYING_BUTTONS = ("BTN_A", "BTN_TRIGGER")

# BUTTON MAPPING -- a real-world gotcha worth understanding before
# touching robot_state_button_handler() below (2026-09-18, after a field
# report that pressing the controller's "Y" button behaves as if "A" had
# been pressed instead): evdev's BTN_A/BTN_B/BTN_X/BTN_Y are not
# standalone codes, they are ALIASES the Linux kernel defines on top of a
# positional set -- BTN_A == BTN_SOUTH, BTN_B == BTN_EAST, BTN_X ==
# BTN_NORTH, BTN_Y == BTN_WEST (see linux/input-event-codes.h). On a
# genuine Xbox controller, physical Y is in the NORTH position and X is
# WEST -- i.e. the kernel's own naming has X and Y's "standard" position
# aliases backwards relative to Microsoft's own physical layout. Most
# drivers/receivers compensate for this so BTN_X/BTN_Y still line up with
# the printed labels, but not every third-party dongle or generic HID
# driver does -- and a driver that instead exposes this controller under
# the OLDER, pre-"gamepad" Linux joystick event set entirely (BTN_TRIGGER,
# BTN_THUMB, BTN_TOP, BTN_BASE, ...) is common for cheap receivers too,
# see motor_control/dump_gamepad_buttons.py's KNOWN_BUTTON_NAMES for that
# whole set. Both are real, plausible explanations for "Y behaves like A"
# on this project's actual hardware -- there is no way to tell which
# (if either) applies without running that diagnostic against the real
# controller; guessing and hardcoding a "fix" without it risks trading one
# wrong mapping for another. robot_state_button_handler()'s arm_auto_btn/
# record_btn/save_waypoint_btn/snapshot_btn/shutdown_btn/stop_btn
# parameters (and the matching GAMEPAD_*_BTN environment variables
# link/server.py reads) exist precisely so the real mapping, once known,
# can be set without touching this file at all.

PWM_SCALE = 255          # matches link/robot_state.py's PWM_MIN/PWM_MAX

# Normalized (-1..1) stick movement below this counts as centered.
#
# Widened from 0.08 (~20/255 raw PWM units) to 30/255 (2026-09-19), after a
# field report that the gamepad's BTN_A ("arm AUTO") never seems to actually
# arm AUTO mode. Likely root cause, once robot_state_drive_handler() and
# GamepadReader._read_events() are read together: _read_events() calls
# on_drive() on EVERY EV_ABS event, including the continuous analog noise a
# centered, untouched stick still produces -- not just a deliberate push.
# robot_state_drive_handler()'s _on_drive() treats ANY nonzero PWM as a
# "genuine push" and immediately forces state.set_mode("MANUAL") before
# anything else runs. With the previous ~20-unit deadzone, this specific
# controller's idle-stick noise (reported to reach up to +/-30 raw PWM
# units) was enough to clear it on essentially every ABS event -- so within
# milliseconds of BTN_A calling state.set_mode("AUTO"), the next noise event
# forced it straight back to MANUAL. ABS events fire continuously (several
# times a second even at rest), so this is indistinguishable from "pressing
# A does nothing" -- there was nothing wrong with the button itself.
#
# Widening the deadzone to cover +/-30 raw units means that noise band now
# normalizes to _pwm_from_axis() == 0 in the first place, so it never
# reaches on_drive()/_on_drive() as a "push" at all, and AUTO mode stays
# armed until an actual stick movement (or another mode change) ends it.
# If a future controller reports even noisier idle sticks, re-run
# motor_control/dump_gamepad_axes.py on it and raise this further rather
# than guessing.
AXIS_DEADZONE = 30 / PWM_SCALE  # ~0.1176 normalized (30 raw PWM units)
RETRY_INTERVAL_S = 5.0   # how often to re-scan for a controller if none is found/it disconnects

# Rumble (force feedback): Linux's FF API plays short timed "effects" (an
# effect is uploaded once, then told to play N times), not a plain on/off
# switch -- there's no documented "vibrate forever" value that works the
# same way across drivers. So a continuous-feeling vibration is built here
# out of repeated short pulses instead: RUMBLE_EFFECT_MS is how long each
# individual pulse lasts, RUMBLE_REFRESH_S is how often a fresh one is
# triggered while rumbling is active. Refresh happens well before the
# previous pulse ends (400ms pulse, retriggered every 250ms) so there's no
# gap the driver could feel as the vibration stopping and restarting.
RUMBLE_EFFECT_MS = 400
RUMBLE_REFRESH_S = 0.25
RUMBLE_IDLE_POLL_S = 0.5  # how often _rumble_loop rechecks for a controller while none is connected yet
RUMBLE_ERROR_LOG_INTERVAL_S = 5.0  # min seconds between repeated "rumble failed" warnings, so a persistent failure doesn't spam the log every RUMBLE_REFRESH_S (0.25s)

# Two rumble intensities (2026-09-11): a strong buzz and a clearly weaker
# one, so the driver can feel a difference (e.g. GPS fix quality, see
# motor_control/gps_condition_logger.py's on_gps_quality) without it being
# a subtle change on a controller motor that only really has "off" and
# "on-ish" to begin with. WEAK is deliberately still well above the
# lowest magnitudes (which risk being imperceptible on some controllers)
# -- roughly a quarter of full strength.
RUMBLE_STRONG_MAGNITUDE = 0xFFFF
RUMBLE_WEAK_MAGNITUDE = 0x4000


def _normalize_axis(raw_value, info_min, info_max):
    """Maps a raw evdev ABS event value onto -1.0..1.0, centered on the
    middle of THIS controller's own reported range. Deliberately does not
    assume a fixed range like -32768..32767 -- unlike pygame (which
    always normalizes to -1..1 itself), evdev reports whatever range the
    specific device/driver combination declares, found via
    device.capabilities(absinfo=True)."""
    center = (info_max + info_min) / 2
    half_range = (info_max - info_min) / 2
    if half_range == 0:
        return 0.0
    value = (raw_value - center) / half_range
    return max(-1.0, min(1.0, value))


def _pwm_from_axis(normalized, deadzone=AXIS_DEADZONE, scale=PWM_SCALE):
    """Stick pushed away from center -> motor PWM (-scale..scale), with a
    dead zone around center so small analog noise doesn't produce a tiny
    nonzero duty cycle. Sign is inverted: evdev (like the SDL/pygame
    convention remote_control.py's previous code relied on) reports a
    stick pushed UP/forward as a NEGATIVE Y value -- this flips it so
    "forward" maps to positive PWM, matching link/robot_state.py's DRV
    convention."""
    if abs(normalized) < deadzone:
        return 0
    return int(round(-normalized * scale))


def _rumble_effect(duration_ms, magnitude):
    """Builds one FF_RUMBLE evdev Effect at the given magnitude (0..0xFFFF,
    typically RUMBLE_STRONG_MAGNITUDE or RUMBLE_WEAK_MAGNITUDE), applied to
    both motors (the strong low-frequency one and the weak high-frequency
    one most Xbox-style controllers have) -- both set the same so the two
    intensity levels this module exposes are unambiguous, not a blend of
    the two motors' different feel. Kept standalone (rather than inlined
    in _rumble_loop) so it's the one part of the rumble feature that's
    pure/constructible without a real device, same reasoning as
    _normalize_axis/_pwm_from_axis above."""
    rumble = ff.Rumble(strong_magnitude=magnitude, weak_magnitude=magnitude)
    return ff.Effect(
        ecodes.FF_RUMBLE, -1, 0,
        ff.Trigger(0, 0),
        ff.Replay(duration_ms, 0),
        ff.EffectType(ff_rumble_effect=rumble),
    )


def _supports_ff_rumble(device):
    """Pure predicate (2026-09-11, added after a "vibrations don't work"
    report): does this evdev device advertise FF_RUMBLE support at all?
    Checked by _rumble_loop below on every new device it targets, so a
    controller/receiver/connection that genuinely lacks force-feedback
    support -- a real possibility this module's docstring already flagged
    for Bluetooth -- logs ONE clear warning instead of silently retrying
    an upload_effect() that can never succeed. Takes the device itself
    (not a bare capabilities dict) so callers don't need to know evdev's
    capabilities() shape; kept standalone so it's unit-testable against a
    fake device with a plain capabilities() method, no real hardware or
    evdev installation required."""
    capabilities = device.capabilities()
    return (
        ecodes.EV_FF in capabilities
        and ecodes.FF_RUMBLE in capabilities[ecodes.EV_FF]
    )


class GamepadReader:
    """Finds an Xbox-style controller and reads it forever, either in a
    background thread (start(), used by link/server.py, whose main thread
    is busy serving TCP connections) or in the calling thread
    (run_blocking(), used by motor_control/remote_control.py's Remote,
    which has always blocked the whole script this way).

    Two axes (left/right stick Y) call on_drive(left_pwm, right_pwm)
    whenever either one changes. Every button press AND release calls
    on_button(code, pressed) with the raw evdev key code (e.g.
    evdev.ecodes.BTN_A) -- callers that only care about presses just
    check `if pressed`.

    Also exposes start_rumble()/set_intensity()/stop_rumble() for a
    continuous controller vibration (see RUMBLE_EFFECT_MS/RUMBLE_REFRESH_S
    and RUMBLE_STRONG_MAGNITUDE/RUMBLE_WEAK_MAGNITUDE above, and
    _rumble_loop below) -- used by the gps_log_on_full_*.py field-test
    scripts to buzz the controller for as long as a GPS-logging condition
    is active (strong for a DGPS-corrected fix, weak otherwise -- see
    motor_control/gps_condition_logger.py's on_gps_quality), so the
    driver gets physical confirmation, including fix quality, without
    looking at a screen. All three methods are no-ops (never raise) with
    no controller connected, matching this class's existing "degrade
    without crashing" convention."""

    def __init__(self, on_drive, on_button=None, device_path=None,
                 left_y_code=DEFAULT_LEFT_Y_CODE, right_y_code=DEFAULT_RIGHT_Y_CODE,
                 retry_interval=RETRY_INTERVAL_S):
        self.on_drive = on_drive
        self.on_button = on_button or (lambda code, pressed: None)
        self.device_path = device_path
        self.left_y_code = left_y_code
        self.right_y_code = right_y_code
        self.retry_interval = retry_interval
        self._running = False
        self._left_norm = 0.0
        self._right_norm = 0.0
        # Rumble state -- see start_rumble()/stop_rumble()/_rumble_loop().
        # _device is set to the currently-connected InputDevice by
        # _read_events() below (None whenever no controller is connected,
        # including at startup and during a reconnect), so the rumble
        # thread always sends effects to whichever device is actually
        # live right now instead of a possibly-stale handle.
        self._device = None
        self._rumble_active = False
        self._rumble_strong = True  # target intensity for the current/next pulse
        self._rumble_lock = threading.Lock()
        self._rumble_thread = None
        # pulse()'s pending auto-stop timer, if any -- see pulse() below.
        self._pulse_timer = None

    def start(self):
        """Starts reading in a background daemon thread."""
        thread = threading.Thread(target=self.run_blocking, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def run_blocking(self):
        """Same loop as start(), but runs in the calling thread. Retries
        device discovery every `retry_interval` seconds for as long as no
        suitable controller is found, and again if one disconnects mid-
        session -- so plugging the controller in after this has already
        started still works, and unplugging it doesn't crash whatever
        called this (link/server.py's whole control server, or a
        gps_log_on_*.py script)."""
        self._running = True
        if not _EVDEV_AVAILABLE:
            log.warning(
                "evdev not installed (%s) -- gamepad disabled, no joystick/button "
                "input will reach the robot. Run `pip install -r requirements.txt` "
                "on the Pi to enable it.",
                _EVDEV_IMPORT_ERROR,
            )
            return

        while self._running:
            device = self._find_device()
            if device is None:
                log.warning(
                    "no gamepad found (looked for a device exposing an absolute "
                    "axis and BTN_A) -- retrying in %ss. Plug in the Xbox controller.",
                    self.retry_interval,
                )
                time.sleep(self.retry_interval)
                continue

            log.info("gamepad connected: %s (%s)", device.name, device.path)
            try:
                self._read_events(device)
            except OSError as exc:
                # Most common real-world cause: the controller was
                # unplugged mid-session -- reads start failing (ENODEV).
                # Go back to scanning rather than crashing the whole
                # thread (and, in link/server.py's case, the whole
                # control server) over a disconnect that could just be
                # someone changing batteries.
                log.warning("gamepad disconnected (%s) -- rescanning.", exc)

    def _find_device(self):
        if self.device_path:
            try:
                return evdev.InputDevice(self.device_path)
            except OSError as exc:
                log.warning("configured gamepad device %s not available (%s)",
                            self.device_path, exc)
                return None

        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
            except OSError:
                continue
            capabilities = device.capabilities()
            has_abs = ecodes.EV_ABS in capabilities
            has_gamepad_button = ecodes.EV_KEY in capabilities and any(
                getattr(ecodes, name, None) in capabilities[ecodes.EV_KEY]
                for name in GAMEPAD_IDENTIFYING_BUTTONS
            )
            if has_abs and has_gamepad_button:
                return device
        return None

    def _read_events(self, device):
        self._device = device  # rumble thread now targets this device
        try:
            abs_info = dict(device.capabilities(absinfo=True).get(ecodes.EV_ABS, []))
            left_code = getattr(ecodes, self.left_y_code)
            right_code = getattr(ecodes, self.right_y_code)

            for event in device.read_loop():
                if not self._running:
                    return

                if event.type == ecodes.EV_ABS and event.code in (left_code, right_code):
                    info = abs_info.get(event.code)
                    if info is None:
                        continue
                    normalized = _normalize_axis(event.value, info.min, info.max)
                    if event.code == left_code:
                        self._left_norm = normalized
                    else:
                        self._right_norm = normalized
                    self.on_drive(
                        _pwm_from_axis(self._left_norm),
                        _pwm_from_axis(self._right_norm),
                    )

                elif event.type == ecodes.EV_KEY and event.value in (0, 1):
                    # value 1 = pressed, 0 = released -- the hold-repeat
                    # value (2) some drivers send is deliberately ignored
                    # here, it's not a new press.
                    self.on_button(event.code, event.value == 1)
        finally:
            # Controller gone (loop returned or raised OSError) -- clear
            # the handle so _rumble_loop stops targeting a stale/closed
            # device and just waits for the next reconnect instead.
            self._device = None

    def start_rumble(self, strong=True):
        """Starts a continuous vibration on the connected controller,
        until stop_rumble() is called. `strong` picks the initial
        intensity (True = RUMBLE_STRONG_MAGNITUDE, False =
        RUMBLE_WEAK_MAGNITUDE) -- change it later without stopping the
        vibration via set_intensity(). Safe to call repeatedly: if
        already rumbling, this just updates the intensity (same as
        calling set_intensity()) instead of starting a second thread.
        Safe to call with no controller connected -- the background
        thread it starts just waits for one to show up in self._device,
        same retry spirit as run_blocking() itself."""
        with self._rumble_lock:
            self._rumble_strong = strong
            if self._rumble_active:
                return
            self._rumble_active = True
            self._rumble_thread = threading.Thread(target=self._rumble_loop, daemon=True)
            self._rumble_thread.start()

    def set_intensity(self, strong):
        """Changes the intensity of an already-running vibration (True =
        strong, False = weak) without stopping and restarting it -- the
        next pulse _rumble_loop triggers picks it up. A no-op (in the
        sense that it only records the new target for whenever rumbling
        next starts) if nothing is currently rumbling; call start_rumble()
        to actually begin vibrating."""
        with self._rumble_lock:
            self._rumble_strong = strong

    def stop_rumble(self):
        """Stops a vibration started by start_rumble(). Safe to call even
        if nothing is currently rumbling. The actual effect teardown
        (stopping playback, erasing the uploaded effect) happens inside
        _rumble_loop itself once it notices this flag went False -- only
        that thread ever touches the device's force-feedback state, so
        there's no race between it uploading/playing a pulse and this
        erasing it out from under it."""
        with self._rumble_lock:
            self._rumble_active = False

    def pulse(self, strong, duration_s):
        """One-shot vibration (2026-09-18): starts rumbling at `strong`
        intensity and reliably stops it again after `duration_s` seconds,
        without blocking the calling thread -- unlike hand-rolling
        start_rumble() + time.sleep() + stop_rumble(), which would freeze
        whichever thread calls it (here, link/gps_reader.py's GPSReader
        background thread, which needs to keep reading GPS fixes while
        this plays out). Used by link/server.py to physically confirm a
        live GPS fix quality change (strong on gaining DGPS, weak on
        losing it) the instant it happens.

        If a previous pulse's stop timer is still pending when this is
        called again (e.g. the fix quality flaps quickly), that timer is
        cancelled and replaced by this call's -- so the vibration stops
        `duration_s` after the LATEST pulse started, not the first one,
        and two overlapping pulses never fight over turning each other
        off early. Safe to call with no controller connected, same
        "degrade without crashing" convention as start_rumble()/
        stop_rumble() themselves."""
        with self._rumble_lock:
            if self._pulse_timer is not None:
                self._pulse_timer.cancel()
            timer = threading.Timer(duration_s, self.stop_rumble)
            timer.daemon = True
            self._pulse_timer = timer
            timer.start()
        # Outside the lock -- start_rumble() takes it itself, and it also
        # runs perfectly well even if the exact same lock re-entered here
        # (it doesn't, but this ordering keeps that always trivially true).
        self.start_rumble(strong=strong)

    def _rumble_loop(self):
        """Runs in its own thread for the lifetime of one start_rumble()/
        stop_rumble() cycle: re-uploads and replays a short RUMBLE_EFFECT_MS
        pulse every RUMBLE_REFRESH_S seconds for as long as _rumble_active
        stays True (see the module-level comment on those two constants
        for why a repeated pulse is used instead of one long effect).

        Also watches self._rumble_strong (settable live via
        set_intensity()) against `effect_strong`, the intensity the
        currently-uploaded effect_id was actually built with -- an evdev
        FF effect's magnitude is baked in at upload time, so a change in
        intensity means erasing the old effect and uploading a new one at
        the new magnitude, not just writing a different value.

        Tracks the device it uploaded the current effect to (`device`,
        local to this method) separately from self._device (which can
        change under it at any time on a reconnect) -- if they differ, or
        a write/upload fails with OSError (device unplugged mid-pulse),
        the stale effect id is dropped and a fresh one is uploaded against
        whatever device is current next time round, instead of crashing
        this thread over a disconnect.

        2026-09-11, after a "vibrations don't work" field report: this
        used to swallow every OSError silently and never checked whether
        the device supports FF_RUMBLE at all, so a controller/connection
        that genuinely lacks force-feedback support (plausible here, see
        this module's docstring honesty note on Bluetooth FF reliability)
        produced total, permanent silence with zero diagnostic trail --
        indistinguishable from "nothing to report". Now it checks
        _supports_ff_rumble() once per device (it can't change while the
        same device stays connected, so re-checking every pulse would
        just waste a syscall) and logs a warning, once per connection,
        instead of retrying an upload that can never succeed; genuine
        OSErrors are also logged now, rate-limited to
        RUMBLE_ERROR_LOG_INTERVAL_S so a persistent failure doesn't spam
        the log four times a second."""
        if not _EVDEV_AVAILABLE:
            return
        device = None
        effect_id = None
        effect_strong = None
        device_supports_ff = False
        warned_unsupported = False
        last_error_log = 0.0
        try:
            while True:
                with self._rumble_lock:
                    if not self._rumble_active:
                        break
                    target_strong = self._rumble_strong
                current_device = self._device
                if current_device is None:
                    time.sleep(RUMBLE_IDLE_POLL_S)
                    continue
                if current_device is not device:
                    device = current_device
                    effect_id = None
                    effect_strong = None
                    warned_unsupported = False
                    try:
                        device_supports_ff = _supports_ff_rumble(device)
                    except OSError:
                        device_supports_ff = False

                if not device_supports_ff:
                    if not warned_unsupported:
                        log.warning(
                            "gamepad %s does not advertise FF_RUMBLE support -- "
                            "vibration cannot work on this controller/connection. "
                            "FF_RUMBLE over Bluetooth is sometimes unsupported "
                            "even when the same controller works fine wired -- "
                            "try it over USB for the field test. (This warning "
                            "is only logged once per connection.)",
                            getattr(device, "path", device),
                        )
                        warned_unsupported = True
                    time.sleep(RUMBLE_REFRESH_S)
                    continue

                try:
                    if effect_id is None or target_strong != effect_strong:
                        if effect_id is not None:
                            device.erase_effect(effect_id)
                        magnitude = RUMBLE_STRONG_MAGNITUDE if target_strong else RUMBLE_WEAK_MAGNITUDE
                        effect_id = device.upload_effect(_rumble_effect(RUMBLE_EFFECT_MS, magnitude))
                        effect_strong = target_strong
                    device.write(ecodes.EV_FF, effect_id, 1)
                except OSError as exc:
                    now = time.monotonic()
                    if now - last_error_log >= RUMBLE_ERROR_LOG_INTERVAL_S:
                        log.warning("gamepad rumble failed (%s) -- will keep retrying.", exc)
                        last_error_log = now
                    device = None
                    effect_id = None
                    effect_strong = None
                time.sleep(RUMBLE_REFRESH_S)
        finally:
            if device is not None and effect_id is not None:
                try:
                    device.write(ecodes.EV_FF, effect_id, 0)
                    device.erase_effect(effect_id)
                except OSError:
                    pass


def robot_state_button_handler(state, on_shutdown=None,
                                arm_auto_btn="BTN_A", record_btn="BTN_B",
                                save_waypoint_btn="BTN_X", snapshot_btn="BTN_Y",
                                shutdown_btn="BTN_START", stop_btn=None):
    """Returns an on_button callback wiring the gamepad's buttons to a
    link.robot_state.RobotState -- used by link/server.py.

    2026-09-18 remap -- the right-hand button cluster now matches how the
    robot is actually driven day-to-day, not the earlier NAV-focused set:
    - arm_auto_btn (default BTN_A): re-arms AUTO mode -- moved here from
      BTN_Y (which itself had moved here from BTN_A on 2026-09-12; see
      git history for both).
    - record_btn (default BTN_B): toggles video recording (CAM,REC_START/
      REC_STOP -- genuinely implemented as of this date, see
      camera/stream_server.py's VideoRecorder and link/robot_state.py's
      camera_command()). "record_btn" pressed while state.is_recording is
      False sends REC_START; pressed again, REC_STOP -- so one button
      does both, no separate "stop recording" button needed. "si la
      caméra est présente": if the camera script isn't running, or has no
      frame yet, camera_command() raises CommandError, which is caught
      and logged here rather than propagated -- see the try/except in
      _on_button below, needed because an uncaught exception here would
      silently kill the whole GamepadReader background thread, taking
      every OTHER button and both sticks down with it.
    - save_waypoint_btn (default BTN_X): appends the robot's current GPS
      fix to a waypoints file (RobotState.save_waypoint()) -- same
      failure-must-not-crash-the-thread reasoning as record_btn above
      (raises CommandError with no GPS fix yet).
    - snapshot_btn (default BTN_Y): camera snapshot (CAM,SNAP) -- the one
      button here that already existed as a working command (via the
      website's console), just newly reachable from the gamepad too.
      Same try/except reasoning as record_btn/save_waypoint_btn.
    - stop_btn (default None, i.e. NO button bound): previously BTN_B, a
      full stop (state.stop() -- motors zeroed, mode back to IDLE, any
      route cleared), same as the STP sentence. Removed from the default
      right-cluster mapping above because BTN_B now records video
      instead -- this is a DELIBERATE simplification, not an oversight,
      made explicit here because it removes a physical emergency-stop
      button from the controller. The remaining safety nets: any genuine
      stick push always hands control back to MANUAL immediately, even
      mid-AUTO (robot_state_drive_handler() below), and the website keeps
      its own "STOP" button (sends STP instantly, no console step). If a
      dedicated gamepad stop button is wanted after all, pass a name here
      (or set GAMEPAD_STOP_BTN in link/server.py's environment) -- a
      shoulder button/bumper (BTN_TL/BTN_TR), both unused by this mapping,
      would be a reasonable choice.
    - shutdown_btn (default BTN_START, unchanged): still stops the robot
      first, then calls `on_shutdown` if given (link/server.py wires this
      to actually power off the Raspberry Pi -- see
      ControlServer._shutdown_pi() there).

    Every one of the six parameters above is a NAME on evdev.ecodes,
    resolved once here rather than hardcoded (2026-09-12, extended
    2026-09-18) -- because this project's actual controller/receiver may
    not report a given physical button under the evdev code its label
    would suggest (see this module's "BUTTON MAPPING" comment above
    DEFAULT_LEFT_Y_CODE, written after a field report that pressing "Y"
    behaves like "A"). Passing a name here that isn't a real evdev.ecodes
    attribute logs one clear warning instead of a silent no-op reachable
    only by reading this source. Run `python3 -m
    motor_control.dump_gamepad_buttons` on the Pi against the real
    controller to find out what each physical button's ACTUAL code is,
    then either pass the right names here or, without touching code, set
    GAMEPAD_ARM_AUTO_BTN / GAMEPAD_RECORD_BTN / GAMEPAD_SAVE_WAYPOINT_BTN
    / GAMEPAD_SNAPSHOT_BTN / GAMEPAD_SHUTDOWN_BTN / GAMEPAD_STOP_BTN,
    which link/server.py reads for exactly this purpose.

    The gamepad is always listened to and MANUAL always wins by default
    (see robot_state_drive_handler() below: any genuine stick push
    switches back to MANUAL immediately, even mid-AUTO-drive) -- there is
    no MOD button on the website, since a physical operator should never
    need one to take control back.

    arm_auto_btn only actually arms AUTO if there is something to drive
    toward (state.has_nav_target(): a NAV point from the website, or an
    uploaded GPS route) -- pressing it with neither set is a silent
    no-op rather than switching into a mode that would just sit there
    computing nothing on every GPS fix (see link/robot_state.py's
    _autonomous_pwm_locked()). Once armed, link/autopilot.py really
    drives the robot toward nav_target on every GPS fix.

    Taking the joystick back over from an active AUTO drive is handled
    separately, in robot_state_drive_handler() below -- not here, since
    that needs to distinguish an actual stick push from the idle/centered
    stick's own analog noise, which button presses don't have."""
    # Resolved once here (not on every call) -- guarded by _EVDEV_AVAILABLE
    # since ecodes is None when evdev isn't installed at all (see the
    # top of this module); _on_button below already short-circuits on
    # that same flag before ever touching these, so None here is a safe
    # placeholder rather than a real getattr(None, ...) crash risk.
    # stop_btn is None by default (no button bound at all, see docstring)
    # -- resolved the same way as the rest so a name IS still honored if
    # one is passed, but None never triggers the "unknown name" warning
    # below (it's a deliberate absence, not a typo).
    button_names = {
        "arm_auto": arm_auto_btn,
        "record": record_btn,
        "save_waypoint": save_waypoint_btn,
        "snapshot": snapshot_btn,
        "shutdown": shutdown_btn,
        "stop": stop_btn,
    }
    codes = {}
    for action, name in button_names.items():
        if name is None:
            codes[action] = None
            continue
        resolved = getattr(ecodes, name, None) if _EVDEV_AVAILABLE else None
        codes[action] = resolved
        if _EVDEV_AVAILABLE and resolved is None:
            log.warning(
                "gamepad button name %r (for %s) is not a known evdev code "
                "-- that action will never trigger. Check the GAMEPAD_*_BTN "
                "environment variables (or this call's matching parameter) "
                "against `python3 -m motor_control.dump_gamepad_buttons`'s "
                "output.",
                name, action,
            )

    # Two actions accidentally sharing one evdev code (a copy-paste in the
    # GAMEPAD_*_BTN environment variables, most likely) would silently
    # shadow one of them -- only the first matching `if`/`elif` branch in
    # _on_button below would ever fire for that code. Worth one clear
    # warning at setup time rather than a confusing "button does nothing"
    # report later.
    if _EVDEV_AVAILABLE:
        seen = {}
        for action, code in codes.items():
            if code is None:
                continue
            if code in seen:
                log.warning(
                    "gamepad buttons %r (%s) and %r (%s) resolve to the SAME "
                    "evdev code -- only %s will ever trigger for that button. "
                    "Check the GAMEPAD_*_BTN environment variables for a "
                    "duplicate.",
                    button_names[seen[code]], seen[code], button_names[action], action, seen[code],
                )
            else:
                seen[code] = action

    def _on_button(code, pressed):
        if not pressed or not _EVDEV_AVAILABLE:
            return
        if code == codes["arm_auto"]:
            if state.has_nav_target():
                state.set_mode("AUTO")
            else:
                log.info(
                    "%s pressed but no nav target/route is set (send NAV "
                    "from the website or upload a GPS route first) -- "
                    "staying in the current mode.",
                    arm_auto_btn,
                )
        elif code == codes["record"]:
            action = "REC_STOP" if state.is_recording else "REC_START"
            try:
                state.camera_command(action)
            except Exception as exc:
                # Broad except deliberately: camera_command() can raise for
                # plenty of ordinary reasons (camera script not running, no
                # frame yet) and an uncaught exception here would kill this
                # whole GamepadReader thread -- taking every other button
                # and both sticks down with it, not just recording.
                log.warning(
                    "%s pressed (%s) but the camera didn't cooperate (%s) "
                    "-- is `python3 -m camera` running?",
                    record_btn, action, exc,
                )
        elif code == codes["save_waypoint"]:
            try:
                path = state.save_waypoint()
                log.info("%s pressed -- saved current GPS fix to %s", save_waypoint_btn, path)
            except Exception as exc:
                log.warning(
                    "%s pressed but the waypoint could not be saved (%s) "
                    "-- is there a GPS fix yet?",
                    save_waypoint_btn, exc,
                )
        elif code == codes["snapshot"]:
            try:
                state.camera_command("SNAP")
            except Exception as exc:
                log.warning(
                    "%s pressed but the camera snapshot failed (%s) -- "
                    "is `python3 -m camera` running?",
                    snapshot_btn, exc,
                )
        elif code == codes["stop"]:
            state.stop()
        elif code == codes["shutdown"]:
            state.stop()
            if on_shutdown is not None:
                on_shutdown()
    return _on_button


def robot_state_drive_handler(state):
    """Returns an on_drive callback wiring the gamepad's sticks to a
    link.robot_state.RobotState -- used by link/server.py. A genuine
    stick push (nonzero PWM on either side) switches back to MANUAL mode
    first, then always drives -- a physical operator can always take
    back control from an active AUTO drive by touching a stick, same
    "manual override always wins" convention this project already uses
    for NAV/STP clearing an in-progress route.

    2026-09-12 bug fix -- an idle/centered stick no longer reaches
    drive() at all UNLESS the robot is already in MANUAL mode: GamepadReader
    calls on_drive on every ABS event, including the analog noise/jitter
    a centered, untouched stick can still produce -- _pwm_from_axis maps
    those to (0, 0). The nonzero gate below (unchanged since introduction)
    already stopped that idle noise from FORCING MANUAL mode, but it used
    to still call state.drive(0, 0) unconditionally regardless of mode --
    and RobotState.drive() always zeroes left_pwm/right_pwm and tells the
    motor driver to stop, with no regard for which mode is active. In
    AUTO mode, that meant nearly every idle-stick event (they fire
    continuously, several times a second, from ordinary stick noise) was
    silently overwriting whatever PWM update_gps_fix()'s autopilot tick
    had just computed a moment before -- the robot would arm AUTO (mode
    really did become "AUTO") but never actually move, which is exactly
    what got reported as "AUTO mode never engages". Routing idle (0, 0)
    through state.is_manual() first means it's now a no-op in AUTO/IDLE
    (autopilot's own PWM output is left alone) while still zeroing the
    motors normally when a stick is released after a genuine MANUAL
    drive."""
    def _on_drive(left_pwm, right_pwm):
        is_push = left_pwm != 0 or right_pwm != 0
        if is_push:
            state.set_mode("MANUAL")
            state.drive(left_pwm, right_pwm)
        elif state.is_manual():
            state.drive(left_pwm, right_pwm)
    return _on_drive
