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
a continuous controller vibration via evdev's force-feedback (FF) API,
used by the gps_log_on_full_*.py field-test scripts so the driver gets a
physical "recording right now" cue without watching a screen. See the
RUMBLE_EFFECT_MS/RUMBLE_REFRESH_S comment below for why this is built out
of repeated short pulses rather than one continuous effect.

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

AXIS_DEADZONE = 0.08     # normalized (-1..1) stick movement below this counts as centered
PWM_SCALE = 255          # matches link/robot_state.py's PWM_MIN/PWM_MAX
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
            has_a_button = (
                ecodes.EV_KEY in capabilities
                and ecodes.BTN_A in capabilities[ecodes.EV_KEY]
            )
            if has_abs and has_a_button:
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
                                arm_auto_btn="BTN_Y", stop_btn="BTN_B",
                                shutdown_btn="BTN_START"):
    """Returns an on_button callback wiring the gamepad's buttons to a
    link.robot_state.RobotState -- used by link/server.py.

    arm_auto_btn/stop_btn/shutdown_btn (2026-09-12, added after a report
    that BTN_Y and BTN_START "don't work as expected" on this project's
    actual controller/receiver): names of attributes on evdev.ecodes,
    resolved once here rather than hardcoded, in case this specific
    hardware reports those two buttons under different codes -- the
    exact same class of real-world quirk this project already hit once
    for the right stick's axis (see this module's DEFAULT_RIGHT_Y_CODE
    and its docstring). Run `python3 -m motor_control.dump_gamepad_buttons`
    on the Pi to find out empirically what the real names are, then
    either pass them here or, without touching code, set the
    GAMEPAD_ARM_AUTO_BTN / GAMEPAD_STOP_BTN / GAMEPAD_SHUTDOWN_BTN
    environment variables link/server.py reads for exactly this purpose.
    Defaults match this function's previous hardcoded behavior exactly,
    so nothing changes for a controller that does report BTN_Y/BTN_B/
    BTN_START normally.

    The gamepad is always listened to and MANUAL always wins by default
    (see robot_state_drive_handler() below: any genuine stick push
    switches back to MANUAL immediately, even mid-AUTO-drive) -- there is
    no MOD button on the website anymore, since a physical operator
    should never need one to take control back.

    BTN_Y re-arms AUTO mode -- this is the controller's "go to the
    target" button (moved here from BTN_A on 2026-09-12, alongside the
    /control website's redesign; BTN_A is unbound as of that change).
    It only actually arms AUTO if there is something to drive toward
    (state.has_nav_target(): a NAV point from the website, or an
    uploaded GPS route) -- pressing it with neither set is a silent
    no-op rather than switching into a mode that would just sit there
    computing nothing on every GPS fix (see link/robot_state.py's
    _autonomous_pwm_locked()). Once armed, link/autopilot.py really
    drives the robot toward nav_target on every GPS fix.

    BTN_B stops the robot -- same as the STP sentence (full stop: motors
    zeroed, mode back to IDLE, any in-progress route cleared), not just a
    hand-back to MANUAL, since a plain mode switch on its own doesn't
    cancel whatever nav_target/route is still armed.

    BTN_START (2026-09-12: repurposed from "a second emergency stop,
    redundant with BTN_B" -- see git history for the previous behavior)
    now stops the robot the same way BTN_B does, AND additionally calls
    `on_shutdown` (if given): link/server.py wires this to cleanly stop
    this robot's own control scripts and power off the Raspberry Pi
    itself -- see ControlServer._shutdown_pi() there, and this project's
    README ("Pilotage moteur et manette") for the sudoers setup that
    requires. `on_shutdown` is optional (None by default) so every other
    caller of this function (there are none today, but this keeps the
    bar low for one later, e.g. a test) doesn't need to care about it.

    Taking the joystick back over from an active AUTO drive is handled
    separately, in robot_state_drive_handler() below -- not here, since
    that needs to distinguish an actual stick push from the idle/centered
    stick's own analog noise, which button presses don't have."""
    # Resolved once here (not on every call) -- guarded by _EVDEV_AVAILABLE
    # since ecodes is None when evdev isn't installed at all (see the
    # top of this module); _on_button below already short-circuits on
    # that same flag before ever touching these, so None here is a safe
    # placeholder rather than a real getattr(None, ...) crash risk.
    arm_auto_code = getattr(ecodes, arm_auto_btn, None) if _EVDEV_AVAILABLE else None
    stop_code = getattr(ecodes, stop_btn, None) if _EVDEV_AVAILABLE else None
    shutdown_code = getattr(ecodes, shutdown_btn, None) if _EVDEV_AVAILABLE else None
    if _EVDEV_AVAILABLE:
        for name, resolved in ((arm_auto_btn, arm_auto_code),
                                (stop_btn, stop_code),
                                (shutdown_btn, shutdown_code)):
            if resolved is None:
                log.warning(
                    "gamepad button name %r is not a known evdev code -- "
                    "that action will never trigger. Check "
                    "GAMEPAD_ARM_AUTO_BTN/GAMEPAD_STOP_BTN/"
                    "GAMEPAD_SHUTDOWN_BTN (or this call's arm_auto_btn/"
                    "stop_btn/shutdown_btn) against `python3 -m "
                    "motor_control.dump_gamepad_buttons`'s output.",
                    name,
                )

    def _on_button(code, pressed):
        if not pressed or not _EVDEV_AVAILABLE:
            return
        if code == arm_auto_code:
            if state.has_nav_target():
                state.set_mode("AUTO")
            else:
                log.info(
                    "%s pressed but no nav target/route is set (send NAV "
                    "from the website or upload a GPS route first) -- "
                    "staying in the current mode.",
                    arm_auto_btn,
                )
        elif code == stop_code:
            state.stop()
        elif code == shutdown_code:
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
