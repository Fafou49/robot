"""Shared machinery behind the motor_control/gps_log_on_*.py scripts.

Each of those scripts drives the robot with a gamepad exactly like
remote_control.py (same Remote class, unchanged), and wants to log raw
NMEA sentences from the GPS receiver only while the two motors' duty
cycles satisfy some specific condition -- full throttle in a straight
line (gps_log_on_full_throttle.py), full-speed rotation in place
(gps_log_on_full_rotation.py), or both at once in a single run
(gps_log_on_full_maneuvers.py). Rather than duplicating the GPS-reading/
logging loop in every one of those scripts, it lives once here.

MultiConditionGPSLogger is the general case: it reads the GPS serial
port ONCE and checks every condition against each line, writing each
condition's matching lines to its own log file. This single-reader
design matters because a serial port can't reliably be read by two
independent processes/threads at once -- each would only see an
unpredictable share of the incoming lines, splitting the NMEA stream
between them instead of each seeing it whole. So gps_log_on_full_
maneuvers.py uses MultiConditionGPSLogger with two conditions instead of
running the two single-condition scripts side by side.

ConditionGPSLogger (single condition, kept for gps_log_on_full_throttle.py
and gps_log_on_full_rotation.py) is just MultiConditionGPSLogger with a
list of exactly one condition -- same public interface as before this
was generalized, no changes needed in either of those two scripts.

Same serial device/baud rate as gps/gps_parse.py and link/gps_reader.py
(the project's other two NMEA readers), so all stay in sync should the
GPS hardware ever change.

Only logs genuine NMEA GPS sentences (2026-09-11 fix -- see
_is_gps_sentence() below): GPS_DEVICE is the SAME serial port
gps/dgps_transfer.py writes RTCM/DGPS correction bytes to (to feed them
into the receiver), so running that script alongside one of the
gps_log_on_full_*.py loggers made the full_*_gps.log files pick up that
correction traffic interleaved with the GPS's own NMEA output -- a field
report (2026-09-11) that these logs contained "DGPS corrections" as well
as GPS frames, when only the GPS frames themselves were wanted. Before
this fix, every non-empty line read off the serial port during a
triggered period was logged verbatim, whatever it actually was.

Also tracks GGA fix quality (2026-09-11), same $--GGA `gps_qual` field
link/gps_reader.py already reads: DGPS_QUALITY (2, the standard NMEA
value for a differentially-corrected fix -- this project's whole point,
see gps/dgps_transfer.py and pages/rapport_rover_dgps.html) vs anything
else (no fix, a plain autonomous GPS fix, ...). Exposed via
`on_gps_quality(is_dgps)`, fired only while at least one condition is
currently being logged.

UPDATE (2026-09-18): `on_transition`/`on_gps_quality` (both below) used
to be wired up by the gps_log_on_full_*.py scripts to make the gamepad
rumble strong/weak via link.gamepad_handler.GamepadReader -- that wiring
has been removed from all three of those scripts (they no longer touch
the gamepad's rumble motor at all). Both hooks are left exactly as they
were, generic and unused by anything in this repo right now, in case a
future feature wants to react to "a condition just started/stopped" or
"the live fix's DGPS quality just changed" again -- they cost nothing to
keep and nothing here needs to change if that day comes. The actual
DGPS-quality vibration feature this project has today lives entirely
outside this field-test module: see link/gps_reader.py's GPSReader
(reads the SAME kind of GGA quality field, but on the main navigation
pipeline's live fix, not a field-test recording session) and
link/gamepad_handler.py's GamepadReader.pulse(), wired together in
link/server.py -- a short, one-shot buzz the instant fix quality changes
during real operation, not a continuous buzz tied to a maneuver like the
one this module logs.
"""
import datetime
import threading
import time

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:  # pragma: no cover -- exercised whenever pyserial
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    serial = None
    _SERIAL_AVAILABLE = False

try:
    import pynmea2
    _PYNMEA2_AVAILABLE = True
except ImportError:  # pragma: no cover -- exercised whenever pynmea2
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    pynmea2 = None
    _PYNMEA2_AVAILABLE = False

GPS_DEVICE = "/dev/ttyACM0"
GPS_BAUDRATE = 57600

# Standard NMEA GGA fix-quality indicator value for a differentially
# corrected (DGPS) fix -- 0 is no fix, 1 is a plain autonomous GPS fix,
# 2 is DGPS, higher values (RTK and friends) aren't used by this project's
# own correction pipeline (gps/dgps_transfer.py sends RTCM corrections
# for a DGPS fix, not RTK) so they're deliberately NOT treated as "DGPS"
# here -- only exactly 2 is.
DGPS_QUALITY = 2

# Standard NMEA-0183 talker IDs and GPS/GNSS sentence types this
# project's receiver may emit -- used by _is_gps_sentence() below to tell
# a genuine GPS frame apart from anything else that can turn up on the
# same serial line (most notably RTCM/DGPS correction traffic, see the
# module docstring above). GN is the multi-constellation talker most
# receivers use once more than one satellite system is combined
# (GPS+GLONASS, etc.); GP is the older GPS-only talker -- gps_reader.py
# and _gga_quality() below already treat both as equivalent for GGA/RMC,
# this just extends the same idea to every sentence type a GPS receiver
# normally sends, not only the two this project happens to parse itself.
GPS_TALKERS = ("GP", "GN", "GL", "GA", "GB", "BD", "GI")
GPS_SENTENCE_TYPES = ("GGA", "RMC", "GLL", "GSA", "GSV", "VTG", "ZDA", "GST", "GBS")


def _timestamp():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _gga_quality(line):
    """Pure parse, no hardware/state involved: returns the integer GGA
    fix-quality indicator for a $GPGGA/$GNGGA sentence, or None for
    anything else -- a different sentence type (e.g. $--RMC, which
    doesn't carry this field), an empty/malformed line, a GGA sentence
    with the quality field blank (some receivers send this before their
    first fix), or pynmea2 not being installed. Kept standalone (rather
    than inlined in _loop()) so it's unit-tested the same way as
    is_full_throttle()/is_full_rotation() elsewhere in this project --
    same pynmea2-based approach and honesty note as link/gps_reader.py's
    parse_fix()."""
    if not _PYNMEA2_AVAILABLE or not line:
        return None
    if not line.startswith(("$GPGGA", "$GNGGA")):
        return None
    try:
        msg = pynmea2.parse(line)
    except pynmea2.ParseError:
        return None
    if not isinstance(msg, pynmea2.types.talker.GGA):
        return None
    try:
        return int(msg.gps_qual)
    except (TypeError, ValueError):
        return None


def is_dgps_quality(quality):
    """True if a GGA quality indicator (as returned by _gga_quality(), or
    None) represents a DGPS-corrected fix. Standalone so DGPS_QUALITY is
    the one place the actual threshold lives."""
    return quality == DGPS_QUALITY


def _is_gps_sentence(line, talkers=GPS_TALKERS, sentence_types=GPS_SENTENCE_TYPES):
    """Pure predicate, no hardware/pynmea2 involved: True if `line` starts
    with a recognized NMEA talker+sentence-type prefix (e.g. "$GPGGA",
    "$GNRMC") -- i.e. looks like a genuine GPS frame, not RTCM/DGPS
    correction bytes or any other noise that can show up on the same
    serial line (see this module's docstring). Deliberately a cheap
    prefix check rather than a full pynmea2 parse/checksum validation:
    unlike _gga_quality() (which needs the real field values), this only
    ever needs to decide what's worth writing to a log file, and a
    prefix check is enough for that -- real RTCM/binary correction bytes,
    decoded as ASCII, essentially never happen to start with "$" followed
    by one of these exact talker+type combinations by chance. Also keeps
    this filter usable without pynmea2 installed, unlike _gga_quality()."""
    if not line or not line.startswith("$") or len(line) < 6:
        return False
    return line[1:3] in talkers and line[3:6] in sentence_types


class MultiConditionGPSLogger:
    """Watches a Remote instance's motor duty cycles (dutyCycleLeft/
    dutyCycleRight, guarded by Remote's own `verrou` lock) against
    several independent conditions at once, off a SINGLE shared GPS
    serial connection, and appends every raw NMEA line to whichever
    condition(s) are currently true -- each to its own log file.

    `conditions` is a list of (trigger, log_path, trigger_name) tuples:
    - trigger(left, right) -> bool: pure predicate, no hardware involved
      (e.g. is_full_throttle, is_full_rotation).
    - log_path: file that condition's matching lines are appended to.
    - trigger_name: used only for that condition's START/END marker
      lines, so each log stays self-describing even if several are
      later grepped together.

    Every condition is evaluated independently on every line -- one can
    be active while another isn't, and each gets its own marker
    transitions regardless of what the others are doing. Runs in its own
    background thread, entirely independent from Remote's own
    pwm()/fonction1() threads -- it only ever *reads* the duty cycles,
    never writes them.

    `on_transition(trigger_name, triggered)` (2026-09-11, optional) is
    called every time one condition's triggered state flips, right
    alongside that transition's START/END marker line -- same
    information, just also handed to the caller as a callback instead of
    only ever landing in a log file. Added so the gps_log_on_full_*.py
    scripts can react live to a maneuver starting/stopping (e.g. buzzing
    the gamepad via link.gamepad_handler.GamepadReader.start_rumble()/
    stop_rumble() so the driver gets physical confirmation recording is
    active) without needing to tail their own log file to find out.
    Defaults to a no-op so existing callers/tests are unaffected.

    `on_gps_quality(is_dgps)` (2026-09-11, optional) is called every time
    a GGA line updates the known fix quality (see _gga_quality()/
    DGPS_QUALITY above) WHILE at least one condition is currently
    triggered -- i.e. only while data is actually being written to a log
    file, never during idle driving between maneuvers. Used by the
    gps_log_on_full_*.py scripts to set the gamepad's rumble intensity
    (strong for a DGPS-corrected fix, weak otherwise) live during
    recording. Also defaults to a no-op. last_is_dgps (a property) always
    reflects the most recently known quality, triggered or not, so a
    fresh on_transition(..., True) can seed start_rumble() with the right
    intensity immediately instead of waiting for the next GGA line."""

    def __init__(self, remote, conditions, device=GPS_DEVICE, baudrate=GPS_BAUDRATE,
                 on_transition=None, on_gps_quality=None):
        self.remote = remote
        self.conditions = list(conditions)
        self.device = device
        self.baudrate = baudrate
        self.on_transition = on_transition or (lambda trigger_name, triggered: None)
        self.on_gps_quality = on_gps_quality or (lambda is_dgps: None)
        self._last_is_dgps = False  # safe default until the first GGA line arrives
        self._running = False

    @property
    def last_is_dgps(self):
        """Most recently known GGA fix quality, as a bool (True = DGPS-
        corrected). Defaults to False (treated as "not DGPS") before the
        first GGA sentence with a quality field is seen."""
        return self._last_is_dgps

    def _read_duty_cycles(self):
        with self.remote.verrou:
            return self.remote.dutyCycleLeft, self.remote.dutyCycleRight

    def start(self):
        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def _writes_for_line(self, line, left, right, was_triggered):
        """Pure step of the dispatch loop, no I/O of its own: given one
        already-read+stripped line off the serial port, the current motor
        duty cycles, and the previous per-condition triggered state (a
        list, updated in place), returns the list of (condition_index,
        text) pairs to append to that condition's log file. Kept separate
        from the actual file/serial I/O in _loop() so it can be
        unit-tested without mocking a serial port or real files.

        `line` is only actually written to a log (once a condition is
        triggered) if _is_gps_sentence(line) says it looks like a genuine
        GPS frame -- see that function and this module's docstring for
        why not every line read off GPS_DEVICE is one (2026-09-11 fix).
        START/END marker lines are unaffected by this: they're written on
        every triggered-state transition regardless of what `line` is."""
        writes = []
        for i, (trigger, _log_path, trigger_name) in enumerate(self.conditions):
            triggered = trigger(left, right)

            if triggered != was_triggered[i]:
                # Log the transition itself too, so the file clearly
                # shows when each triggered period starts/stops, not
                # just an undifferentiated block of NMEA lines.
                marker = f"{trigger_name}_START" if triggered else f"{trigger_name}_END"
                writes.append((i, f"{_timestamp()} # {marker}\n"))
                self.on_transition(trigger_name, triggered)
            was_triggered[i] = triggered

            if triggered and _is_gps_sentence(line):
                writes.append((i, f"{_timestamp()} {line}\n"))
        return writes

    def _loop(self):
        if not _SERIAL_AVAILABLE:
            print("pyserial not installed -- GPS logging disabled. "
                  "Run `pip install -r requirements.txt` to enable it.")
            return

        try:
            ser = serial.Serial(port=self.device, baudrate=self.baudrate, timeout=1)
        except Exception as exc:  # SerialException, FileNotFoundError, PermissionError...
            print(f"GPS device {self.device} unavailable ({exc}) -- GPS logging disabled.")
            return

        names = ", ".join(trigger_name for _, _, trigger_name in self.conditions)
        print(f"GPS logger ready on {self.device} @ {self.baudrate} baud -- watching: {names}.")

        was_triggered = [False] * len(self.conditions)
        log_files = [
            open(log_path, "a", encoding="ascii", errors="replace")
            for _, log_path, _ in self.conditions
        ]
        try:
            while self._running:
                try:
                    raw = ser.readline()
                    line = raw.decode("ascii", errors="replace").strip()
                except Exception as exc:
                    print(f"GPS read error: {exc}")
                    time.sleep(1)
                    continue

                quality = _gga_quality(line)
                if quality is not None:
                    self._last_is_dgps = is_dgps_quality(quality)

                left, right = self._read_duty_cycles()
                for i, text in self._writes_for_line(line, left, right, was_triggered):
                    log_files[i].write(text)
                    log_files[i].flush()

                # Only report live quality while something is actually
                # being recorded (was_triggered was just updated in place
                # by _writes_for_line above) -- no point buzzing an
                # intensity signal for data that isn't being logged.
                if quality is not None and any(was_triggered):
                    self.on_gps_quality(self._last_is_dgps)
        finally:
            for f in log_files:
                f.close()


class ConditionGPSLogger(MultiConditionGPSLogger):
    """Single-condition convenience wrapper around
    MultiConditionGPSLogger, for scripts that only ever watch one
    maneuver (gps_log_on_full_throttle.py, gps_log_on_full_rotation.py).
    Same public interface (`trigger`/`log_path`/`trigger_name` attributes,
    `start()`/`stop()`) as before this class was generalized -- neither
    of those two scripts needed to change."""

    def __init__(self, remote, trigger, log_path, trigger_name="TRIGGER",
                 device=GPS_DEVICE, baudrate=GPS_BAUDRATE, on_transition=None,
                 on_gps_quality=None):
        super().__init__(remote, [(trigger, log_path, trigger_name)], device=device,
                          baudrate=baudrate, on_transition=on_transition,
                          on_gps_quality=on_gps_quality)
        self.trigger = trigger
        self.log_path = log_path
        self.trigger_name = trigger_name
