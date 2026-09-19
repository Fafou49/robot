"""Background GPS reader for the robot's live current position: parses
NMEA sentences from a serial GPS receiver (same device/baud rate as
gps/gps_transfer.py and gps/gps_parse.py) and updates a RobotState with
the latest fix, so STA can report a real current position, course and
speed instead of the 0.0 placeholder.

Kept separate from gps/gps_transfer.py and gps/gps_parse.py rather than
importing them: both open the serial port and start their own blocking
loop as a side effect of being *imported* (top-level `serial.Serial(...)`
and, in gps_parse.py, a top-level `while True`), so importing either one
here would either crash immediately (no /dev/ttyACM0 in a dev/test
environment) or fight both scripts over the same serial port. This
module owns its own connection instead, in a background thread, and
degrades gracefully if the device isn't there: it logs one warning and
leaves RobotState's current position at the honest 0.0 placeholder
rather than crashing the whole control server -- the same "optional,
best-effort hardware" pattern already used for the live camera feed
(camera/stream_server.py).

IMPORTANT -- honesty note about how this was verified: pynmea2 and
pyserial (both already in requirements.txt) could not be installed in
the sandbox this was written in (no PyPI access there -- the same
limitation noted elsewhere in this project for pytest). parse_fix()'s
control flow was written carefully against pynmea2's documented,
stable public API -- the same attributes gps/gps_transfer.py and
gps/gps_parse.py already use successfully (.latitude, .longitude,
pynmea2.parse()) plus two more from the same message classes
(.spd_over_grnd, .true_course, .status, .gps_qual) -- but it has NOT
been run against the real library or a real GPS fix. Run
`pytest tests/test_gps_reader.py` on the Pi (or any machine with
pynmea2 installed) before relying on this. The graceful-degradation
paths (device absent, and even the libraries themselves missing) ARE
exercised for real here and confirmed not to crash the control server.
"""
import logging
import threading
import time

log = logging.getLogger("link.gps_reader")

try:
    import pynmea2
    import serial
    _GPS_LIBS_AVAILABLE = True
    _GPS_LIBS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover -- exercised whenever these
    # aren't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    pynmea2 = None
    serial = None
    _GPS_LIBS_AVAILABLE = False
    _GPS_LIBS_IMPORT_ERROR = exc

DEFAULT_DEVICE = "/dev/ttyACM0"  # matches gps/gps_transfer.py, gps/gps_parse.py
DEFAULT_BAUDRATE = 57600
KNOTS_TO_KMH = 1.852


def parse_fix(line: str):
    """Parses one raw NMEA line. Returns a dict {lat, lon, speed_kmh, cap,
    quality} (speed_kmh/cap are None for a GGA sentence, which doesn't
    carry them; quality is None for an RMC sentence, which doesn't carry
    THAT -- see the `quality` key below) for a sentence with a valid fix,
    or None for anything else (wrong sentence type, checksum/parse
    failure, no fix yet -- RMC status "V", GGA quality 0, or pynmea2 not
    installed).

    `quality` (added 2026-09-18) is the raw GGA fix-quality indicator
    (standard NMEA values: 1 = plain autonomous GPS fix, 2 = DGPS-
    corrected -- this project's own gps/dgps_transfer.py correction
    pipeline targets exactly quality 2, higher values like RTK aren't
    produced by it) -- used by GPSReader below to fire on_gps_quality()
    on a genuine DGPS-acquired/lost transition, which in turn drives the
    gamepad's short confirmation vibration (link/server.py). Same
    DGPS_QUALITY threshold and reasoning as motor_control/
    gps_condition_logger.py's is_dgps_quality() -- kept as an independent
    copy here rather than imported, same "kept separate" philosophy this
    module's own docstring already explains for gps/gps_transfer.py and
    gps/gps_parse.py (this is the main navigation pipeline, that module
    is field-test-only tooling; neither should have to import the other
    to change independently)."""
    if not _GPS_LIBS_AVAILABLE:
        return None

    line = line.strip()
    if not line.startswith(("$GPRMC", "$GNRMC", "$GPGGA", "$GNGGA")):
        return None

    try:
        msg = pynmea2.parse(line)
    except pynmea2.ParseError:
        return None

    if isinstance(msg, pynmea2.types.talker.RMC):
        if msg.status != "A":  # "A" = active/valid fix, "V" = void
            return None
        speed_kmh = float(msg.spd_over_grnd) * KNOTS_TO_KMH if msg.spd_over_grnd else 0.0
        cap = float(msg.true_course) if msg.true_course else None
        return {"lat": msg.latitude, "lon": msg.longitude, "speed_kmh": speed_kmh,
                "cap": cap, "quality": None}

    if isinstance(msg, pynmea2.types.talker.GGA):
        if not msg.gps_qual or int(msg.gps_qual) == 0:
            return None
        return {"lat": msg.latitude, "lon": msg.longitude, "speed_kmh": None,
                "cap": None, "quality": int(msg.gps_qual)}

    return None


# Standard NMEA GGA fix-quality indicator value for a differentially
# corrected (DGPS) fix -- see parse_fix()'s docstring above. Exactly 2,
# same as motor_control/gps_condition_logger.py's DGPS_QUALITY.
DGPS_QUALITY = 2


class GPSReader:
    """Runs in a background thread, updates `state` (a
    link.robot_state.RobotState) with each valid fix parse_fix() finds.
    Speed/course are only ever set from a GPRMC sentence (the only one of
    the two this reads that carries them) -- a GGA-only fix updates
    position and leaves the last known speed/course as-is.

    on_gps_quality(is_dgps) (2026-09-18, optional) is called only on a
    genuine EDGE -- the live fix quality actually flipping between DGPS
    and not-DGPS -- never once per GGA line. This is what link/server.py
    uses to buzz the gamepad (a short, one-shot pulse -- see
    link.gamepad_handler.GamepadReader.pulse()) the instant fix quality
    actually changes during real operation, replacing the older
    continuous-while-a-maneuver-is-logged vibration that used to live in
    motor_control/gps_log_on_full_*.py (removed 2026-09-18 -- that was a
    field-test-only feature, unrelated to this main navigation pipeline).
    Defaults to a no-op so every existing caller/test is unaffected."""

    def __init__(self, state, device=DEFAULT_DEVICE, baudrate=DEFAULT_BAUDRATE,
                 on_gps_quality=None):
        self.state = state
        self.device = device
        self.baudrate = baudrate
        self.on_gps_quality = on_gps_quality or (lambda is_dgps: None)
        # None until the first GGA quality is seen -- deliberately NOT a
        # bool default (unlike gps_condition_logger.py's last_is_dgps,
        # which defaults to False): a None->False transition is not a
        # real event (the fix was simply never DGPS to begin with, there
        # was no "losing" it), so it must never fire on_gps_quality(False)
        # by itself. Only None->True and any actual True<->False flip do.
        self._last_is_dgps = None
        self._running = False

    def start(self):
        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def _loop(self):
        if not _GPS_LIBS_AVAILABLE:
            log.warning(
                "pynmea2/pyserial not installed (%s) -- GPS reading disabled, "
                "current position stays at the 0.0 placeholder. Run "
                "`pip install -r requirements.txt` to enable it.",
                _GPS_LIBS_IMPORT_ERROR,
            )
            return

        try:
            ser = serial.Serial(port=self.device, baudrate=self.baudrate, timeout=1)
        except Exception as exc:  # SerialException, FileNotFoundError, PermissionError...
            log.warning(
                "GPS device %s not available (%s) -- current position stays "
                "at the 0.0 placeholder until a receiver is connected.",
                self.device, exc,
            )
            return

        log.info("GPS reader started on %s @ %s baud", self.device, self.baudrate)
        while self._running:
            try:
                raw = ser.readline()
                line = raw.decode("ascii", errors="replace")
            except Exception as exc:
                log.warning("GPS read error: %s", exc)
                time.sleep(1)
                continue

            fix = parse_fix(line)
            if fix is not None:
                # None until a GGA sentence with a quality field has
                # actually arrived (RMC-only lines carry no quality info
                # at all) -- see RobotState.is_dgps's own docstring for
                # why this stays a tri-state rather than defaulting to
                # False. Computed once here and reused both for
                # RobotState's persisted status (below) and for the
                # edge-triggered on_gps_quality() callback (further down),
                # rather than recomputing it twice.
                is_dgps = fix["quality"] == DGPS_QUALITY if fix["quality"] is not None else None
                self.state.update_gps_fix(
                    fix["lat"], fix["lon"], speed_kmh=fix["speed_kmh"], cap=fix["cap"],
                    is_dgps=is_dgps,
                )
                if fix["quality"] is not None:
                    if self._last_is_dgps is None:
                        # First GGA quality ever seen: only worth firing
                        # if it's already DGPS (the fix "arrived" already
                        # corrected) -- a first reading of "not DGPS" is
                        # not a real "lost precision" event, there was
                        # nothing before it to lose.
                        self._last_is_dgps = is_dgps
                        if is_dgps:
                            self.on_gps_quality(True)
                    elif is_dgps != self._last_is_dgps:
                        self._last_is_dgps = is_dgps
                        self.on_gps_quality(is_dgps)
