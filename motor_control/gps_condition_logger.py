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

GPS_DEVICE = "/dev/ttyACM0"
GPS_BAUDRATE = 57600


def _timestamp():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


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
    never writes them."""

    def __init__(self, remote, conditions, device=GPS_DEVICE, baudrate=GPS_BAUDRATE):
        self.remote = remote
        self.conditions = list(conditions)
        self.device = device
        self.baudrate = baudrate
        self._running = False

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
        already-read+stripped NMEA line, the current motor duty cycles,
        and the previous per-condition triggered state (a list, updated
        in place), returns the list of (condition_index, text) pairs to
        append to that condition's log file. Kept separate from the
        actual file/serial I/O in _loop() so it can be unit-tested
        without mocking a serial port or real files."""
        writes = []
        for i, (trigger, _log_path, trigger_name) in enumerate(self.conditions):
            triggered = trigger(left, right)

            if triggered != was_triggered[i]:
                # Log the transition itself too, so the file clearly
                # shows when each triggered period starts/stops, not
                # just an undifferentiated block of NMEA lines.
                marker = f"{trigger_name}_START" if triggered else f"{trigger_name}_END"
                writes.append((i, f"{_timestamp()} # {marker}\n"))
            was_triggered[i] = triggered

            if triggered and line:
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

                left, right = self._read_duty_cycles()
                for i, text in self._writes_for_line(line, left, right, was_triggered):
                    log_files[i].write(text)
                    log_files[i].flush()
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
                 device=GPS_DEVICE, baudrate=GPS_BAUDRATE):
        super().__init__(remote, [(trigger, log_path, trigger_name)], device=device, baudrate=baudrate)
        self.trigger = trigger
        self.log_path = log_path
        self.trigger_name = trigger_name
