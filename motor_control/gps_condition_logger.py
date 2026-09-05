"""Shared machinery behind the motor_control/gps_log_on_*.py scripts.

Each of those scripts drives the robot with a gamepad exactly like
remote_control.py (same Remote class, unchanged), and wants to log raw
NMEA sentences from the GPS receiver only while the two motors' duty
cycles satisfy some specific condition -- full throttle in a straight
line (gps_log_on_full_throttle.py), full-speed rotation in place
(gps_log_on_full_rotation.py), and potentially others later (step
response on other maneuvers). Rather than duplicating the GPS-reading/
logging loop in every one of those scripts, it lives once here,
parameterized by the trigger predicate and the log file to use.

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


class ConditionGPSLogger:
    """Watches a Remote instance's motor duty cycles (dutyCycleLeft/
    dutyCycleRight, guarded by Remote's own `verrou` lock) and appends
    every raw NMEA line read from the GPS receiver to `log_path`, for as
    long as `trigger(left, right)` returns True. Runs in its own
    background thread, entirely independent from Remote's own
    pwm()/fonction1() threads -- it only ever *reads* the duty cycles,
    never writes them.

    `trigger_name` is used only for the START/END marker lines written
    to the log whenever `trigger` flips, so each maneuver's log stays
    self-describing (e.g. "FULL_THROTTLE_START"/"FULL_ROTATION_START")
    even if several logs are later grepped together."""

    def __init__(self, remote, trigger, log_path, trigger_name="TRIGGER",
                 device=GPS_DEVICE, baudrate=GPS_BAUDRATE):
        self.remote = remote
        self.trigger = trigger
        self.log_path = log_path
        self.trigger_name = trigger_name
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

        print(f"GPS logger ready on {self.device} @ {self.baudrate} baud -- "
              f"logging to {self.log_path} whenever {self.trigger_name} is true.")

        was_triggered = False
        with open(self.log_path, "a", encoding="ascii", errors="replace") as log_file:
            while self._running:
                try:
                    raw = ser.readline()
                    line = raw.decode("ascii", errors="replace").strip()
                except Exception as exc:
                    print(f"GPS read error: {exc}")
                    time.sleep(1)
                    continue

                left, right = self._read_duty_cycles()
                triggered = self.trigger(left, right)

                if triggered != was_triggered:
                    # Log the transition itself too, so the file clearly
                    # shows when each triggered period starts/stops, not
                    # just an undifferentiated block of NMEA lines.
                    marker = f"{self.trigger_name}_START" if triggered else f"{self.trigger_name}_END"
                    log_file.write(f"{_timestamp()} # {marker}\n")
                    log_file.flush()
                was_triggered = triggered

                if triggered and line:
                    log_file.write(f"{_timestamp()} {line}\n")
                    log_file.flush()
