"""Background EPever Tracer (solar charge controller) reader: polls the
charge controller over Modbus RTU (RS485, via the CC-USB-RS485-150U cable
and its ch343 kernel driver -- see README.md's "Liaison RS485 / Tracer"
section) and updates a RobotState with the latest power readings, so the
new PWR sentence (see link/server.py and pages/power.html on
robot-webserver) can report real solar/battery/load data instead of a
placeholder.

Kept separate from the standalone check_tracer.py diagnostic script
(delivered earlier, not part of this repo) rather than importing it: that
script is a one-shot CLI tool that opens the port, asks one question and
exits -- this module owns a long-lived connection in a background thread
instead, polling repeatedly, and degrades gracefully if the cable isn't
there. Same "optional, best-effort hardware" pattern already used for the
live camera feed (camera/stream_server.py) and the GPS receiver
(link/gps_reader.py): a missing/unplugged cable logs a warning and leaves
RobotState's power fields at their honest "unavailable" placeholder
rather than crashing the whole control server.

MODBUS REGISTER MAP -- EPever B-series protocol (official EPEVER
"Communication Protocol V2.3"), confirmed against an independent
open-source implementation (rosswarren/epevermodbus). Two separate
blocks, read as two separate Modbus requests since they aren't address-
contiguous:
  - "Real-time Datum" (function 0x04, base 0x3100): PV voltage/current/
    power, battery charging power, load voltage/current/power, battery
    and controller temperature, battery SOC.
  - "Statistical Parameters" (function 0x04, base 0x331A): battery
    voltage and battery charging current -- NOT in the 0x3100 block
    (a documented EPever quirk; see the diysolarforum thread "Epever
    Tracer Modbus - digging deeper" for the community history of
    confusion around this exact split).
All values are /100 except battery SOC, which is a plain percentage
(verified against the chip's own register table, NOT yet cross-checked
against this project's real Tracer display -- flag any obviously wrong
SOC reading, e.g. "0.85%" instead of "85%", if that turns out to be
backwards on this particular firmware).

IMPORTANT -- honesty note, same as link/gps_reader.py: pyserial is
already a requirement (used by gps_reader.py too), but this module's
Modbus framing (CRC16, request building, response parsing) was validated
standalone against the real Tracer hardware on 2026-10-03 via a one-off
diagnostic script (single-register read of 0x3100, the PV voltage
register) -- NOT yet against the full multi-register reads this module
actually performs. Run `pytest tests/test_tracer_reader.py` (mock-based,
always runs) AND a real read on the Pi before relying on the full field
set (battery/load/temperature/SOC) in the field.
"""
import logging
import threading
import time

log = logging.getLogger("link.tracer_reader")

try:
    import serial
    _SERIAL_AVAILABLE = True
    _SERIAL_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover -- exercised whenever pyserial
    # isn't installed (e.g. this project's dev sandbox); on the Pi, with
    # requirements.txt installed, this branch is never taken.
    serial = None
    _SERIAL_AVAILABLE = False
    _SERIAL_IMPORT_ERROR = exc

DEFAULT_DEVICE = "/dev/ttyCH343USB0"  # the RS485 cable's ch343 driver device node
DEFAULT_BAUDRATE = 115200
DEFAULT_SLAVE_ID = 1
DEFAULT_POLL_INTERVAL_S = 3.0  # matches STA's own polling cadence (robot-webserver)
DEFAULT_READ_TIMEOUT_S = 1.0

# -- Modbus register map (see module docstring) ------------------------------
REALTIME_START = 0x3100
REALTIME_QUANTITY = 0x1B  # covers 0x3100..0x311A inclusive (27 registers)
BATTERY_START = 0x331A
BATTERY_QUANTITY = 3      # covers 0x331A..0x331C inclusive

# Offsets within the REALTIME block (index = address - REALTIME_START).
_PV_VOLTAGE = 0x00        # 0x3100, /100, V
_PV_CURRENT = 0x01        # 0x3101, /100, A
_PV_POWER = 0x02          # 0x3102-0x3103 (double), /100, W
_BATTERY_CHARGING_POWER = 0x06  # 0x3106-0x3107 (double), /100, W
_LOAD_VOLTAGE = 0x0C      # 0x310C, /100, V
_LOAD_CURRENT = 0x0D      # 0x310D, /100, A
_LOAD_POWER = 0x0E        # 0x310E-0x310F (double), /100, W
_BATTERY_TEMP = 0x10      # 0x3110, /100, signed, degC
_CONTROLLER_TEMP = 0x11   # 0x3111, /100, signed, degC
_BATTERY_SOC = 0x1A       # 0x311A, NOT scaled, %

# Offsets within the BATTERY block (index = address - BATTERY_START).
_BATTERY_VOLTAGE = 0x00          # 0x331A, /100, V
_BATTERY_CHARGING_CURRENT = 0x01  # 0x331B-0x331C (double, signed), /100, A


class ModbusError(Exception):
    """Raised by read_input_registers() for anything that keeps a request
    from yielding a trustworthy set of register values -- timeout,
    truncated response, a Modbus exception reply, or a CRC mismatch. One
    exception type for all of these since every caller in this module
    reacts the same way (log and treat this poll as failed)."""


def modbus_crc16(data: bytes) -> bytes:
    """Modbus RTU CRC-16 (polynomial 0xA001, init 0xFFFF, LSB-first).
    Transmitted low byte first, then high byte -- exactly what
    to_bytes(2, 'little') produces. Verified 2026-10-03 against the
    textbook reference frame 01 03 00 00 00 0A -> C5 CD (see
    tests/test_tracer_reader.py)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, "little")


def build_read_input_registers(slave_id: int, start_register: int, quantity: int) -> bytes:
    """Builds a Modbus RTU function-0x04 (Read Input Registers) request."""
    body = bytes([
        slave_id, 0x04,
        (start_register >> 8) & 0xFF, start_register & 0xFF,
        (quantity >> 8) & 0xFF, quantity & 0xFF,
    ])
    return body + modbus_crc16(body)


def read_input_registers(ser, slave_id: int, start_register: int, quantity: int) -> list:
    """Sends one function-0x04 request and returns `quantity` raw uint16
    register values. Raises ModbusError (see its docstring) for any
    response that isn't a clean, CRC-valid success reply."""
    request = build_read_input_registers(slave_id, start_register, quantity)
    ser.reset_input_buffer()
    ser.write(request)
    ser.flush()

    expected_len = 5 + 2 * quantity  # slave_id + func + byte_count + data + crc
    response = ser.read(expected_len)

    if len(response) == 0:
        raise ModbusError(f"no response from slave {slave_id} (timeout)")
    if len(response) < 5:
        raise ModbusError(f"truncated response ({len(response)} bytes)")
    if response[1] & 0x80:
        raise ModbusError(f"Modbus exception, code {response[2]:#04x}")
    if len(response) < expected_len:
        raise ModbusError(f"incomplete response ({len(response)}/{expected_len} bytes)")

    payload, received_crc = response[:-2], response[-2:]
    if modbus_crc16(payload) != received_crc:
        raise ModbusError("CRC mismatch")

    byte_count = response[2]
    if byte_count != 2 * quantity:
        raise ModbusError(f"unexpected byte count ({byte_count}, expected {2 * quantity})")

    data = response[3:3 + byte_count]
    return [int.from_bytes(data[i:i + 2], "big") for i in range(0, byte_count, 2)]


def _u32(regs, index):
    """Combines two consecutive registers into an unsigned 32-bit value --
    EPever's documented convention is low register first, high register
    second (ascending address = ascending significance)."""
    return (regs[index + 1] << 16) | regs[index]


def _s32(regs, index):
    value = _u32(regs, index)
    return value - 0x100000000 if value >= 0x80000000 else value


def _s16(regs, index):
    value = regs[index]
    return value - 0x10000 if value >= 0x8000 else value


def read_power_snapshot(ser, slave_id: int) -> dict:
    """Reads both register blocks and returns a dict of scaled, named
    values. Raises ModbusError (uncaught here -- see TracerReader._loop)
    if either request fails; a snapshot is all-or-nothing, since a
    partial reading (e.g. PV data but no battery voltage) is more
    confusing on the power page than simply retrying next poll."""
    realtime = read_input_registers(ser, slave_id, REALTIME_START, REALTIME_QUANTITY)
    battery = read_input_registers(ser, slave_id, BATTERY_START, BATTERY_QUANTITY)

    return {
        "pv_voltage": realtime[_PV_VOLTAGE] / 100.0,
        "pv_current": realtime[_PV_CURRENT] / 100.0,
        "pv_power": _u32(realtime, _PV_POWER) / 100.0,
        "battery_charging_power": _u32(realtime, _BATTERY_CHARGING_POWER) / 100.0,
        "load_voltage": realtime[_LOAD_VOLTAGE] / 100.0,
        "load_current": realtime[_LOAD_CURRENT] / 100.0,
        "load_power": _u32(realtime, _LOAD_POWER) / 100.0,
        "battery_temp": _s16(realtime, _BATTERY_TEMP) / 100.0,
        "controller_temp": _s16(realtime, _CONTROLLER_TEMP) / 100.0,
        "battery_soc": realtime[_BATTERY_SOC],
        "battery_voltage": battery[_BATTERY_VOLTAGE] / 100.0,
        "battery_charging_current": _s32(battery, _BATTERY_CHARGING_CURRENT) / 100.0,
    }


class TracerReader:
    """Runs in a background thread, polling the Tracer every
    `poll_interval` seconds and updating `state` (a
    link.robot_state.RobotState) via state.update_power_reading(). Same
    "open once, degrade on failure, no retry-reopen" shape as
    link.gps_reader.GPSReader -- see that module's docstring for the
    rationale; a cable plugged in after this thread has already given up
    needs the control server restarted, same as the GPS today."""

    def __init__(self, state, device=DEFAULT_DEVICE, baudrate=DEFAULT_BAUDRATE,
                 slave_id=DEFAULT_SLAVE_ID, poll_interval=DEFAULT_POLL_INTERVAL_S):
        self.state = state
        self.device = device
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.poll_interval = poll_interval
        self._running = False

    def start(self):
        self._running = True
        thread = threading.Thread(target=self._loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._running = False

    def _loop(self):
        if not _SERIAL_AVAILABLE:
            log.warning(
                "pyserial not installed (%s) -- Tracer/solar reading disabled, "
                "power fields stay unavailable. Run `pip install -r "
                "requirements.txt` to enable it.",
                _SERIAL_IMPORT_ERROR,
            )
            return

        try:
            ser = serial.Serial(
                port=self.device, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=DEFAULT_READ_TIMEOUT_S,
            )
        except Exception as exc:  # SerialException, FileNotFoundError, PermissionError...
            log.warning(
                "Tracer device %s not available (%s) -- power fields stay "
                "unavailable until the RS485 cable is connected.",
                self.device, exc,
            )
            return

        log.info("Tracer reader started on %s @ %s baud (slave id %s)",
                  self.device, self.baudrate, self.slave_id)
        while self._running:
            try:
                snapshot = read_power_snapshot(ser, self.slave_id)
            except ModbusError as exc:
                log.warning("Tracer read failed: %s", exc)
                self.state.update_power_reading(available=False)
            except Exception as exc:  # serial.SerialException mid-operation, etc.
                log.warning("Tracer serial error: %s", exc)
                self.state.update_power_reading(available=False)
            else:
                self.state.update_power_reading(available=True, **snapshot)
            time.sleep(self.poll_interval)
