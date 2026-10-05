"""Tests for link.tracer_reader: pure Modbus framing/math (always runs,
no dependencies) plus TracerReader's background loop against a fake
serial port (needs pyserial importable -- see conftest.py's note on why
gpiod/evdev are stubbed there; pyserial itself has no such stub since
link.gps_reader already requires it unconditionally for this whole test
suite to import link.server).

Honesty note, same as tests/test_gps_reader.py: the CRC16/request-framing
logic (modbus_crc16, build_read_input_registers, a single-register read of
0x3100) WAS validated against the real Tracer hardware on 2026-10-03 (see
the standalone check_tracer.py diagnostic delivered that day) -- the
multi-register reads read_power_snapshot() performs (the 0x3100 and
0x331A blocks together) have NOT been run against the real device yet.
Run a real read on the Pi (e.g. via the new PWR sentence, or a quick
`python3 -c "..."` using read_power_snapshot directly) before trusting
the full field set in the field.
"""
import pytest

serial = pytest.importorskip("serial")  # noqa: F841 -- just probing availability

from link.tracer_reader import (  # noqa: E402
    BATTERY_QUANTITY,
    BATTERY_START,
    REALTIME_QUANTITY,
    REALTIME_START,
    ModbusError,
    TracerReader,
    _BATTERY_CHARGING_CURRENT,
    _BATTERY_CHARGING_POWER,
    _BATTERY_SOC,
    _BATTERY_TEMP,
    _BATTERY_VOLTAGE,
    _CONTROLLER_TEMP,
    _LOAD_CURRENT,
    _LOAD_POWER,
    _LOAD_VOLTAGE,
    _PV_CURRENT,
    _PV_POWER,
    _PV_VOLTAGE,
    build_read_input_registers,
    modbus_crc16,
    read_input_registers,
    read_power_snapshot,
)
from link.robot_state import RobotState  # noqa: E402


# --- modbus_crc16 / build_read_input_registers (pure math) ------------------

def test_modbus_crc16_matches_the_textbook_reference_frame():
    # 01 03 00 00 00 0A -> C5 CD, transmitted low byte first -- the same
    # vector this project self-verified by hand on 2026-10-03 before
    # trusting the algorithm (see check_tracer.py's delivery message).
    assert modbus_crc16(bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])) == bytes([0xC5, 0xCD])


def test_build_read_input_registers_matches_the_real_validated_request():
    # The exact bytes this project sent to the real Tracer on 2026-10-03
    # (slave 1, function 0x04, register 0x3100, quantity 1) and got a
    # real, CRC-valid 15.82V reply back for.
    request = build_read_input_registers(1, 0x3100, 1)
    assert request == bytes([0x01, 0x04, 0x31, 0x00, 0x00, 0x01, 0x3F, 0x36])


# --- read_input_registers (needs a fake serial port) -------------------------

class FakeSerial:
    """Queue of canned responses: each call to read() pops (a prefix of)
    the next one. write()/flush()/reset_input_buffer() are no-ops that
    just record what was sent, same shape as camera/stream_server.py's
    tests' fakes elsewhere in this project."""

    def __init__(self):
        self.sent = []
        self.responses = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.sent.append(bytes(data))

    def flush(self):
        pass

    def read(self, n):
        if not self.responses:
            return b""
        return self.responses.pop(0)[:n]


def _framed(payload: bytes) -> bytes:
    return payload + modbus_crc16(payload)


def test_read_input_registers_success():
    # Real captured reply from the Tracer for register 0x3100 (PV voltage
    # = 0x062E / 100 = 15.82V), 2026-10-03.
    ser = FakeSerial()
    ser.responses = [_framed(bytes([0x01, 0x04, 0x02, 0x06, 0x2E]))]
    assert read_input_registers(ser, 1, 0x3100, 1) == [0x062E]


def test_read_input_registers_raises_on_timeout():
    ser = FakeSerial()
    ser.responses = [b""]
    with pytest.raises(ModbusError, match="timeout"):
        read_input_registers(ser, 1, 0x3100, 1)


def test_read_input_registers_raises_on_truncated_response():
    ser = FakeSerial()
    ser.responses = [bytes([0x01, 0x04])]
    with pytest.raises(ModbusError, match="truncated"):
        read_input_registers(ser, 1, 0x3100, 1)


def test_read_input_registers_raises_on_modbus_exception_reply():
    ser = FakeSerial()
    ser.responses = [_framed(bytes([0x01, 0x84, 0x02]))]  # illegal data address
    with pytest.raises(ModbusError, match="exception"):
        read_input_registers(ser, 1, 0x3100, 1)


def test_read_input_registers_raises_on_crc_mismatch():
    ser = FakeSerial()
    ser.responses = [bytes([0x01, 0x04, 0x02, 0x06, 0x2E, 0x00, 0x00])]  # wrong CRC
    with pytest.raises(ModbusError, match="CRC"):
        read_input_registers(ser, 1, 0x3100, 1)


# --- read_power_snapshot (both register blocks, double-registers, signed) ---

def _registers_to_response(regs):
    data = b"".join(v.to_bytes(2, "big") for v in regs)
    payload = bytes([0x01, 0x04, len(data)]) + data
    return _framed(payload)


def test_read_power_snapshot_decodes_every_field_correctly():
    realtime = [0] * REALTIME_QUANTITY
    realtime[_PV_VOLTAGE] = 1582          # 15.82 V
    realtime[_PV_CURRENT] = 120           # 1.20 A
    realtime[_PV_POWER] = 1898            # 18.98 W, low word
    realtime[_PV_POWER + 1] = 0           # high word
    realtime[_BATTERY_CHARGING_POWER] = 1441
    realtime[_BATTERY_CHARGING_POWER + 1] = 0
    realtime[_LOAD_VOLTAGE] = 1290
    realtime[_LOAD_CURRENT] = 50
    realtime[_LOAD_POWER] = 645
    realtime[_LOAD_POWER + 1] = 0
    realtime[_BATTERY_TEMP] = 2430                # +24.30 degC
    realtime[_CONTROLLER_TEMP] = 0x10000 - 50      # -0.50 degC (signed int16)
    realtime[_BATTERY_SOC] = 87                    # NOT /100 -- raw percent

    battery = [0] * BATTERY_QUANTITY
    battery[_BATTERY_VOLTAGE] = 1310
    battery[_BATTERY_CHARGING_CURRENT] = 110
    battery[_BATTERY_CHARGING_CURRENT + 1] = 0

    ser = FakeSerial()
    ser.responses = [_registers_to_response(realtime), _registers_to_response(battery)]
    snapshot = read_power_snapshot(ser, 1)

    assert snapshot == {
        "pv_voltage": 15.82, "pv_current": 1.2, "pv_power": 18.98,
        "battery_charging_power": 14.41,
        "load_voltage": 12.9, "load_current": 0.5, "load_power": 6.45,
        "battery_temp": 24.3, "controller_temp": -0.5, "battery_soc": 87,
        "battery_voltage": 13.1, "battery_charging_current": 1.1,
    }

    # Two separate requests, correct target blocks (not address-contiguous).
    assert build_read_input_registers(1, REALTIME_START, REALTIME_QUANTITY) == ser.sent[0]
    assert build_read_input_registers(1, BATTERY_START, BATTERY_QUANTITY) == ser.sent[1]


def test_read_power_snapshot_raises_if_either_block_fails():
    # All-or-nothing by design (see module docstring): a working realtime
    # block but a failing battery block must not silently return a
    # half-filled dict.
    ser = FakeSerial()
    ser.responses = [_registers_to_response([0] * REALTIME_QUANTITY), b""]
    with pytest.raises(ModbusError):
        read_power_snapshot(ser, 1)


# --- TracerReader background loop (degrade-gracefully paths) ----------------

def test_tracer_reader_degrades_gracefully_with_no_device(monkeypatch):
    # Device path that cannot possibly exist -- must log and return, never
    # raise out of the background thread (same contract as GPSReader).
    state = RobotState()
    reader = TracerReader(state, device="/dev/definitely-not-a-real-device")
    reader._loop()  # run synchronously instead of via start(), for the test
    p = state.power_status()
    assert p["available"] is None  # never even attempted a poll


def test_tracer_reader_updates_state_on_a_successful_poll(monkeypatch):
    fake_ser = FakeSerial()
    realtime = [0] * REALTIME_QUANTITY
    realtime[_PV_VOLTAGE] = 1582
    battery = [0] * BATTERY_QUANTITY
    battery[_BATTERY_VOLTAGE] = 1310
    fake_ser.responses = [_registers_to_response(realtime), _registers_to_response(battery)]

    monkeypatch.setattr(
        "link.tracer_reader.serial.Serial", lambda *a, **kw: fake_ser
    )

    state = RobotState()
    reader = TracerReader(state, poll_interval=0)
    reader._running = True

    # Run exactly one iteration of the loop body by stopping it right
    # after the first successful update -- same "drive the loop from the
    # test" approach as a bare call to _loop() above, just stopping
    # between iterations instead of never entering the while loop.
    original_update = state.update_power_reading

    def _update_once(*a, **kw):
        original_update(*a, **kw)
        reader._running = False

    monkeypatch.setattr(state, "update_power_reading", _update_once)
    reader._loop()

    p = state.power_status()
    assert p["available"] is True
    assert p["pv_voltage"] == 15.82
    assert p["battery_voltage"] == 13.1
