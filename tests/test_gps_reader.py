"""Tests for GPS support: link.nmea's decimal_to_nmea/nmea_to_decimal
(pure math, no dependencies, always run) and link.gps_reader.parse_fix
(needs pynmea2 -- skipped via pytest.importorskip if it isn't installed).

Honesty note: pynmea2/pyserial couldn't be installed in the sandbox this
was written in (no PyPI access there), so the parse_fix tests below were
never actually executed against the real library -- they're written
against pynmea2's documented API and standard NMEA sentence syntax
(checksums computed by hand, see the comment above each sentence), but
should be treated as unverified until they've actually run once, e.g. on
the Pi with requirements.txt installed:
    pytest tests/test_gps_reader.py
"""
import types

import pytest

from link.nmea import decimal_to_nmea, nmea_to_decimal


# --- decimal_to_nmea / nmea_to_decimal (no dependencies) --------------------

@pytest.mark.parametrize("decimal,is_lon,expected_str,expected_dir", [
    (47.391534, False, "4723.492", "N"),
    (-0.739006, True, "00044.340", "W"),
    (48.117300, False, "4807.038", "N"),
    (11.516667, True, "01131.000", "E"),
    (0.0, False, "00.000", "N"),
])
def test_decimal_to_nmea(decimal, is_lon, expected_str, expected_dir):
    nmea_str, direction = decimal_to_nmea(decimal, is_longitude=is_lon)
    assert direction == expected_dir
    assert float(nmea_str) == pytest.approx(float(expected_str), abs=1e-3)


@pytest.mark.parametrize("decimal,is_lon", [
    (47.391534, False),
    (-0.739006, True),
    (48.1173, False),
    (-11.5167, True),
    (89.999, False),
])
def test_nmea_to_decimal_round_trip(decimal, is_lon):
    nmea_str, direction = decimal_to_nmea(decimal, is_longitude=is_lon)
    back = nmea_to_decimal(nmea_str, direction)
    assert back == pytest.approx(decimal, abs=1e-4)


def test_nmea_to_decimal_invalid_returns_none():
    assert nmea_to_decimal("not-a-number", "N") is None
    assert nmea_to_decimal(None, "N") is None


# --- parse_fix (needs pynmea2) ----------------------------------------------

pynmea2 = pytest.importorskip("pynmea2")

from link.gps_reader import parse_fix  # noqa: E402 -- must follow importorskip


def test_parse_fix_ignores_unrelated_lines():
    assert parse_fix("") is None
    assert parse_fix("not a sentence at all") is None
    assert parse_fix("$GPGSV,3,1,09,...*4B") is None  # a real but unhandled sentence type


def test_parse_fix_valid_rmc_with_speed_and_course():
    # Checksum computed by hand (XOR of the bytes between $ and *) -- the
    # same algorithm as this project's own $PROV sentences.
    line = "$GPRMC,123519,A,4723.492,N,00044.340,W,2.0,284.5,230394,,,A*63"
    fix = parse_fix(line)
    assert fix is not None
    assert fix["lat"] == pytest.approx(47.391533, abs=1e-4)
    assert fix["lon"] == pytest.approx(-0.7390, abs=1e-3)
    assert fix["speed_kmh"] == pytest.approx(2.0 * 1.852, abs=1e-3)
    assert fix["cap"] == pytest.approx(284.5, abs=1e-3)


def test_parse_fix_void_rmc_returns_none():
    line = "$GPRMC,123519,V,4723.492,N,00044.340,W,2.0,284.5,230394,,,N*7B"
    assert parse_fix(line) is None


def test_parse_fix_valid_gga_has_no_speed_or_course():
    line = "$GPGGA,123519,4723.492,N,00044.340,W,1,08,0.9,15.2,M,45.0,M,,*61"
    fix = parse_fix(line)
    assert fix is not None
    assert fix["lat"] == pytest.approx(47.391533, abs=1e-4)
    assert fix["lon"] == pytest.approx(-0.7390, abs=1e-3)
    assert fix["speed_kmh"] is None
    assert fix["cap"] is None


def test_parse_fix_gga_no_fix_returns_none():
    line = "$GPGGA,123519,4723.492,N,00044.340,W,0,00,,,M,,M,,*48"
    assert parse_fix(line) is None


# --- GPSReader._loop(): edge-triggered on_gps_quality (2026-09-18) ---------
#
# Deliberately does NOT need pynmea2/pyserial installed (unlike the
# parse_fix tests above, which do): parse_fix() and serial.Serial are
# both monkeypatched directly on the link.gps_reader module, the same
# "stub the hardware-facing bit, test the pure control flow around it"
# spirit as tests/conftest.py's evdev stub for link.gamepad_handler. This
# is what makes it possible to actually exercise the DGPS-transition
# logic in this sandbox at all, where neither library can be installed.

class _FakeState:
    def __init__(self):
        self.fixes = []

    def update_gps_fix(self, lat, lon, speed_kmh=None, cap=None, is_dgps=None):
        self.fixes.append((lat, lon, speed_kmh, cap, is_dgps))


def _run_loop_with_fixes(monkeypatch, fixes, return_state=False):
    """Drives one GPSReader._loop() pass against a scripted list of
    `fix` dicts (as parse_fix() would return them), stopping the loop
    itself once they're exhausted. Returns the list of on_gps_quality
    calls made along the way -- or, with return_state=True, a
    (calls, state) pair so a caller can also inspect what was passed to
    RobotState.update_gps_fix() (see _FakeState above)."""
    import link.gps_reader as gr

    monkeypatch.setattr(gr, "_GPS_LIBS_AVAILABLE", True)

    index = {"i": 0}

    def fake_parse_fix(line):
        i = index["i"]
        if i >= len(fixes):
            return None
        index["i"] += 1
        return fixes[i]

    monkeypatch.setattr(gr, "parse_fix", fake_parse_fix)

    calls = []
    state = _FakeState()
    reader = gr.GPSReader(state, on_gps_quality=lambda is_dgps: calls.append(is_dgps))
    reader._running = True

    class _FakeSerial:
        def __init__(self, port, baudrate, timeout):
            pass

        def readline(self):
            if index["i"] >= len(fixes):
                reader._running = False  # stop _loop()'s while loop after this line
                return b""
            return b"$GPGGA,dummy*00\r\n"

    monkeypatch.setattr(gr, "serial", types.SimpleNamespace(Serial=_FakeSerial))

    reader._loop()
    if return_state:
        return calls, state
    return calls


def test_gps_reader_fires_true_the_moment_dgps_is_first_seen(monkeypatch):
    calls = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
    ])
    assert calls == [True]


def test_gps_reader_does_not_fire_on_a_first_non_dgps_reading(monkeypatch):
    # A None -> False transition is not a real "lost precision" event --
    # the fix was simply never DGPS to begin with, see GPSReader's own
    # docstring/comment on _last_is_dgps.
    calls = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 1},
    ])
    assert calls == []


def test_gps_reader_does_not_refire_while_quality_stays_the_same(monkeypatch):
    calls = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
    ])
    assert calls == [True]  # only once, on the first line


def test_gps_reader_fires_false_when_dgps_is_lost(monkeypatch):
    calls = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 1},
    ])
    assert calls == [True, False]


def test_gps_reader_ignores_rmc_lines_with_no_quality_field(monkeypatch):
    # An RMC fix's "quality" is always None (see parse_fix()'s docstring)
    # -- must never be treated as a transition either way.
    calls = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": 5.0, "cap": 90.0, "quality": None},
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
    ])
    assert calls == [True]


# --- GPSReader._loop(): is_dgps forwarded to RobotState.update_gps_fix() ---
# (2026-09-19, for the STA DGPS field -- see RobotState.is_dgps)

def test_gps_reader_forwards_is_dgps_none_for_rmc_only_fix(monkeypatch):
    calls, state = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": 5.0, "cap": 90.0, "quality": None},
    ], return_state=True)
    assert state.fixes[0][4] is None  # is_dgps


def test_gps_reader_forwards_is_dgps_true_for_a_dgps_gga_fix(monkeypatch):
    calls, state = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 2},
    ], return_state=True)
    assert state.fixes[0][4] is True


def test_gps_reader_forwards_is_dgps_false_for_a_non_dgps_gga_fix(monkeypatch):
    calls, state = _run_loop_with_fixes(monkeypatch, [
        {"lat": 1.0, "lon": 1.0, "speed_kmh": None, "cap": None, "quality": 1},
    ], return_state=True)
    assert state.fixes[0][4] is False
