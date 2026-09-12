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
