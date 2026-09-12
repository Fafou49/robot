"""Tests for motor_control.gps_condition_logger._gga_quality().

Kept in its own file, separate from tests/test_gps_condition_logger.py:
pytest.importorskip() at module level skips collection of the WHOLE file
if the import fails, not just the tests that come after it -- putting
this here instead of in test_gps_condition_logger.py means that file's
own plain tests (including is_dgps_quality(), which needs no library at
all) still run in an environment without pynmea2 (e.g. this project's
dev sandbox), rather than being skipped along with the pynmea2-dependent
ones for no reason.

Honesty note (same as tests/test_gps_reader.py's parse_fix tests):
pynmea2 could not be installed in the sandbox this was written in (no
PyPI access there), so these are written against pynmea2's documented API
and standard NMEA sentence syntax (checksums computed by hand, see the
comment above each sentence) but have NOT actually been run against the
real library. Run `pytest tests/test_gga_quality.py` on a machine with
pynmea2 installed (e.g. the Pi, with requirements.txt) before trusting
this further than "it's written the way parse_fix() successfully was".
"""
import pytest

pynmea2 = pytest.importorskip("pynmea2")

from motor_control.gps_condition_logger import (  # noqa: E402 -- must follow importorskip
    _gga_quality, is_dgps_quality,
)


def test_gga_quality_ignores_unrelated_lines():
    assert _gga_quality("") is None
    assert _gga_quality("not a sentence at all") is None
    # A real RMC sentence -- valid NMEA, but doesn't carry a quality field.
    assert _gga_quality("$GPRMC,123519,A,4723.492,N,00044.340,W,2.0,284.5,230394,,,A*63") is None


def test_gga_quality_reads_dgps_indicator():
    # Checksum computed by hand (XOR of the bytes between $ and *), same
    # approach as tests/test_gps_reader.py.
    line = "$GPGGA,123519,4723.492,N,00044.340,W,2,08,0.9,15.2,M,45.0,M,,*62"
    assert _gga_quality(line) == 2
    assert is_dgps_quality(_gga_quality(line)) is True


def test_gga_quality_reads_non_dgps_indicator():
    line = "$GPGGA,123519,4723.492,N,00044.340,W,1,08,0.9,15.2,M,45.0,M,,*61"
    assert _gga_quality(line) == 1
    assert is_dgps_quality(_gga_quality(line)) is False


def test_gga_quality_handles_the_gn_talker_too():
    line = "$GNGGA,123519,4723.492,N,00044.340,W,2,08,0.9,15.2,M,45.0,M,,*7C"
    assert _gga_quality(line) == 2


def test_gga_quality_no_fix_yet_returns_zero_not_none():
    # Quality 0 ("no fix") is still a meaningful, parseable value -- must
    # not be confused with "not a GGA sentence" (None).
    line = "$GPGGA,123519,4723.492,N,00044.340,W,0,00,,,M,,M,,*48"
    assert _gga_quality(line) == 0
    assert is_dgps_quality(0) is False
