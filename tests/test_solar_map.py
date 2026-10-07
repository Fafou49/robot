"""Tests for link/solar_map.py's pure logic: the 5-metre buffering gate,
the tmp-file append/drain round trip, and the lat/lon grid-cell bucketing
-- no server/socket involved, same style as tests/test_autopilot.py and
tests/test_nmea.py for this project's other standalone-logic modules.

Run with:
    pytest
"""
import pytest

from link import solar_map


# --- should_record_point() --------------------------------------------------

def test_should_record_point_true_for_the_very_first_point():
    assert solar_map.should_record_point(None, 47.391533, -0.739000) is True


def test_should_record_point_false_under_5_metres():
    # ~1m north of the last buffered point.
    assert solar_map.should_record_point(
        (47.391533, -0.739000), 47.391542, -0.739000
    ) is False


def test_should_record_point_true_past_5_metres():
    # ~90m north of the last buffered point.
    assert solar_map.should_record_point(
        (47.391533, -0.739000), 47.392343, -0.739000
    ) is True


def test_should_record_point_true_right_at_the_threshold():
    # Picks two points exactly SOLAR_SURVEY_MIN_DISTANCE_M apart (south,
    # along a meridian, where 1 degree of latitude is close enough to a
    # fixed number of metres for this test's own tolerance) and checks
    # the boundary is inclusive (">="), not strict.
    from link.autopilot import haversine_distance_m

    lat0, lon0 = 47.391533, -0.739000
    # Binary-search a lat offset that lands within 1mm of exactly 5m --
    # avoids hand-deriving the degrees-per-metre constant at this
    # latitude, and stays correct if EARTH_RADIUS_M ever changes.
    lo, hi = 0.0, 0.001
    for _ in range(40):
        mid = (lo + hi) / 2
        d = haversine_distance_m(lat0, lon0, lat0 + mid, lon0)
        if d < solar_map.SOLAR_SURVEY_MIN_DISTANCE_M:
            lo = mid
        else:
            hi = mid
    lat1 = lat0 + hi
    assert solar_map.should_record_point((lat0, lon0), lat1, lon0) is True


# --- append_point() / drain_points() ----------------------------------------

def test_drain_points_on_a_file_that_does_not_exist_yet(tmp_path):
    assert solar_map.drain_points(str(tmp_path / "nope.jsonl")) == []


def test_append_then_drain_round_trip(tmp_path):
    path = str(tmp_path / "solar_survey_tmp.jsonl")
    solar_map.append_point(path, 1000.0, 47.1, -0.7, 12.5)
    solar_map.append_point(path, 1300.0, 47.2, -0.8, 18.25)

    points = solar_map.drain_points(path)
    assert points == [
        (1000, 47.1, -0.7, 12.5),
        (1300, 47.2, -0.8, 18.25),
    ]


def test_drain_points_truncates_rather_than_deletes(tmp_path):
    import os

    path = str(tmp_path / "solar_survey_tmp.jsonl")
    solar_map.append_point(path, 1000.0, 47.1, -0.7, 12.5)
    solar_map.drain_points(path)

    assert os.path.exists(path)
    assert solar_map.drain_points(path) == []  # nothing left to drain

    # A fresh append right after a drain must still work.
    solar_map.append_point(path, 2000.0, 47.3, -0.9, 5.0)
    assert solar_map.drain_points(path) == [(2000, 47.3, -0.9, 5.0)]


def test_drain_points_skips_malformed_lines_without_aborting(tmp_path):
    path = tmp_path / "solar_survey_tmp.jsonl"
    path.write_text(
        '{"ts": 1000, "lat": 47.1, "lon": -0.7, "pv_power": 12.5}\n'
        "not json at all\n"
        '{"ts": 1300, "lat": 47.2, "lon": -0.8, "pv_power": 18.25}\n'
    )
    points = solar_map.drain_points(str(path))
    assert points == [
        (1000, 47.1, -0.7, 12.5),
        (1300, 47.2, -0.8, 18.25),
    ]


def test_append_point_creates_parent_directories(tmp_path):
    path = str(tmp_path / "nested" / "dir" / "solar_survey_tmp.jsonl")
    solar_map.append_point(path, 1000.0, 47.1, -0.7, 12.5)
    assert solar_map.drain_points(path) == [(1000, 47.1, -0.7, 12.5)]


# --- resolve_tmp_path() -----------------------------------------------------

def test_resolve_tmp_path_uses_env_override(monkeypatch, tmp_path):
    override = str(tmp_path / "custom.jsonl")
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", override)
    assert solar_map.resolve_tmp_path() == override


def test_resolve_tmp_path_default_without_override(monkeypatch):
    monkeypatch.delenv("SOLAR_SURVEY_TMP_PATH", raising=False)
    assert solar_map.resolve_tmp_path() == solar_map.DEFAULT_TMP_PATH


# --- cell_key() / cell_center() ---------------------------------------------

def test_cell_key_buckets_nearby_points_together():
    # Two points much closer together than one cell's own size.
    key_a = solar_map.cell_key(47.391533, -0.739000)
    key_b = solar_map.cell_key(47.391534, -0.739001)
    assert key_a == key_b


def test_cell_key_separates_distant_points():
    key_a = solar_map.cell_key(47.391533, -0.739000)
    key_b = solar_map.cell_key(47.4, -0.8)
    assert key_a != key_b


def test_cell_key_handles_negative_longitude_consistently():
    # This project's own operating area is west of Greenwich (negative
    # longitude) -- same two nearby points, mirrored into the eastern
    # hemisphere, must bucket the same way (Python's `//` always floors,
    # including for negative operands).
    west_a = solar_map.cell_key(47.391533, -0.739000)
    west_b = solar_map.cell_key(47.391534, -0.739004)
    east_a = solar_map.cell_key(47.391533, 0.739000)
    east_b = solar_map.cell_key(47.391534, 0.739004)
    assert west_a == west_b
    assert east_a == east_b


def test_cell_center_round_trips_into_the_same_cell():
    key = solar_map.cell_key(47.391533, -0.739000)
    center_lat, center_lon = solar_map.cell_center(*key)
    assert solar_map.cell_key(center_lat, center_lon) == key


def test_cell_center_is_the_midpoint_of_the_cell():
    cell_lat_idx, cell_lon_idx = 1000, -2000
    center_lat, center_lon = solar_map.cell_center(cell_lat_idx, cell_lon_idx)
    assert center_lat == pytest.approx(
        (cell_lat_idx + 0.5) * solar_map.SOLAR_MAP_CELL_SIZE_DEG
    )
    assert center_lon == pytest.approx(
        (cell_lon_idx + 0.5) * solar_map.SOLAR_MAP_CELL_SIZE_DEG
    )
