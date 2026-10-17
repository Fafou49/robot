"""Tests for link/sunrise.py's local (no-network) sunrise calculation --
pure math, no dependencies, same style as tests/test_solar_map.py for
this project's other standalone-logic modules.

Rather than asserting exact minute-of-day values against a specific
city/date (this sandbox has no network access to cross-check against a
live almanac, and a memorized "known" sunrise time for an arbitrary date
would itself be unverifiable), these tests check PHYSICAL INVARIANTS
that hold for any date: at the equator, day length is always ~12h
(sunrise ~6:00 local solar time, refraction-adjusted a few minutes
earlier) regardless of season; at either equinox, sunrise is ~12h before
solar noon at EVERY latitude (the sun's declination is ~0, so the
day/night split is even everywhere). Both are well-known, date-
independent facts, not specific lookups. See link/sunrise.py's own
docstring for a manual sanity check this was already run against for
Paris (48.85N, 2.35E) summer/winter solstices, which also looked right
(early-June sunrise ~5:45 local, late-December ~8:40 local) but isn't
asserted here, for the above reason.

Run with:
    pytest tests/test_sunrise.py
"""
import datetime

import pytest

from link import sunrise


def _minutes_for(month, day, lat, lon, year=2026):
    d = datetime.datetime(year, month, day, tzinfo=datetime.timezone.utc)
    jd = sunrise._julian_day(d)
    return sunrise.sunrise_utc_minutes_of_day(jd, lat, lon)


# --- sunrise_utc_minutes_of_day: physical invariants -------------------

@pytest.mark.parametrize("month,day", [(3, 20), (6, 21), (9, 23), (12, 21)])
def test_equator_sunrise_is_always_close_to_6am_local_solar_time(month, day):
    # lon=0 means UTC time IS local solar time here -- day length at the
    # equator stays ~12h year-round, so sunrise should land within a
    # loose +/-15 minute band of 6:00 (356 minutes, i.e. 6:00 minus the
    # ~4 minute refraction advance) regardless of season.
    minutes = _minutes_for(month, day, lat=0.0, lon=0.0)
    assert minutes == pytest.approx(356, abs=15)


@pytest.mark.parametrize("lat", [0.0, 20.0, 40.0, 48.85, 60.0, -33.9])
def test_equinox_sunrise_is_always_close_to_12h_before_solar_noon(lat):
    # At the March equinox, solar declination is ~0 everywhere, so every
    # latitude (that isn't polar) gets an even day/night split -- sunrise
    # should land within a loose +/-15 minute band of solar-noon-minus-
    # 12h. Using lon=0 so "solar noon" is simply 720 minutes (12:00 UTC)
    # minus the equation-of-time correction, which is small (~single-
    # digit minutes) in March.
    minutes = _minutes_for(3, 20, lat=lat, lon=0.0)
    assert minutes == pytest.approx(356, abs=15)


def test_longer_days_in_the_hemisphere_s_own_summer():
    # Northern hemisphere, summer solstice: sunrise should be
    # significantly EARLIER than at the equinox (long day). Southern
    # hemisphere (negative latitude): the reverse -- June is ITS winter,
    # so sunrise should be LATER than at the equinox. Comparing against
    # the same latitude's own equinox value (rather than a fixed minute
    # count) is what makes this assertion date/location-independent.
    equinox_north = _minutes_for(3, 20, lat=48.85, lon=0.0)
    solstice_north = _minutes_for(6, 21, lat=48.85, lon=0.0)
    assert solstice_north < equinox_north  # earlier sunrise in summer

    equinox_south = _minutes_for(3, 20, lat=-33.9, lon=0.0)
    solstice_south = _minutes_for(6, 21, lat=-33.9, lon=0.0)
    assert solstice_south > equinox_south  # later sunrise in (southern) winter


def test_longitude_shifts_sunrise_by_4_minutes_per_degree():
    # A location further EAST sees the sun rise EARLIER in UTC terms
    # (the Earth doesn't have to rotate as far for that meridian to catch
    # up to the sun) -- exactly 4 minutes of time per degree of
    # longitude, a fixed conversion factor independent of date/latitude.
    minutes_greenwich = _minutes_for(3, 20, lat=48.85, lon=0.0)
    minutes_10_east = _minutes_for(3, 20, lat=48.85, lon=10.0)
    assert minutes_10_east == pytest.approx(minutes_greenwich - 40, abs=0.5)


def test_polar_summer_returns_none_sun_never_sets():
    # High-latitude, summer solstice: the midnight-sun case -- the sun
    # never dips below the refraction-adjusted horizon, so there is no
    # sunrise to report that day. Must return None, not raise (the
    # acos() argument falls outside [-1, 1] -- this is the one condition
    # sunrise_utc_minutes_of_day() exists specifically to catch).
    minutes = _minutes_for(6, 21, lat=78.0, lon=0.0)  # above the Arctic Circle
    assert minutes is None


def test_polar_winter_returns_none_sun_never_rises():
    minutes = _minutes_for(12, 21, lat=78.0, lon=0.0)
    assert minutes is None


# --- next_sunrise_utc(): day-rollover + "strictly after" semantics -----

def test_next_sunrise_utc_returns_todays_sunrise_when_still_ahead():
    midnight = datetime.datetime(2026, 3, 20, tzinfo=datetime.timezone.utc)
    result = sunrise.next_sunrise_utc(48.85, 2.35, after_ts=midnight.timestamp())
    result_dt = datetime.datetime.fromtimestamp(result, tz=datetime.timezone.utc)
    assert result_dt.date() == midnight.date()


def test_next_sunrise_utc_rolls_over_to_tomorrow_once_todays_has_passed():
    # Just after noon UTC on a given day -- today's sunrise (early
    # morning) has already passed, so this must report TOMORROW's.
    noon = datetime.datetime(2026, 3, 20, 12, 0, 0, tzinfo=datetime.timezone.utc)
    result = sunrise.next_sunrise_utc(48.85, 2.35, after_ts=noon.timestamp())
    result_dt = datetime.datetime.fromtimestamp(result, tz=datetime.timezone.utc)
    assert result_dt.date() == datetime.date(2026, 3, 21)


def test_next_sunrise_utc_is_strictly_after_the_given_timestamp():
    midnight = datetime.datetime(2026, 3, 20, tzinfo=datetime.timezone.utc)
    result = sunrise.next_sunrise_utc(48.85, 2.35, after_ts=midnight.timestamp())
    assert result > midnight.timestamp()


def test_next_sunrise_utc_defaults_to_now_when_after_ts_omitted(monkeypatch):
    fixed_now = datetime.datetime(2026, 3, 20, 0, 0, 0, tzinfo=datetime.timezone.utc).timestamp()
    monkeypatch.setattr(sunrise.time, "time", lambda: fixed_now)
    with_default = sunrise.next_sunrise_utc(48.85, 2.35)
    explicit = sunrise.next_sunrise_utc(48.85, 2.35, after_ts=fixed_now)
    assert with_default == explicit


def test_next_sunrise_utc_returns_none_through_a_polar_night_stretch():
    # Both the given day AND the next are polar night at this latitude in
    # December -- no sunrise to report either way.
    midwinter = datetime.datetime(2026, 12, 21, tzinfo=datetime.timezone.utc)
    result = sunrise.next_sunrise_utc(78.0, 0.0, after_ts=midwinter.timestamp())
    assert result is None
