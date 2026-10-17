"""Local sunrise calculation (NOAA Solar Calculator algorithm, public
domain -- the same formulas behind most sunrise/sunset calculators,
including the popular `astral` package) -- pure math, no third-party
dependency, no network call.

Used by link.camera_power_policy's reactive/eco camera decision
(2026-10-10, explicit user request) to know how many hours remain until
sunrise. Deliberately computed LOCALLY rather than via an online API
(e.g. sunrise-sunset.org, a reasonable alternative that was considered):
an outdoor robot's WiFi link is exactly the kind of thing that can be
flaky when the battery is already marginal -- precisely the moment this
calculation matters most. An online API would fail silently at the worst
possible time, right when "uncertainty -> eco" (the user's own stated
rule) most needs a real answer, not a network timeout. Accuracy here is
within a few minutes, more than enough for a coarse battery-reserve
safety margin -- the policy itself already works in 5-minute ticks with
an explicit SAFETY_MARGIN_HOURS buffer (see link/camera_power_policy.py).

Longitude convention matches the rest of this project (link.nmea's
decimal_to_nmea/nmea_to_decimal, RobotState.current_lat/current_lon):
positive = East, negative = West.

Honesty note: implemented carefully against the well-known NOAA Solar
Calculator formulas, and checked here against physical invariants that
hold for ANY date (so they can be verified without a live reference):
at the equator, sunrise is always close to 6:00 local solar time
(adjusted a few minutes earlier by atmospheric refraction); at either
equinox, sunrise is close to 12 hours before solar noon at every
latitude. Both are confirmed by tests/test_sunrise.py. This was NOT
cross-checked against a live reference API or a published almanac for a
specific city/date in this sandbox (no network access here) -- worth a
one-time sanity check against a known almanac value for the actual
deployment coordinates before trusting it operationally.
"""
import datetime
import math
import time

# 90 degrees (true horizon) + ~34' of atmospheric refraction + ~16' of
# solar angular radius -- the standard NOAA correction for the sun's
# apparent (not geometric) position at the moment it's seen to rise/set.
SUNRISE_SUNSET_ANGLE_DEG = 90.833


def _julian_day(dt_utc):
    """Julian Day Number (with fractional day) for a timezone-aware UTC
    datetime -- the standard Fliegel & Van Flandern algorithm."""
    y, m, d = dt_utc.year, dt_utc.month, dt_utc.day
    a = (14 - m) // 12
    yy = y + 4800 - a
    mm = m + 12 * a - 3
    jdn = d + (153 * mm + 2) // 5 + 365 * yy + yy // 4 - yy // 100 + yy // 400 - 32045
    day_fraction = (dt_utc.hour + dt_utc.minute / 60 + dt_utc.second / 3600) / 24
    return jdn + day_fraction - 0.5


def _julian_century(jd):
    return (jd - 2451545.0) / 36525.0


def _geom_mean_long_sun_deg(t):
    return (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360


def _geom_mean_anomaly_sun_deg(t):
    return 357.52911 + t * (35999.05029 - 0.0001537 * t)


def _eccent_earth_orbit(t):
    return 0.016708634 - t * (0.000042037 + 0.0000001267 * t)


def _sun_eq_of_center_deg(t):
    m = math.radians(_geom_mean_anomaly_sun_deg(t))
    return (math.sin(m) * (1.914602 - t * (0.004817 + 0.000014 * t))
            + math.sin(2 * m) * (0.019993 - 0.000101 * t)
            + math.sin(3 * m) * 0.000289)


def _sun_true_long_deg(t):
    return _geom_mean_long_sun_deg(t) + _sun_eq_of_center_deg(t)


def _sun_app_long_deg(t):
    omega = math.radians(125.04 - 1934.136 * t)
    return _sun_true_long_deg(t) - 0.00569 - 0.00478 * math.sin(omega)


def _mean_obliq_ecliptic_deg(t):
    seconds = 21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))
    return 23.0 + (26.0 + seconds / 60.0) / 60.0


def _obliq_corr_deg(t):
    omega = math.radians(125.04 - 1934.136 * t)
    return _mean_obliq_ecliptic_deg(t) + 0.00256 * math.cos(omega)


def _sun_declination_deg(t):
    e = math.radians(_obliq_corr_deg(t))
    lam = math.radians(_sun_app_long_deg(t))
    return math.degrees(math.asin(math.sin(e) * math.sin(lam)))


def _eq_of_time_minutes(t):
    epsilon = math.radians(_obliq_corr_deg(t))
    l0 = math.radians(_geom_mean_long_sun_deg(t))
    e = _eccent_earth_orbit(t)
    m = math.radians(_geom_mean_anomaly_sun_deg(t))
    y = math.tan(epsilon / 2) ** 2
    e_time = (y * math.sin(2 * l0) - 2 * e * math.sin(m)
              + 4 * e * y * math.sin(m) * math.cos(2 * l0)
              - 0.5 * y * y * math.sin(4 * l0) - 1.25 * e * e * math.sin(2 * m))
    return math.degrees(e_time) * 4


def _hour_angle_deg(jd, lat):
    """Shared half of the sunrise/sunset calculation: the hour angle (in
    degrees) between solar noon and the moment the sun crosses
    SUNRISE_SUNSET_ANGLE_DEG below the (apparent) horizon at `lat`, for
    the UT day whose midnight has this `jd`. Sunrise is solar noon minus
    this angle, sunset is solar noon plus this angle -- see
    sunrise_utc_minutes_of_day/sunset_utc_minutes_of_day below, which are
    otherwise identical. Returns (eq_time_minutes, ha_deg), or
    (eq_time_minutes, None) if the sun doesn't cross the horizon at all
    that day (polar day/night) -- the acos() argument would fall outside
    [-1, 1]."""
    t = _julian_century(jd)
    eq_time = _eq_of_time_minutes(t)
    solar_dec = _sun_declination_deg(t)
    lat_r = math.radians(lat)
    dec_r = math.radians(solar_dec)
    cos_ha = (
        math.cos(math.radians(SUNRISE_SUNSET_ANGLE_DEG)) / (math.cos(lat_r) * math.cos(dec_r))
        - math.tan(lat_r) * math.tan(dec_r)
    )
    if cos_ha < -1 or cos_ha > 1:
        return eq_time, None
    return eq_time, math.degrees(math.acos(cos_ha))


def sunrise_utc_minutes_of_day(jd, lat, lon):
    """Minutes after UTC midnight of the UT day containing `jd` (Julian
    Day at that midnight) that the sun rises at (lat, lon) in degrees
    (lon positive East, see this module's docstring). Can be negative or
    exceed 1440 near a UTC date boundary -- that's fine, the caller just
    adds it as an offset to that day's midnight (see next_sunrise_utc
    below) rather than treating it as a literal "minutes since 00:00"
    clock reading. Returns None if the sun doesn't cross the horizon at
    all that day (polar day/night) -- the acos() argument would fall
    outside [-1, 1]."""
    eq_time, ha_deg = _hour_angle_deg(jd, lat)
    if ha_deg is None:
        return None
    return 720 - 4 * (lon + ha_deg) - eq_time


def sunset_utc_minutes_of_day(jd, lat, lon):
    """Same as sunrise_utc_minutes_of_day above, but for sunset -- the
    hour angle is ADDED to solar noon instead of subtracted (sunset
    happens the same number of degrees past noon that sunrise happened
    before it). Returns None under the same polar day/night condition."""
    eq_time, ha_deg = _hour_angle_deg(jd, lat)
    if ha_deg is None:
        return None
    return 720 - 4 * (lon - ha_deg) - eq_time


def _next_solar_event_utc(lat, lon, after_ts, minutes_of_day_fn):
    """Shared walk for next_sunrise_utc/next_sunset_utc below: tries
    today then tomorrow (UTC calendar days), returns the first event
    strictly after `after_ts`, or None if neither day has one (polar
    day/night)."""
    if after_ts is None:
        after_ts = time.time()
    now = datetime.datetime.fromtimestamp(after_ts, tz=datetime.timezone.utc)
    for day_offset in (0, 1):
        day = (now + datetime.timedelta(days=day_offset)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        jd = _julian_day(day)
        minutes = minutes_of_day_fn(jd, lat, lon)
        if minutes is None:
            continue
        event_dt = day + datetime.timedelta(minutes=minutes)
        event_ts = event_dt.timestamp()
        if event_ts > after_ts:
            return event_ts
    return None


def next_sunrise_utc(lat, lon, after_ts=None):
    """Next sunrise at (lat, lon) strictly after `after_ts` (a Unix UTC
    timestamp, defaults to time.time()) as a Unix UTC timestamp --
    today's sunrise if it hasn't happened yet, otherwise tomorrow's.
    Returns None if the sun doesn't rise on either of the two days
    checked (polar day/night) -- the caller should treat that the same
    as "unknown" (see link.camera_power_policy's own "uncertainty ->
    eco" rule, which is exactly what a None return should trigger)."""
    return _next_solar_event_utc(lat, lon, after_ts, sunrise_utc_minutes_of_day)


def next_sunset_utc(lat, lon, after_ts=None):
    """Next sunset at (lat, lon) strictly after `after_ts` -- same
    semantics as next_sunrise_utc above (today's if still ahead,
    otherwise tomorrow's; None on polar day/night). Added 2026-10-12 for
    link.camera_power_policy's day/night determination (sunrise/sunset
    bracket the current moment to tell day from night without a clock-
    based guess)."""
    return _next_solar_event_utc(lat, lon, after_ts, sunset_utc_minutes_of_day)
