import numpy as np
import pytest

from byeol import timebase as tb


def test_leap_seconds_known_values():
    assert tb.gps_minus_utc(1989, 6, 1) == 5  # TAI-UTC = 24, GPS = TAI - 19
    assert tb.gps_minus_utc(1990, 6, 1) == 6
    assert tb.gps_minus_utc(2017, 1, 1) == 18
    assert tb.gps_minus_utc(2026, 10, 6) == 18
    assert tb.gps_minus_utc(1980, 1, 6) == 0


def test_gps_seconds_utc_roundtrip_known_epoch():
    # 2017-01-01T00:00:00 UTC = 1167264018 s since the GPS epoch (1980-01-06); GPS is 18 s ahead
    assert tb.utc_unix_to_gps_seconds(1483228800.0) == pytest.approx(1167264018.0)
    assert tb.gps_seconds_to_utc_unix(1167264018.0) == pytest.approx(1483228800.0)


def test_gps_week_tow():
    assert tb.gps_week_tow_to_seconds(0, 0.0) == 0.0
    assert tb.gps_week_tow_to_seconds(2, 3600.0) == 2 * 604800 + 3600.0


def test_gps_epoch_is_unix_315964800():
    assert tb.utc_unix_to_gps_seconds(315964800.0) == pytest.approx(0.0)


def test_clock_map_fits_offset_and_skew():
    t_imu = np.linspace(0, 100, 50)
    t_cam = 1.00002 * t_imu + 0.350
    rng = np.random.default_rng(0)
    cm = tb.ClockMap.fit(t_imu, t_cam + rng.normal(0, 1e-4, 50))
    assert cm.scale == pytest.approx(1.00002, abs=2e-6) and cm.offset == pytest.approx(0.350, abs=2e-3)
    assert cm.map(10.0) == pytest.approx(1.00002 * 10 + 0.35, abs=2e-3)
    assert cm.inverse(cm.map(42.0)) == pytest.approx(42.0)
    assert cm.residual_rms < 3e-4


def test_clock_map_needs_enough_points():
    with pytest.raises(ValueError, match="at least"):
        tb.ClockMap.fit(np.array([0.0]), np.array([1.0]))
