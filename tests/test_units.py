import pytest

from byeol import units as u


def test_length_and_speed_conversions():
    assert u.feet_to_m(100.0) == pytest.approx(30.48)
    assert u.m_to_feet(30.48) == pytest.approx(100.0)
    assert u.knots_to_ms(10.0) == pytest.approx(5.14444, rel=1e-5)
    assert u.kmh_to_ms(36.0) == pytest.approx(10.0)
    assert u.mph_to_ms(10.0) == pytest.approx(4.4704)
    assert u.fpm_to_ms(1000.0) == pytest.approx(5.08)  # feet per minute (vertical speed)


def test_angles_and_mavlink_scaled_ints():
    assert u.deg_to_rad(180.0) == pytest.approx(3.141592653589793)
    assert u.mm_to_m(1500) == 1.5
    assert u.cms_to_ms(250) == 2.5  # MAVLink: cm/s
    assert u.cdeg_to_rad(9000) == pytest.approx(1.5707963267948966)  # hundredths of a degree


def test_to_si_dispatch_and_errors():
    assert u.to_si(100, "ft") == pytest.approx(30.48)
    assert u.to_si(10, "kt") == pytest.approx(5.14444, rel=1e-5)
    assert u.to_si(5, "m") == 5
    with pytest.raises(ValueError, match="unit"):
        u.to_si(1, "parsec")
