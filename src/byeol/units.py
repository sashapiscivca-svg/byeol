"""Units: byeol uses SI only internally (metres, seconds, radians, m/s). Converters for inputs from different regions and MAVLink integers."""
from __future__ import annotations

FOOT = 0.3048
KNOT = 1852.0 / 3600.0
MPH = 1609.344 / 3600.0


def feet_to_m(x: float) -> float:
    return x * FOOT


def m_to_feet(x: float) -> float:
    return x / FOOT


def knots_to_ms(x: float) -> float:
    return x * KNOT


def kmh_to_ms(x: float) -> float:
    return x / 3.6


def mph_to_ms(x: float) -> float:
    return x * MPH


def fpm_to_ms(x: float) -> float:
    """Feet per minute (vertical speed in aviation units) -> m/s."""
    return x * FOOT / 60.0


def deg_to_rad(x: float) -> float:
    import math

    return math.radians(x)


def mm_to_m(x: float) -> float:
    return x / 1000.0


def cms_to_ms(x: float) -> float:
    """MAVLink: velocities in cm/s (e.g. GLOBAL_POSITION_INT.vx) -> m/s."""
    return x / 100.0


def cdeg_to_rad(x: float) -> float:
    """MAVLink: heading in hundredths of a degree (GLOBAL_POSITION_INT.hdg) -> rad."""
    return deg_to_rad(x / 100.0)


_TABLE = {
    "m": lambda x: x, "ft": feet_to_m, "mm": mm_to_m,
    "m/s": lambda x: x, "kt": knots_to_ms, "kn": knots_to_ms, "km/h": kmh_to_ms, "mph": mph_to_ms,
    "ft/min": fpm_to_ms, "cm/s": cms_to_ms,
    "rad": lambda x: x, "deg": deg_to_rad, "cdeg": cdeg_to_rad,
}


def to_si(x: float, unit: str) -> float:
    try:
        return _TABLE[unit](x)
    except KeyError:
        raise ValueError(f"unknown unit {unit!r}; supported: {sorted(_TABLE)}") from None
