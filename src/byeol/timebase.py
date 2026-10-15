"""Time systems: UTC/Unix, GPS seconds, leap-second correction; linear camera<->IMU clock mapping.

Inside byeol, time is seconds of a single clock (usually the FC boot time). For logs with GPS time and for synchronising
the camera with the IMU: `gps_seconds_to_utc_unix`, `ClockMap`. The leap-second table must be updated from IERS Bulletin C
if new leap seconds are announced (the table follows the IERS TAI-UTC history; last entry 2017-01-01, TAI-UTC = 37 s).
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
from datetime import date

import numpy as np

GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06T00:00:00Z
# (effective date, TAI-UTC, s); GPS = TAI - 19 s
_TAI_UTC = [
    (date(1980, 1, 6), 19), (date(1981, 7, 1), 20), (date(1982, 7, 1), 21), (date(1983, 7, 1), 22),
    (date(1985, 7, 1), 23), (date(1988, 1, 1), 24), (date(1990, 1, 1), 25), (date(1991, 1, 1), 26),
    (date(1992, 7, 1), 27), (date(1993, 7, 1), 28), (date(1994, 7, 1), 29), (date(1996, 1, 1), 30),
    (date(1997, 7, 1), 31), (date(1999, 1, 1), 32), (date(2006, 1, 1), 33), (date(2009, 1, 1), 34),
    (date(2012, 7, 1), 35), (date(2015, 7, 1), 36), (date(2017, 1, 1), 37),
]
_DATES = [d for d, _ in _TAI_UTC]


def gps_minus_utc(year: int, month: int, day: int) -> int:
    k = bisect.bisect_right(_DATES, date(year, month, day)) - 1
    if k < 0:
        raise ValueError("date is before the GPS epoch (1980-01-06)")
    return _TAI_UTC[k][1] - 19


def _leap_at_unix(t_unix: float) -> int:
    d = date.fromordinal(date(1970, 1, 1).toordinal() + int(t_unix // 86400))
    return gps_minus_utc(d.year, d.month, d.day)


def utc_unix_to_gps_seconds(t_unix: float) -> float:
    return t_unix - GPS_EPOCH_UNIX + _leap_at_unix(t_unix)


def gps_seconds_to_utc_unix(t_gps: float) -> float:
    t = t_gps + GPS_EPOCH_UNIX
    return t - _leap_at_unix(t - 18)  # approximate at the leap boundary (±1 s during the transition second)


def gps_week_tow_to_seconds(week: int, tow: float) -> float:
    """GPS week and seconds of week -> seconds since the GPS epoch (does not handle the 1024-week rollover)."""
    return week * 604800.0 + tow


@dataclass(frozen=True)
class ClockMap:
    """t_other = scale * t_this + offset (least squares over event pairs, e.g. frame trigger ↔ IMU timestamp)."""

    scale: float
    offset: float
    residual_rms: float

    @classmethod
    def fit(cls, t_this: np.ndarray, t_other: np.ndarray) -> ClockMap:
        t_this, t_other = np.asarray(t_this, float), np.asarray(t_other, float)
        if len(t_this) < 2 or len(t_this) != len(t_other):
            raise ValueError("at least 2 pairs of time instants of equal length are required")
        A = np.stack([t_this, np.ones_like(t_this)], axis=1)
        (a, b), *_ = np.linalg.lstsq(A, t_other, rcond=None)
        res = t_other - (a * t_this + b)
        return cls(float(a), float(b), float(np.sqrt((res**2).mean())))

    def map(self, t: float) -> float:
        return self.scale * t + self.offset

    def inverse(self, t_other: float) -> float:
        return (t_other - self.offset) / self.scale
