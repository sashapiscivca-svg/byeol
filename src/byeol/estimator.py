"""Streaming camera motion estimator: IMU + height + gimbal angle (+ FC velocity/heading) -> (dx, dy) in pixels.

Per-IMU-sample pipeline: complementary tilt filter (orientation.Mahony), gyroscope integration
with bias compensation (cumulative quaternion), velocity filter (velocity.VelocityKF). History is stored
in a ring buffer; on a `frame_shift(t0, t1)` query the relative camera rotation and translation between the times
of the two frames are taken, and `camera.ego_shift` gives the pixel shift; uncertainty via linear propagation.
Fully deterministic, numpy only. This module does NOT touch the flight control loop.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from . import camera as cam
from . import rotation as rot
from .cameramodel import CameraModel
from .orientation import Mahony
from .velocity import VelocityKF


class HeightSource(Enum):
    """Source of height above the surface (hardware not yet fixed, so it is a parameter, D-013)."""

    FUSED_FC = "FUSED_FC"  # flight controller's fused altitude (primary over water)
    RANGEFINDER = "RANGEFINDER"  # rangefinder/lidar (over ground, when available)
    BARO = "BARO"  # barometer with corrected surface level
    MANUAL = "MANUAL"  # constant height set by the operator


# Assumed std of absolute height error, m (at height_trust = 1); not measured, subject to calibration
HEIGHT_SIGMA_ABS = {HeightSource.FUSED_FC: 0.4, HeightSource.RANGEFINDER: 0.1, HeightSource.BARO: 1.0,
                    HeightSource.MANUAL: 2.0}


@dataclass
class EgoConfig:
    camera: CameraModel | cam.Pinhole
    height_source: HeightSource = HeightSource.FUSED_FC
    height_fallback: HeightSource | None = None  # e.g. FUSED_FC as a fallback for RANGEFINDER
    height_trust: float = 1.0  # (0, 1]: height std = HEIGHT_SIGMA_ABS[source] / trust
    height_max_age: float = 2.0  # s: primary-source height older than this is treated as missing
    require_velocity: bool = True  # D-012: without FC velocity valid=False (False is for research only)
    vel_max_age: float = 1.0  # s: maximum age of the latest FC velocity measurement
    # noise assumptions (to be estimated from vendor data/static recordings; see docs/TECHNICAL_REPORT.md)
    gyro_density: float = 3e-4  # rad/s/sqrt(Hz)
    bias_sigma: float = 0.003  # residual bias after convergence, rad/s
    bias_sigma0: float = 0.015  # initial (before convergence), rad/s
    bias_tau: float = 12.0  # s, convergence time constant of the bias estimate
    scale_sigma: float = 0.005  # gyroscope scale error (fraction)
    time_sigma: float = 0.003  # camera-IMU synchronisation error, s
    gap_rate_sigma: float = 0.3  # rad/s: unknown angular rate during a dropout
    tilt_sigma: float = 0.03  # rad: tilt error -> plane normal
    height_rel_sigma: float = 0.03
    vel_model_sigma: float = 0.05  # m/s: model inaccuracy (drift, wind over water, etc.)
    floor_px: float = 0.05  # uncertainty floor: rolling shutter, distortion, flat-ground assumption
    # operating mode
    max_gap: float = 0.03  # s: an IMU sample interval longer than this is treated as a dropout
    max_extrap: float = 0.05  # s: how far beyond the last sample extrapolation is allowed
    max_rate: float = 35.0  # rad/s: above this is saturation/outlier, the sample is rejected
    history_s: float = 3.0
    min_height: float = 1.0  # m
    height_tau: float = 1.0  # s, barometer smoothing
    cam_imu_offset: float = 0.0  # s: t_imu = t_camera + offset
    zero_vel_sigma: float | None = None  # weak "velocity ≈ 0" prior (hover) when FC velocity is unavailable
    pixels: np.ndarray | None = None  # reference pixels (N,2); None = frame centre
    # filter tuning
    mahony: dict = field(default_factory=dict)
    kf: dict = field(default_factory=dict)


@dataclass
class EgoResult:
    dx: float
    dy: float
    cov: np.ndarray  # 2x2, px^2
    conf: float  # 0..1 = 1/(1+sigma_px^2), sigma_px = sqrt(largest eigenvalue of cov)
    valid: bool
    reason: str = "ok"
    sigma: float = float("nan")  # sqrt(tr(cov)/2), px
    height_source: str = ""  # which height source was used

    def as_tuple(self) -> tuple[float, float, float]:
        """Same format as dal.ego.estimate_shift: (dx, dy, conf)."""
        return self.dx, self.dy, self.conf


def _invalid(reason: str) -> EgoResult:
    return EgoResult(float("nan"), float("nan"), np.eye(2) * 1e6, 0.0, False, reason, float("inf"))


class EgoEstimator:
    def __init__(self, cfg: EgoConfig) -> None:
        self.cfg = cfg
        if not 0.0 < cfg.height_trust <= 1.0:
            raise ValueError(f"height_trust must be in (0, 1], got {cfg.height_trust}")
        self.mahony = Mahony(**cfg.mahony)
        self.kf = VelocityKF(**cfg.kf)
        self.stats = {"imu": 0, "dropped": 0, "gaps": 0, "height_ignored": 0}
        # IMU sample history
        self._t: list[float] = []
        self._qc: list[np.ndarray] = []  # cumulative gyro-integration quaternion
        self._w: list[np.ndarray] = []  # effective angular rate of segment (t[k-1], t[k]], bias removed
        self._qa: list[np.ndarray] = []  # quaternion from Mahony
        self._v: list[np.ndarray] = []
        self._pv: list[np.ndarray] = []
        self._u: list[float] = []  # accumulated dropout time
        self._last_g: np.ndarray | None = None
        self._last_f: np.ndarray | None = None
        self._pend_yaw: float | None = None
        self._pend_vel: tuple[np.ndarray, float] | None = None
        self._heading_seen = False
        self._h: dict[HeightSource, list[tuple[float, float]]] = {}
        self._vel_ts: list[float] = []
        self._gim: list[tuple[float, float]] = []
        self._zv_counter = 0

    # ---------------- inputs ----------------
    def push_gimbal(self, t: float, theta: float) -> None:
        """Gimbal angle theta (rad): 0 = forward along the horizon, pi/2 = nadir."""
        self._gim.append((t, float(theta)))
        del self._gim[:-200]

    def push_height(self, t: float, h: float, source: HeightSource | None = None) -> None:
        """Height above the plane (water/ground), m, with the given source (defaults to the primary source from the config).

        Only samples from the primary and fallback sources are accepted; others are ignored (counter stats["height_ignored"])."""
        source = self.cfg.height_source if source is None else source
        if source not in (self.cfg.height_source, self.cfg.height_fallback):
            self.stats["height_ignored"] += 1
            return
        if not np.isfinite(h):
            return
        hist = self._h.setdefault(source, [])
        if hist:
            t0, h0 = hist[-1]
            a = max(t - t0, 0.0) / (self.cfg.height_tau + max(t - t0, 0.0))
            h = h0 + a * (h - h0)
        hist.append((t, float(h)))
        del hist[:-200]

    def _height_at(self, t: float) -> tuple[float, HeightSource] | None:
        for src in (self.cfg.height_source, self.cfg.height_fallback):
            hist = self._h.get(src) if src is not None else None
            if hist and hist[0][0] - self.cfg.height_max_age <= t <= hist[-1][0] + self.cfg.height_max_age:
                return self._lookup(hist, t), src
        return None

    def push_velocity(self, t: float, v_ned: np.ndarray, sigma: float = 0.2) -> None:
        """Velocity from the flight controller EKF, NED, m/s (mandatory input, D-012)."""
        self._vel_ts.append(float(t))
        del self._vel_ts[:-200]
        self._pend_vel = (np.asarray(v_ned, float), float(sigma))

    def push_heading(self, t: float, yaw: float) -> None:
        self._pend_yaw = float(yaw)
        self._heading_seen = True

    def push_imu(self, t: float, gyro: np.ndarray, accel: np.ndarray) -> None:
        g, a = np.asarray(gyro, float), np.asarray(accel, float)
        if not (np.isfinite(t) and np.all(np.isfinite(g)) and np.all(np.isfinite(a))) or np.abs(g).max() > self.cfg.max_rate:
            self.stats["dropped"] += 1
            return
        if self._t and t <= self._t[-1]:
            self.stats["dropped"] += 1
            return
        self.stats["imu"] += 1
        if not self._t:
            self.mahony.init_from_accel(a, yaw=self._pend_yaw if self._pend_yaw is not None else 0.0)
            self._append(t, np.array([1.0, 0, 0, 0]), np.zeros(3), 0.0, g, a)
            return
        dt = t - self._t[-1]
        gap = dt > self.cfg.max_gap
        if gap:
            self.stats["gaps"] += 1
            g_int, a_int = self._last_g, self._last_f  # hold the last values
        else:
            g_int, a_int = 0.5 * (self._last_g + g), a
        b = self.mahony.bias.copy()
        w_eff = g_int - b
        qc = rot.q_normalize(rot.q_mul(self._qc[-1], rot.q_exp(w_eff * dt)))
        self.mahony.update(dt, g_int, a_int, yaw=self._pend_yaw)
        self.kf.predict(dt, self.mahony.R, a_int)
        if self._pend_vel is not None:
            self.kf.update_velocity(*self._pend_vel)
            self._pend_vel = None
            self._zv_counter = 0
        elif self.cfg.zero_vel_sigma is not None:
            self._zv_counter += 1
            if self._zv_counter % 20 == 0:
                self.kf.update_velocity(np.zeros(3), self.cfg.zero_vel_sigma)
        self._append(t, qc, w_eff, self._u[-1] + (dt if gap else 0.0), g, a)

    def _append(self, t, qc, w, u, g, a) -> None:
        self._t.append(t)
        self._qc.append(qc)
        self._w.append(w)
        self._qa.append(self.mahony.q.copy())
        self._v.append(self.kf.v.copy())
        self._pv.append(self.kf.P[:3, :3].copy())
        self._u.append(u)
        self._last_g, self._last_f = g, a
        cap = int(self.cfg.history_s * 400)
        if len(self._t) > 2 * cap:
            for lst in (self._t, self._qc, self._w, self._qa, self._v, self._pv, self._u):
                del lst[:-cap]

    # ---------------- interpolation ----------------
    def _q_cum(self, t: float) -> tuple[np.ndarray, float]:
        """Cumulative quaternion at time t and the extrapolation time (s)."""
        k = bisect.bisect_right(self._t, t) - 1
        if k >= len(self._t) - 1:
            dt = t - self._t[-1]
            w = self._w[-1]
            return rot.q_mul(self._qc[-1], rot.q_exp(w * dt)), max(dt, 0.0)
        return rot.q_mul(self._qc[k], rot.q_exp(self._w[k + 1] * (t - self._t[k]))), 0.0

    def _interp_vec(self, t: float, lst: list) -> np.ndarray:
        k = bisect.bisect_right(self._t, t) - 1
        if k >= len(self._t) - 1:
            return lst[-1]
        a = (t - self._t[k]) / (self._t[k + 1] - self._t[k])
        return lst[k] * (1 - a) + lst[k + 1] * a

    def _q_att(self, t: float) -> np.ndarray:
        k = bisect.bisect_right(self._t, t) - 1
        if k >= len(self._t) - 1:
            return self._qa[-1]
        a = (t - self._t[k]) / (self._t[k + 1] - self._t[k])
        d = rot.q_log(rot.q_mul(rot.q_conj(self._qa[k]), self._qa[k + 1]))
        return rot.q_mul(self._qa[k], rot.q_exp(d * a))

    @staticmethod
    def _lookup(hist: list[tuple[float, float]], t: float) -> float | None:
        if not hist:
            return None
        ts = [x[0] for x in hist]
        k = bisect.bisect_right(ts, t) - 1
        if k < 0:
            return hist[0][1]
        if k >= len(hist) - 1:
            return hist[-1][1]
        a = (t - ts[k]) / (ts[k + 1] - ts[k]) if ts[k + 1] > ts[k] else 0.0
        return hist[k][1] * (1 - a) + hist[k + 1][1] * a

    # ---------------- query ----------------
    def frame_shift(self, t0: float, t1: float) -> EgoResult:
        c = self.cfg
        t0, t1 = t0 + c.cam_imu_offset, t1 + c.cam_imu_offset
        if len(self._t) < 3 or t0 < self._t[0] or t1 <= t0:
            return _invalid("no_imu")
        if t1 - self._t[-1] > c.max_extrap:
            return _invalid("no_imu")
        hs = self._height_at(t0)
        if hs is None:
            return _invalid("no_height")
        h0, hsrc = hs
        if h0 < c.min_height:
            return _invalid("low_height")
        th0, th1 = self._lookup(self._gim, t0), self._lookup(self._gim, t1)
        if th0 is None:
            return _invalid("no_gimbal")
        if c.require_velocity:
            k = bisect.bisect_right(self._vel_ts, t1) - 1
            if not self._vel_ts:
                return _invalid("no_velocity")
            if k < 0 or t1 - self._vel_ts[k] > c.vel_max_age:
                return _invalid("stale_velocity")
        dt = t1 - t0
        q0, _ = self._q_cum(t0)
        q1, extra = self._q_cum(t1)
        dq = rot.q_mul(rot.q_conj(q0), q1)
        dRb = rot.q_to_R(dq)
        k0, k1 = bisect.bisect_right(self._t, t0) - 1, min(bisect.bisect_right(self._t, t1) - 1, len(self._t) - 1)
        unobs = self._u[k1] - self._u[max(k0, 0)] + extra
        Rbc0, Rbc1 = cam.gimbal_R_bc(th0), cam.gimbal_R_bc(th1)
        R_wb0 = rot.q_to_R(self._q_att(t0))
        R_wc0 = R_wb0 @ Rbc0
        # camera translation: integral of velocity (trapezoid) in world frame
        ts = [t0] + [x for x in self._t[max(k0, 0) + 1:k1 + 1] if t0 < x < t1] + [t1]
        vs = np.array([self._interp_vec(x, self._v) for x in ts])
        dp_w = np.trapezoid(vs, ts, axis=0)
        t01 = R_wc0.T @ dp_w
        R01 = Rbc0.T @ dRb @ Rbc1
        n, d = cam.ground_plane_in_camera(R_wb0, Rbc0, h0)
        px = c.camera.center if c.pixels is None else np.asarray(c.pixels, float)
        s = cam.ego_shift(c.camera, R01, t01, n, d, px)
        if not np.isfinite(s).all():
            return _invalid("horizon")
        shift = s.mean(axis=0)
        cov = self._covariance(R01, t01, n, d, px, shift, dt, dq, Rbc1, R_wc0, ts[0], unobs, h0, hsrc)
        lam = float(np.linalg.eigvalsh(cov)[-1])
        sig = float(np.sqrt(np.trace(cov) / 2))
        conf = 1.0 / (1.0 + lam)
        return EgoResult(float(shift[0]), float(shift[1]), cov, conf, True, "ok", sig, hsrc.value)

    def _covariance(self, R01, t01, n, d, px, shift, dt, dq, Rbc1, R_wc0, t_ref, unobs, h0, hsrc) -> np.ndarray:
        c = self.cfg
        omega = rot.q_log(dq) / dt
        run = self._t[-1] - self._t[0]
        sb = c.bias_sigma + c.bias_sigma0 * np.exp(-run / c.bias_tau)
        sbz = sb if self._heading_seen else max(sb, c.bias_sigma0)
        var_b = np.array([sb**2, sb**2, sbz**2])
        var = c.gyro_density**2 * dt + var_b * dt**2 + (c.scale_sigma * omega * dt) ** 2 \
            + (c.time_sigma * omega) ** 2 + (c.gap_rate_sigma * unobs) ** 2
        S_rot = Rbc1.T @ np.diag(var) @ Rbc1
        k = max(bisect.bisect_right(self._t, t_ref) - 1, 0)
        S_t = R_wc0.T @ (dt**2 * (self._pv[k] + np.eye(3) * c.vel_model_sigma**2)) @ R_wc0
        sh = np.sqrt(c.height_rel_sigma**2 + (HEIGHT_SIGMA_ABS[hsrc] / c.height_trust / h0) ** 2)
        S_tilt = c.tilt_sigma**2 * (np.eye(3) - np.outer(n, n))
        S = np.zeros((10, 10))
        S[0:3, 0:3], S[3:6, 3:6], S[6, 6], S[7:10, 7:10] = S_rot, S_t, sh**2, S_tilt

        def f(p: np.ndarray) -> np.ndarray:
            n2 = rot.rotvec_to_R(p[7:10]) @ n
            out = cam.ego_shift(c.camera, R01 @ rot.rotvec_to_R(p[0:3]), t01 + p[3:6], n2 / np.linalg.norm(n2),
                                d * (1 + p[6]), px)
            return out.mean(axis=0)

        J = np.zeros((2, 10))
        eps = 1e-5
        for i in range(10):
            p = np.zeros(10)
            p[i] = eps
            J[:, i] = (f(p) - shift) / eps
        return J @ S @ J.T + np.eye(2) * c.floor_px**2
