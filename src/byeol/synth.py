"""Custom synthetic generator: drone trajectory -> noisy IMU + ground-truth pixel shift (via pinhole).

The physics is consistent: world acceleration a(t) is smooth (wobble, gusts), the drone tilt follows from the thrust direction
(body z axis = (g - a)/|g - a|), so the accelerometer reads f = R^T (a - g) = (0, 0, -T) with no drag terms. This is a simplified
model, not a quadcopter simulator (see D-005). Vibration is added to the IMU readings only (the gimbal damps it for the camera).
Modes: hover, linear, gust, vibration, mixed.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import camera as cam

G = 9.80665
_FINE = 1000.0  # Hz, dense ground-truth grid


@dataclass
class SynthConfig:
    mode: str = "hover"
    duration: float = 30.0
    seed: int = 0
    imu_rate: float = 200.0
    cam_rate: float = 30.0
    h0: float = 30.0  # initial height above ground/water, m
    gimbal_deg: float = 75.0
    gimbal_sweep_deg: float = 0.0  # amplitude of slow gimbal sweep (period 20 s)
    speed: float = 5.0  # m/s for linear/mixed
    # sensors
    gyro_noise: float = 3e-4  # rad/s/sqrt(Hz)
    gyro_bias: float = 0.01  # rad/s, std of the initial bias per axis
    gyro_bias_walk: float = 2e-4  # rad/s/sqrt(s)
    accel_noise: float = 4e-3  # m/s^2/sqrt(Hz)
    accel_bias: float = 0.05  # m/s^2 std per axis
    vib_gyro: float = 0.15  # rad/s, vibration amplitude in vibration/mixed
    vib_accel: float = 2.5  # m/s^2
    vib_hz: float = 70.0
    dropout_prob: float = 0.0  # probability of a dropout starting at a sample
    dropout_len: tuple[int, int] = (2, 10)  # dropout length, samples
    gust_count: int = 3
    gust_accel: float = 4.0  # m/s^2 gust peak
    wobble_accel: float = 0.35  # m/s^2 wobble amplitude in hover
    baro_noise: float = 0.3  # m
    baro_drift: float = 0.02  # m/s^0.5 (slow drift)
    vel_noise: float = 0.2  # m/s (flight controller velocity estimate, 10 Hz)
    yaw_noise: float = 0.03  # rad (FC heading, 5 Hz)
    warmup: float = 1.0  # s before the first frame


@dataclass
class Sequence:
    cfg: SynthConfig
    imu_t: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    frame_t: np.ndarray
    height_t: np.ndarray
    height: np.ndarray
    aux_vel_t: np.ndarray
    aux_vel: np.ndarray
    yaw_t: np.ndarray
    yaw: np.ndarray
    gyro_bias_true: np.ndarray
    _fine_t: np.ndarray = field(repr=False, default=None)
    _p: np.ndarray = field(repr=False, default=None)
    _v: np.ndarray = field(repr=False, default=None)
    _acc: object = field(repr=False, default=None)
    _yaw_fn: object = field(repr=False, default=None)

    # --- ground truth ---
    def pos(self, t: float) -> np.ndarray:
        return np.array([np.interp(t, self._fine_t, self._p[:, k]) for k in range(3)])

    def vel(self, t: float) -> np.ndarray:
        return np.array([np.interp(t, self._fine_t, self._v[:, k]) for k in range(3)])

    def R_wb(self, t: float) -> np.ndarray:
        return _attitude(np.atleast_2d(self._acc(np.array([t]))), np.array([self._yaw_fn(np.array([t]))[0]]))[0]

    def gimbal(self, t: float) -> float:
        c = self.cfg
        return float(np.deg2rad(c.gimbal_deg + c.gimbal_sweep_deg * np.sin(2 * np.pi * t / 20.0)))

    def height_true(self, t: float) -> float:
        return float(-self.pos(t)[2])

    def truth_shift(self, K: cam.Pinhole, i0: int, i1: int, pixel: np.ndarray | None = None) -> tuple[float, float]:
        """Ground-truth content shift between frames i0 and i1: ray -> ground point in world -> projection (independent of camera.ego_shift)."""
        px = K.center[0] if pixel is None else np.asarray(pixel, float)
        t0, t1 = float(self.frame_t[i0]), float(self.frame_t[i1])
        Rwc0 = self.R_wb(t0) @ cam.gimbal_R_bc(self.gimbal(t0))
        Rwc1 = self.R_wb(t1) @ cam.gimbal_R_bc(self.gimbal(t1))
        p0, p1 = self.pos(t0), self.pos(t1)
        ray = Rwc0 @ K.ray(px)[0]
        if ray[2] <= 1e-3:
            return float("nan"), float("nan")
        X = p0 + ray * (-p0[2] / ray[2])
        Pc = Rwc1.T @ (X - p1)
        if Pc[2] <= 1e-6:
            return float("nan"), float("nan")
        uv = K.project(Pc[None, :])[0]
        d = uv - px
        return float(d[0]), float(d[1])


def _attitude(a: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """R_wb (N,3,3) from world acceleration a (N,3) and heading yaw (N)."""
    zb = np.array([0.0, 0.0, G]) - a  # (g - a), g points down
    zb /= np.linalg.norm(zb, axis=1, keepdims=True)
    xc = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], axis=1)
    yb = np.cross(zb, xc)
    yb /= np.linalg.norm(yb, axis=1, keepdims=True)
    xb = np.cross(yb, zb)
    return np.stack([xb, yb, zb], axis=2)


def _vee(S: np.ndarray) -> np.ndarray:
    return np.stack([S[:, 2, 1], S[:, 0, 2], S[:, 1, 0]], axis=1)


def generate(cfg: SynthConfig) -> Sequence:
    rng = np.random.default_rng(cfg.seed)
    mode = cfg.mode
    if mode not in ("hover", "linear", "gust", "vibration", "mixed"):
        raise ValueError(f"unknown mode: {mode}")
    T = cfg.duration + cfg.warmup
    # --- world acceleration a(t): wobble + gusts ---
    nwob = 4
    freqs = rng.uniform(0.2, 1.2, (nwob, 3))
    phs = rng.uniform(0, 2 * np.pi, (nwob, 3))
    amps = cfg.wobble_accel * rng.uniform(0.3, 1.0, (nwob, 3)) / np.sqrt(nwob)
    amps[:, 2] *= 0.4
    gusts = []
    if mode in ("gust", "mixed"):
        for _ in range(cfg.gust_count):
            ang = rng.uniform(0, 2 * np.pi)
            gusts.append((rng.uniform(0.2 * T, 0.9 * T), rng.uniform(0.08, 0.2),
                          cfg.gust_accel * np.array([np.cos(ang), np.sin(ang), 0.0]) * rng.uniform(0.6, 1.0)))
    heading0 = rng.uniform(-np.pi, np.pi) if mode in ("linear", "mixed") else 0.0
    yaw_amp, yaw_f, yaw_ph = 0.08, rng.uniform(0.1, 0.3), rng.uniform(0, 2 * np.pi)

    def acc(t: np.ndarray) -> np.ndarray:
        t = np.atleast_1d(t)
        out = np.zeros((len(t), 3))
        for k in range(nwob):
            out += amps[k] * np.sin(2 * np.pi * freqs[k] * t[:, None] + phs[k])
        for tg, sg, ag in gusts:
            out += np.exp(-0.5 * ((t - tg) / sg) ** 2)[:, None] * ag
        return out

    def yaw_fn(t: np.ndarray) -> np.ndarray:
        t = np.atleast_1d(t)
        return heading0 + yaw_amp * np.sin(2 * np.pi * yaw_f * t + yaw_ph)

    # --- ground-truth integration on the dense grid ---
    ft = np.arange(0.0, T + 1e-9, 1.0 / _FINE)
    a = acc(ft)
    v0 = np.zeros(3)
    if mode in ("linear", "mixed"):
        v0 = cfg.speed * np.array([np.cos(heading0), np.sin(heading0), 0.0])
    dt = 1.0 / _FINE
    v = v0 + np.concatenate([np.zeros((1, 3)), np.cumsum(0.5 * (a[1:] + a[:-1]) * dt, axis=0)])
    p = np.array([0.0, 0.0, -cfg.h0]) + np.concatenate([np.zeros((1, 3)), np.cumsum(0.5 * (v[1:] + v[:-1]) * dt, axis=0)])
    R = _attitude(a, yaw_fn(ft))
    Rdot = (R[2:] - R[:-2]) / (2 * dt)
    omega = np.zeros((len(ft), 3))
    omega[1:-1] = _vee(np.einsum("nji,njk->nik", R[1:-1], Rdot))
    omega[0], omega[-1] = omega[1], omega[-2]
    f_spec = np.einsum("nji,nj->ni", R, a - np.array([0.0, 0.0, G]))  # R^T (a - g)

    # --- IMU ---
    imu_t = np.arange(cfg.warmup * 0.0, T, 1.0 / cfg.imu_rate)
    g_true = np.stack([np.interp(imu_t, ft, omega[:, k]) for k in range(3)], axis=1)
    f_true = np.stack([np.interp(imu_t, ft, f_spec[:, k]) for k in range(3)], axis=1)
    n = len(imu_t)
    b0 = rng.normal(0, cfg.gyro_bias, 3)
    bias = b0 + np.cumsum(rng.normal(0, cfg.gyro_bias_walk / np.sqrt(cfg.imu_rate), (n, 3)), axis=0)
    ba = rng.normal(0, cfg.accel_bias, 3)
    gyro = g_true + bias + rng.normal(0, cfg.gyro_noise * np.sqrt(cfg.imu_rate), (n, 3))
    accel = f_true + ba + rng.normal(0, cfg.accel_noise * np.sqrt(cfg.imu_rate), (n, 3))
    if mode in ("vibration", "mixed"):
        ph = rng.uniform(0, 2 * np.pi, (2, 3))
        w = 2 * np.pi * cfg.vib_hz
        gyro += cfg.vib_gyro * np.sin(w * imu_t[:, None] + ph[0])
        accel += cfg.vib_accel * np.sin(w * imu_t[:, None] + ph[1])
    if cfg.dropout_prob > 0:
        keep = np.ones(n, bool)
        i = 0
        while i < n:
            if rng.random() < cfg.dropout_prob:
                L = int(rng.integers(cfg.dropout_len[0], cfg.dropout_len[1] + 1))
                keep[i:i + L] = False
                i += L
            i += 1
        imu_t, gyro, accel, bias = imu_t[keep], gyro[keep], accel[keep], bias[keep]

    # --- frames and auxiliary streams ---
    frame_t = np.arange(cfg.warmup, T, 1.0 / cfg.cam_rate)
    height_t = np.arange(0.0, T, 0.1)
    hz = -np.stack([np.interp(height_t, ft, p[:, 2])], axis=1)[:, 0]
    drift = np.cumsum(rng.normal(0, cfg.baro_drift * np.sqrt(0.1), len(height_t)))
    height = hz + drift + rng.normal(0, cfg.baro_noise, len(height_t))
    aux_vel_t = np.arange(0.0, T, 0.1)
    aux_vel = np.stack([np.interp(aux_vel_t, ft, v[:, k]) for k in range(3)], axis=1)
    aux_vel = aux_vel + rng.normal(0, cfg.vel_noise, aux_vel.shape)
    yaw_t = np.arange(0.0, T, 0.2)
    yaw = yaw_fn(yaw_t) + rng.normal(0, cfg.yaw_noise, len(yaw_t))
    return Sequence(cfg, imu_t, gyro, accel, frame_t, height_t, height, aux_vel_t, aux_vel, yaw_t, yaw, bias,
                    ft, p, v, acc, yaw_fn)

