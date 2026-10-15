"""Synthetic "water + person" scene with target ground truth (D-020): an extension of `vision_baseline.WaterRenderer` without changing its API.

Added to the water frame: (1) a wave layer, a second texture drifting with the current relative to the "bottom" (phase correlation sees
it as spurious motion, physics does not); (2) glints, short-lived bright spots, random in every frame; (3) the target, a dark/bright
blob with a physical size in metres (apparent size = f·size/depth) that moves slowly and smoothly in the world.
Target ground truth: centre in pixels (world position projected through the camera pose), box (cx, cy, d, d), presence flag
(centre inside the frame with a margin). Synthetic data from our own generator only: no real waves, foam or distortion here.
Requires cv2 (as does `vision_baseline`).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import camera as cam
from .synth import Sequence
from .vision_baseline import WaterRenderer, _cv2


@dataclass
class Target:
    """Target on water: position X0 (world, NED, m) at time t_ref, slow motion (velocity + wobble)."""

    X0: np.ndarray
    t_ref: float
    size_m: float = 1.2
    amp: float = 6.0          # intensity levels (0..255) at the peak
    polarity: int = -1        # -1 dark, +1 bright
    vel: np.ndarray = None    # m/s in world (N, E)
    sway_m: float = 0.3
    sway_period: float = 7.0
    sway_phase: float = 0.0

    def pos(self, t: float) -> np.ndarray:
        dt = t - self.t_ref
        v = np.zeros(2) if self.vel is None else np.asarray(self.vel, float)
        w = 2 * np.pi / self.sway_period
        sw = self.sway_m * (np.sin(w * t + self.sway_phase) - np.sin(w * self.t_ref + self.sway_phase))
        p = self.X0.copy()
        p[0] += v[0] * dt
        p[1] += v[1] * dt + sw
        return p


def camera_pose(seq: Sequence, t: float) -> tuple[np.ndarray, np.ndarray]:
    """Camera (R_wc, p) at time t (generator ground truth)."""
    return seq.R_wb(t) @ cam.gimbal_R_bc(seq.gimbal(t)), seq.pos(t)


def place_target(seq: Sequence, K, t_ref: float, pixel=(320.0, 180.0), rng: np.random.Generator | None = None,
                 speed: float = 0.4, **kw) -> Target:
    """Target at the ground point seen by the camera at `pixel` at time t_ref; random heading (rng), speed `speed` m/s."""
    rng = rng if rng is not None else np.random.default_rng(0)
    R, p = camera_pose(seq, t_ref)
    ray = R @ K.ray(np.asarray(pixel, float))[0]
    if ray[2] <= 1e-3:
        raise ValueError("pixel looks above the horizon")
    X = p + ray * (-p[2] / ray[2])
    ang = rng.uniform(0, 2 * np.pi)
    return Target(X0=X, t_ref=t_ref, vel=speed * np.array([np.cos(ang), np.sin(ang)]),
                  sway_phase=float(rng.uniform(0, 2 * np.pi)), **kw)


def target_truth(seq: Sequence, K, tgt: Target, i: int, margin: float = 12.0) -> dict:
    """Target ground truth in frame i: centre, apparent diameter, depth, presence flag."""
    t = float(seq.frame_t[i])
    R, p = camera_pose(seq, t)
    Pc = R.T @ (tgt.pos(t) - p)
    if Pc[2] <= 1e-3:
        return {"center": np.array([np.nan, np.nan]), "d": np.nan, "depth": float(Pc[2]), "present": False}
    uv = K.project(Pc[None, :])[0]
    d = K.f * tgt.size_m / Pc[2]
    w, h = K.width, K.height
    inside = bool(margin <= uv[0] <= w - margin and margin <= uv[1] <= h - margin)
    return {"center": uv, "d": float(d), "depth": float(Pc[2]), "present": inside}


class WaterScene(WaterRenderer):
    """Water with waves and glints + target. `contrast`: std of the bottom texture intensity (as in WaterRenderer)."""

    def __init__(self, K, size: tuple[int, int], contrast: float = 0.5, sensor_noise: float = 1.5, seed: int = 0,
                 wave_frac: float = 0.5, wave_vel: tuple[float, float] = (0.0, 1.0), glint_rate: float = 4.0,
                 glint_amp: float = 25.0, m_per_px: float = 0.1) -> None:
        super().__init__(K, size, contrast=contrast, sensor_noise=sensor_noise, seed=seed, m_per_px=m_per_px)
        cv2 = _cv2()
        rng = np.random.default_rng(seed + 11)
        n = 2048
        w = cv2.GaussianBlur(rng.normal(0, 1, (n, n)).astype(np.float32), (0, 0), 5.0)
        self.wave = (contrast * w / w.std()).astype(np.float32)          # zero mean, std = contrast
        self.wave_frac, self.wave_vel = wave_frac, np.asarray(wave_vel, float)
        self.glint_rate, self.glint_amp = glint_rate, glint_amp

    def _coords(self, seq: Sequence, t: float, shift: np.ndarray | None = None):
        R_wc, p = camera_pose(seq, t)
        rw = self.rays @ R_wc.T
        s = -p[2] / np.maximum(rw[:, 2], 1e-3)
        X = p[None, :] + rw * s[:, None]
        if shift is not None:
            X = X - np.array([shift[0], shift[1], 0.0])[None, :]
        shp = (self.size[1], self.size[0])
        mu = (X[:, 1] / self.res + 1024).reshape(shp).astype(np.float32)
        mv = (X[:, 0] / self.res + 1024).reshape(shp).astype(np.float32)
        return mu, mv

    def render_scene(self, seq: Sequence, i: int, tgt: Target | None = None) -> tuple[np.ndarray, dict | None]:
        cv2 = _cv2()
        t = float(seq.frame_t[i])
        mu, mv = self._coords(seq, t)
        img = cv2.remap(self.tex, mu, mv, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        if self.wave_frac > 0:
            mu2, mv2 = self._coords(seq, t, shift=self.wave_vel * t)
            img = img + self.wave_frac * cv2.remap(self.wave, mu2, mv2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        gt = None
        if tgt is not None:
            gt = target_truth(seq, self.K, tgt, i)
            if np.isfinite(gt["d"]):
                _draw_blob(img, gt["center"], gt["d"], tgt.polarity * tgt.amp)
        if self.glint_rate > 0:
            for _ in range(self.rng.poisson(self.glint_rate)):
                gx, gy = self.rng.uniform(0, self.size[0]), self.rng.uniform(0, self.size[1])
                _draw_blob(img, (gx, gy), self.rng.uniform(2.0, 5.0), self.glint_amp * self.rng.uniform(0.5, 1.0))
        if self.noise > 0:
            img = img + self.rng.normal(0, self.noise, img.shape).astype(np.float32)
        return np.clip(np.round(img), 0, 255).astype(np.float32), gt


def _draw_blob(img: np.ndarray, c, d: float, amp: float) -> None:
    """Adds a soft disc of diameter d (pixel i covers [i, i+1), as in dal.crop_at) with peak amp to img (in place)."""
    R = max(d / 2.0, 0.75)
    x0, x1 = int(max(np.floor(c[0] - R - 2), 0)), int(min(np.ceil(c[0] + R + 2), img.shape[1]))
    y0, y1 = int(max(np.floor(c[1] - R - 2), 0)), int(min(np.ceil(c[1] + R + 2), img.shape[0]))
    if x1 <= x0 or y1 <= y0:
        return
    yy, xx = np.mgrid[y0:y1, x0:x1]
    r = np.hypot(xx + 0.5 - c[0], yy + 0.5 - c[1])
    img[y0:y1, x0:x1] += (amp * np.clip((R - r) / 1.0 + 0.5, 0.0, 1.0)).astype(np.float32)
