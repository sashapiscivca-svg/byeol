"""Image-based baseline: synthetic water frames + OpenCV phase correlation (evaluation only; requires cv2).

The renderer places a random texture on the ground plane (low contrast = water) and images it with a pinhole camera at
the ground-truth poses of the sequence. phase_shift reproduces the logic of dal.ego.estimate_shift (reimplemented,
without depending on dal): Hann window, cv2.phaseCorrelate.
"""
from __future__ import annotations

import numpy as np

from . import camera as cam
from .synth import Sequence


def _cv2():
    import cv2
    return cv2


def phase_shift(prev: np.ndarray, cur: np.ndarray, scale: float = 1.0) -> tuple[float, float, float]:
    cv2 = _cv2()
    a, b = prev.astype(np.float32), cur.astype(np.float32)
    if scale != 1.0:
        a = cv2.resize(a, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        b = cv2.resize(b, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(a, b, win)
    return float(dx / scale), float(dy / scale), float(resp)


class WaterRenderer:
    """2048x2048 texture with 0.1 m spacing; contrast: std of texture intensity; sensor_noise: per-frame (independent) noise."""

    def __init__(self, K: cam.Pinhole, size: tuple[int, int], contrast: float = 1.5, sensor_noise: float = 1.5,
                 seed: int = 0, m_per_px: float = 0.1) -> None:
        cv2 = _cv2()
        self.K, self.size, self.noise, self.res = K, size, sensor_noise, m_per_px
        rng = np.random.default_rng(seed)
        n = 2048
        coarse = cv2.GaussianBlur(rng.normal(0, 1, (n, n)).astype(np.float32), (0, 0), 12)
        fine = cv2.GaussianBlur(rng.normal(0, 1, (n, n)).astype(np.float32), (0, 0), 2.0)
        tex = 0.6 * coarse / coarse.std() + 0.8 * fine / fine.std()
        self.tex = (128 + contrast * tex / tex.std()).astype(np.float32)
        w, h = size
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        self.rays = np.stack([(u - K.cx) / K.f, (v - K.cy) / K.f, np.ones_like(u)], axis=-1).reshape(-1, 3)
        self.rng = np.random.default_rng(seed + 1)

    def render(self, seq: Sequence, i: int) -> np.ndarray:
        cv2 = _cv2()
        t = float(seq.frame_t[i])
        R_wc = seq.R_wb(t) @ cam.gimbal_R_bc(seq.gimbal(t))
        p = seq.pos(t)
        rw = self.rays @ R_wc.T
        s = -p[2] / np.maximum(rw[:, 2], 1e-3)
        X = p[None, :] + rw * s[:, None]
        mu = (X[:, 1] / self.res + 1024).reshape(self.size[1], self.size[0]).astype(np.float32)  # east -> column
        mv = (X[:, 0] / self.res + 1024).reshape(self.size[1], self.size[0]).astype(np.float32)  # north -> row
        img = cv2.remap(self.tex, mu, mv, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        if self.noise > 0:
            img = img + self.rng.normal(0, self.noise, img.shape).astype(np.float32)
        return np.clip(np.round(img), 0, 255).astype(np.float32)  # 8-bit quantisation, as in a real camera
