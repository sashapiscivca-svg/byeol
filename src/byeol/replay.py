"""Replays a synthetic sequence through EgoEstimator in time order; returns errors against ground truth."""
from __future__ import annotations

import numpy as np

from .estimator import EgoConfig, EgoEstimator
from .synth import Sequence


def replay(seq: Sequence, cfg: EgoConfig, stride: int = 1, **kw) -> dict[str, np.ndarray]:
    """For each frame pair (i-stride, i): estimate, ground truth, uncertainty. Auxiliary streams are fed in time order."""
    return replay_multi(seq, cfg, (stride,), **kw)[stride]


def replay_multi(seq: Sequence, cfg: EgoConfig, strides: tuple[int, ...] = (1,), aid_velocity: bool = True,
                 aid_heading: bool = True, vel_sigma: float | None = None,
                 estimator: EgoEstimator | None = None, truth_camera=None, pixel=None) -> dict[int, dict[str, np.ndarray]]:
    """A single IMU pass serving several inter-frame steps (strides) at once."""
    est = estimator or EgoEstimator(cfg)
    K = truth_camera if truth_camera is not None else cfg.camera
    vs = vel_sigma if vel_sigma is not None else seq.cfg.vel_noise
    events = [(t, 0, i) for i, t in enumerate(seq.imu_t)]
    events += [(t, 1, i) for i, t in enumerate(seq.height_t)]
    if aid_velocity:
        events += [(t, 2, i) for i, t in enumerate(seq.aux_vel_t)]
    if aid_heading:
        events += [(t, 3, i) for i, t in enumerate(seq.yaw_t)]
    events += [(float(t), 5, i) for i, t in enumerate(seq.frame_t)]
    events.sort(key=lambda e: (e[0], e[1]))
    rows: dict[int, list] = {st: [] for st in strides}
    for t, kind, i in events:
        if kind == 0:
            est.push_imu(t, seq.gyro[i], seq.accel[i])
        elif kind == 1:
            est.push_height(t, seq.height[i])
        elif kind == 2:
            est.push_velocity(t, seq.aux_vel[i], vs)
        elif kind == 3:
            est.push_heading(t, seq.yaw[i])
        else:
            est.push_gimbal(t, seq.gimbal(t))
            for st in strides:
                if i >= st:
                    j = i - st
                    r = est.frame_shift(float(seq.frame_t[j]), t)
                    tdx, tdy = seq.truth_shift(K, j, i, pixel)
                    rows[st].append((t, r.dx, r.dy, tdx, tdy, r.valid, r.sigma, r.conf, r.cov, i))
    return {st: _pack(rr) for st, rr in rows.items()}


def _pack(rows: list) -> dict[str, np.ndarray]:
    return {
        "t": np.array([r[0] for r in rows]),
        "dx": np.array([r[1] for r in rows]),
        "dy": np.array([r[2] for r in rows]),
        "tdx": np.array([r[3] for r in rows]),
        "tdy": np.array([r[4] for r in rows]),
        "valid": np.array([r[5] for r in rows], dtype=bool),
        "sigma": np.array([r[6] for r in rows]),
        "conf": np.array([r[7] for r in rows]),
        "cov": np.array([r[8] for r in rows]),
        "frame": np.array([r[9] for r in rows]),
    }
