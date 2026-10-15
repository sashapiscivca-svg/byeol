"""Pinhole camera model and the pixel shift of frame content between two time instants.

Coordinate frames (see docs/TECHNICAL_REPORT.md):
- camera: x right, y down the frame, z along the optical axis; pixel (u, v) = (cx + f x/z, cy + f y/z);
- drone body FRD (forward, right, down); world NED (north, east, down);
- gimbal angle theta: 0 = camera looks forward horizontally, 90 degrees = nadir.
Ground/water plane: z_world = 0 (height h above it). Shift (dx, dy) = where a frame-0 pixel ends up in frame 1
minus its initial position; positive dx = content moved right (same convention as phaseCorrelate in dal/ego.py).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pinhole:
    f: float  # focal length in pixels (at the same frame scale in which the shift is expected)
    cx: float
    cy: float

    def ray(self, px: np.ndarray) -> np.ndarray:
        px = np.atleast_2d(px).astype(float)
        return np.stack([(px[:, 0] - self.cx) / self.f, (px[:, 1] - self.cy) / self.f, np.ones(len(px))], axis=1)

    def project(self, P: np.ndarray) -> np.ndarray:
        z = P[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            out = np.stack([self.cx + self.f * P[:, 0] / z, self.cy + self.f * P[:, 1] / z], axis=1)
        out[~(z > 1e-6)] = np.nan
        return out

    @property
    def center(self) -> np.ndarray:
        return np.array([[self.cx, self.cy]])


def gimbal_R_bc(theta: float) -> np.ndarray:
    """Camera->body matrix (columns = camera axes in FRD) for gimbal angle theta (rad, 0 horizon, pi/2 nadir)."""
    s, c = np.sin(theta), np.cos(theta)
    # columns: camera x = right (0,1,0); camera y = (-s,0,c); camera z (optical axis) = (c,0,s)
    return np.array([[0.0, -s, c], [1.0, 0.0, 0.0], [0.0, c, s]])


def ground_plane_in_camera(R_wb: np.ndarray, R_bc: np.ndarray, h: float) -> tuple[np.ndarray, float]:
    """Ground plane in the camera frame: n·P = d. n is the unit normal (pointing down, in camera coordinates), d = height h."""
    R_wc = R_wb @ R_bc
    n = R_wc.T @ np.array([0.0, 0.0, 1.0])
    return n, float(h)


def ego_shift(K: Pinhole, R01: np.ndarray, t01: np.ndarray, n: np.ndarray, d: float,
              pixels: np.ndarray | None = None) -> np.ndarray:
    """Shift (N, 2) of frame-0 pixels into frame 1 for points on the plane n·P = d (camera-0 coordinates).

    R01: rotation camera-1 -> camera-0; t01: position of camera 1 in the camera-0 frame (m).
    NaN if the ray does not intersect the plane in front of the camera (e.g. the horizon).
    """
    px = K.center if pixels is None else np.atleast_2d(pixels).astype(float)
    r = K.ray(px)
    denom = r @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(denom > 1e-3, d / denom, np.nan)
    P0 = r * s[:, None]
    P1 = (P0 - t01[None, :]) @ R01  # (R01^T (P0 - t))^T
    return K.project(P1) - px
