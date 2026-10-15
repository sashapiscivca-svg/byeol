"""Configurable camera model: pinhole + Brown-Conrady distortion (as in OpenCV), any resolution/crop/binning.

Pixel convention: a pixel centre has integer coordinates (as in OpenCV); for a centred camera cx = (W-1)/2.
Three ways to specify it: `from_pixels` (fx, fy, cx, cy), `from_mm` (focal length and sensor size in mm), `from_fov` (HFOV/VFOV).
`resized/binned/cropped` return a model for a different frame of the same optics (f in pixels scales proportionally).
Camera type (`kind`): "rgb" or "lwir" (thermal). Note on thermal sensors: they have low resolution (example sizes:
160x120, 320x256, 640x512), so 1 pixel = a large angle and the same error in metres yields fewer pixels; no vendor-specific
characteristics are built in here: real f, FOV and distortion come from calibration (scripts/calibrate_camera.py).
The interface is compatible with camera.Pinhole (ray, project, center), so camera.ego_shift accepts both.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace

import numpy as np

KINDS = ("rgb", "lwir")


class CameraConfigError(ValueError):
    """Invalid camera parameters (the message names the field and the reason)."""


@dataclass(frozen=True)
class Distortion:
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0

    @property
    def is_zero(self) -> bool:
        return not any((self.k1, self.k2, self.p1, self.p2, self.k3))

    def apply(self, xy: np.ndarray) -> np.ndarray:
        x, y = xy[:, 0], xy[:, 1]
        r2 = x * x + y * y
        rad = 1 + self.k1 * r2 + self.k2 * r2**2 + self.k3 * r2**3
        xd = x * rad + 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
        yd = y * rad + self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
        return np.stack([xd, yd], axis=1)

    def remove(self, xyd: np.ndarray, iters: int = 40) -> np.ndarray:
        """Distortion inversion by fixed-point iteration (as in OpenCV undistortPoints); for a monotonic model."""
        xy = xyd.copy()
        for _ in range(iters):
            x, y = xy[:, 0], xy[:, 1]
            r2 = x * x + y * y
            rad = 1 + self.k1 * r2 + self.k2 * r2**2 + self.k3 * r2**3
            dx = 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
            dy = self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
            xy = np.stack([(xyd[:, 0] - dx) / rad, (xyd[:, 1] - dy) / rad], axis=1)
        return xy


def _need(cond: bool, msg: str) -> None:
    if not cond:
        raise CameraConfigError(msg)


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: Distortion = field(default_factory=Distortion)
    kind: str = "rgb"
    name: str = ""
    notes: str = ""

    def __post_init__(self) -> None:
        _need(int(self.width) == self.width and self.width > 0, f"width must be a positive integer, got {self.width}")
        _need(int(self.height) == self.height and self.height > 0, f"height must be a positive integer, got {self.height}")
        _need(self.fx > 0 and np.isfinite(self.fx), f"fx must be a positive number, got {self.fx}")
        _need(self.fy > 0 and np.isfinite(self.fy), f"fy must be a positive number, got {self.fy}")
        _need(np.isfinite(self.cx) and np.isfinite(self.cy), "cx, cy must be finite")
        _need(self.kind in KINDS, f"kind must be one of {KINDS}, got {self.kind!r}")

    # ---- constructors ----
    @classmethod
    def from_pixels(cls, width, height, fx, fy, cx=None, cy=None, **kw) -> CameraModel:
        cx = (width - 1) / 2 if cx is None else cx
        cy = (height - 1) / 2 if cy is None else cy
        return cls(width, height, float(fx), float(fy), float(cx), float(cy), **kw)

    @classmethod
    def from_mm(cls, width, height, focal_mm, sensor_w_mm, sensor_h_mm, **kw) -> CameraModel:
        _need(focal_mm > 0, f"focal_mm must be positive, got {focal_mm}")
        _need(sensor_w_mm > 0 and sensor_h_mm > 0, "sensor_w_mm and sensor_h_mm must be positive")
        return cls.from_pixels(width, height, focal_mm * width / sensor_w_mm, focal_mm * height / sensor_h_mm, **kw)

    @classmethod
    def from_fov(cls, width, height, hfov_deg, vfov_deg=None, **kw) -> CameraModel:
        _need(0 < hfov_deg < 180, f"hfov must be in (0, 180) degrees, got {hfov_deg}")
        fx = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
        if vfov_deg is None:
            fy = fx  # square pixels
        else:
            _need(0 < vfov_deg < 180, f"vfov must be in (0, 180) degrees, got {vfov_deg}")
            fy = (height / 2) / np.tan(np.radians(vfov_deg) / 2)
        return cls.from_pixels(width, height, fx, fy, **kw)

    # ---- derived quantities ----
    @property
    def f(self) -> float:
        return 0.5 * (self.fx + self.fy)

    @property
    def center(self) -> np.ndarray:
        return np.array([[self.cx, self.cy]])

    @property
    def hfov_deg(self) -> float:
        return float(np.degrees(2 * np.arctan(self.width / 2 / self.fx)))

    @property
    def vfov_deg(self) -> float:
        return float(np.degrees(2 * np.arctan(self.height / 2 / self.fy)))

    # ---- projection ----
    def ray(self, px: np.ndarray) -> np.ndarray:
        px = np.atleast_2d(px).astype(float)
        xyd = np.stack([(px[:, 0] - self.cx) / self.fx, (px[:, 1] - self.cy) / self.fy], axis=1)
        xy = xyd if self.dist.is_zero else self.dist.remove(xyd)
        return np.concatenate([xy, np.ones((len(px), 1))], axis=1)

    def project(self, P: np.ndarray) -> np.ndarray:
        z = P[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            xy = np.stack([P[:, 0] / z, P[:, 1] / z], axis=1)
        if not self.dist.is_zero:
            xy = self.dist.apply(xy)
        out = np.stack([self.cx + self.fx * xy[:, 0], self.cy + self.fy * xy[:, 1]], axis=1)
        out[~(z > 1e-6)] = np.nan
        return out

    # ---- frame changes ----
    def resized(self, width: int, height: int) -> CameraModel:
        _need(width > 0 and height > 0, "new resolution must be positive")
        sx, sy = width / self.width, height / self.height
        return replace(self, width=int(width), height=int(height), fx=self.fx * sx, fy=self.fy * sy,
                       cx=(self.cx + 0.5) * sx - 0.5, cy=(self.cy + 0.5) * sy - 0.5)

    def binned(self, bx: int, by: int | None = None) -> CameraModel:
        by = bx if by is None else by
        _need(bx >= 1 and by >= 1, "binning factors must be >= 1")
        return self.resized(self.width // bx, self.height // by)

    def cropped(self, x0: int, y0: int, width: int, height: int) -> CameraModel:
        _need(x0 >= 0 and y0 >= 0 and x0 + width <= self.width and y0 + height <= self.height,
              f"crop ({x0},{y0},{width},{height}) exceeds the frame {self.width}x{self.height}")
        return replace(self, width=int(width), height=int(height), cx=self.cx - x0, cy=self.cy - y0)

    # ---- serialisation ----
    def to_dict(self) -> dict:
        d = asdict(self)
        d["dist"] = asdict(self.dist)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> CameraModel:
        d = dict(d)
        d["dist"] = Distortion(**d.get("dist", {}))
        return cls(**d)
