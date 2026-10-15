"""Camera calibration from a chessboard (OpenCV is an optional dependency, imported lazily).

`calibrate_images` finds chessboard corners and calls cv2.calibrateCamera (Brown-Conrady model k1,k2,p1,p2,k3);
`calibrate_corners` does the same from already detected corners; `render_chessboard`/`project_board` build a synthetic chessboard for tests (pure numpy).
Idea: Zhang, "A flexible new technique for camera calibration", 2000 (inspiration only; the implementation calls OpenCV, Apache-2.0).
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np

from .cameramodel import CameraModel, Distortion


class CalibrationError(RuntimeError):
    pass


def _cv2():
    try:
        import cv2
    except ImportError as e:
        raise CalibrationError("calibration requires OpenCV: pip install opencv-python-headless") from e
    return cv2


def board_points(board: tuple[int, int], square: float) -> np.ndarray:
    """3D (inner) board corners in the board frame (z=0), metres; board = (corner columns, corner rows)."""
    cols, rows = board
    g = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    return np.concatenate([g * square, np.zeros((len(g), 1), np.float32)], axis=1)


def project_board(model: CameraModel, R: np.ndarray, t: np.ndarray, board, square):
    obj = board_points(board, square)
    P = obj.astype(float) @ R.T + t
    return obj, model.project(P)


@lru_cache(maxsize=4)
def _pixel_rays(model: CameraModel) -> np.ndarray:
    u_v = np.meshgrid(np.arange(model.width, dtype=float), np.arange(model.height, dtype=float))
    return model.ray(np.stack([u_v[0].ravel(), u_v[1].ravel()], axis=1))


def render_chessboard(model: CameraModel, R: np.ndarray, t: np.ndarray, board, square) -> np.ndarray:
    """Intensity 0..255 (uint8): a chessboard of (cols+1)x(rows+1) squares with a 1-square white border; distortion from model."""
    cols, rows = board
    rays = _pixel_rays(model)
    n = R[:, 2]
    s = (n @ t) / (rays @ n)
    X = (rays * s[:, None] - t) @ R  # coordinates in the board frame
    ci = np.floor(X[:, 0] / square + 1e-9).astype(int)  # square (0..cols) relative to the first inner corner
    cj = np.floor(X[:, 1] / square + 1e-9).astype(int)
    # inner corners lie on the grid nodes: squares from -1 to cols, rows are black/white, beyond that is the white border
    inside = (ci >= -1) & (ci <= cols - 1) & (cj >= -1) & (cj <= rows - 1)
    val = np.where(inside & (((ci + cj) % 2) == 0), 40.0, 215.0)
    val[(s <= 0)] = 215.0
    return val.reshape(model.height, model.width).astype(np.uint8)


def calibrate_corners(obj_pts: np.ndarray, views: list[np.ndarray], size: tuple[int, int], name: str = "calibrated") -> tuple[CameraModel, float]:
    cv2 = _cv2()
    if len(views) < 3:
        raise CalibrationError(f"at least 3 frames with a detected chessboard are required, got {len(views)} (10-20 recommended)")
    w, h = size
    rms, K, d, *_ = cv2.calibrateCamera([obj_pts.astype(np.float32)] * len(views),
                                        [np.asarray(v, np.float32).reshape(-1, 1, 2) for v in views], (w, h), None, None)
    d = d.ravel()
    k1, k2, p1, p2 = (float(x) for x in d[:4])
    k3 = float(d[4]) if len(d) > 4 else 0.0
    m = CameraModel(w, h, float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]), Distortion(k1, k2, p1, p2, k3), name=name)
    return m, float(rms)


def calibrate_images(images: list[np.ndarray], board, square: float, name: str = "calibrated"):
    """images: list of intensity images (uint8, H x W). Returns (CameraModel, reprojection RMS in px, number of frames used)."""
    cv2 = _cv2()
    if not images:
        raise CalibrationError("empty image list")
    h, w = images[0].shape[:2]
    views = []
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    for im in images:
        g = im if im.ndim == 2 else cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        if g.shape != (h, w):
            raise CalibrationError("all images must have the same size")
        ok, c = cv2.findChessboardCorners(g, tuple(board), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            c = cv2.cornerSubPix(g, c, (7, 7), (-1, -1), crit)
            views.append(c.reshape(-1, 2))
    if len(views) < 3:
        raise CalibrationError(f"chessboard with {board[0]}x{board[1]} inner corners found in only {len(views)} of {len(images)} frames")
    m, rms = calibrate_corners(board_points(board, square), views, (w, h), name)
    return m, rms, len(views)
