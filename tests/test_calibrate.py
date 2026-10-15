import numpy as np
import pytest

from byeol import rotation as rot
from byeol.calibrate import CalibrationError, calibrate_corners, project_board, render_chessboard
from byeol.cameramodel import CameraModel, Distortion

BOARD = (9, 6)
SQ = 0.03


def poses(n=14, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        R = rot.euler_to_R(*rng.uniform(-0.45, 0.45, 2), rng.uniform(-0.5, 0.5))
        # board centre (0.12, 0.075, 0) m -> placed in front of the camera at 0.45-0.7 m
        t = np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.04, 0.04), rng.uniform(0.45, 0.7)]) - R @ np.array([0.12, 0.075, 0])
        out.append((R, t))
    return out


TRUE = CameraModel.from_pixels(640, 480, fx=600.0, fy=605.0, cx=322.0, cy=238.0, dist=Distortion(k1=-0.12, k2=0.03, p1=0.001, p2=-0.0005))


def test_calibration_from_exact_corners_recovers_parameters():
    pytest.importorskip("cv2")
    pts_obj, views = None, []
    for R, t in poses():
        pts_obj, px = project_board(TRUE, R, t, BOARD, SQ)
        views.append(px)
    est, rms = calibrate_corners(pts_obj, views, (640, 480))
    assert rms < 0.05
    assert est.fx == pytest.approx(600.0, rel=0.005) and est.fy == pytest.approx(605.0, rel=0.005)
    assert abs(est.cx - 322.0) < 1.5 and abs(est.cy - 238.0) < 1.5
    assert est.dist.k1 == pytest.approx(-0.12, abs=0.02)


def test_calibration_from_rendered_images_end_to_end():
    cv2 = pytest.importorskip("cv2")
    from byeol.calibrate import calibrate_images

    imgs = []
    for R, t in poses(12, seed=1):
        im = render_chessboard(TRUE, R, t, BOARD, SQ)
        imgs.append(cv2.GaussianBlur(im, (0, 0), 0.8))
    est, rms, used = calibrate_images(imgs, BOARD, SQ)
    assert used >= 10 and rms < 0.7
    assert est.fx == pytest.approx(600.0, rel=0.02) and abs(est.cx - 322.0) < 4.0
    assert (est.width, est.height) == (640, 480)


def test_too_few_views_message():
    pytest.importorskip("cv2")
    R, t = poses(1)[0]
    pts, px = project_board(TRUE, R, t, BOARD, SQ)
    with pytest.raises(CalibrationError, match="frames"):
        calibrate_corners(pts, [px], (640, 480))


def test_cli_script_end_to_end(tmp_path):
    cv2 = pytest.importorskip("cv2")
    import subprocess
    import sys

    from byeol.camprofile import load_camera

    for i, (R, t) in enumerate(poses(10, seed=2)):
        cv2.imwrite(str(tmp_path / f"v{i:02d}.png"), cv2.GaussianBlur(render_chessboard(TRUE, R, t, BOARD, SQ), (0, 0), 0.8))
    out = tmp_path / "cam.json"
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    r = subprocess.run([sys.executable, str(root / "scripts" / "calibrate_camera.py"), "--images", str(tmp_path / "v*.png"),
                        "--cols", "9", "--rows", "6", "--square-mm", "30", "--name", "t", "--out", str(out)],
                       capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr
    m = load_camera(out)
    assert m.fx == pytest.approx(600.0, rel=0.03) and m.name == "t"


def test_cli_no_files_returns_error(tmp_path):
    import subprocess
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    r = subprocess.run([sys.executable, str(root / "scripts" / "calibrate_camera.py"), "--images", str(tmp_path / "x*.png"),
                        "--cols", "9", "--rows", "6", "--square-mm", "30", "--out", str(tmp_path / "o.json")],
                       capture_output=True, text=True, check=False)
    assert r.returncode != 0
