import numpy as np
import pytest

from byeol import camera as cam
from byeol import rotation as rot

F = 900.0
K = cam.Pinhole(f=F, cx=640.0, cy=360.0)
NADIR = np.pi / 2


def test_gimbal_nadir_axes():
    Rbc = cam.gimbal_R_bc(NADIR)
    assert np.allclose(Rbc[:, 2], [0, 0, 1], atol=1e-12)  # optical axis down (FRD z)
    assert np.allclose(Rbc[:, 1], [-1, 0, 0], atol=1e-12)  # "down" in frame = backward along body


def test_gimbal_forward_axes():
    Rbc = cam.gimbal_R_bc(0.0)
    assert np.allclose(Rbc[:, 2], [1, 0, 0], atol=1e-12)
    assert np.allclose(Rbc[:, 1], [0, 0, 1], atol=1e-12)


def test_pure_pan_shift_is_minus_f_tan():
    # camera turns right (about camera y): content shifts left
    a = 0.01
    R01 = rot.rotvec_to_R(np.array([0, a, 0.0]))
    dx, dy = cam.ego_shift(K, R01, np.zeros(3), np.array([0, 0, 1.0]), 30.0)[0]
    assert dx == pytest.approx(-F * np.tan(a), rel=1e-9) and dy == pytest.approx(0, abs=1e-9)


def test_pure_tilt_shift_is_plus_f_tan():
    a = 0.01
    R01 = rot.rotvec_to_R(np.array([a, 0, 0.0]))
    dx, dy = cam.ego_shift(K, R01, np.zeros(3), np.array([0, 0, 1.0]), 30.0)[0]
    assert dy == pytest.approx(F * np.tan(a), rel=1e-9) and dx == pytest.approx(0, abs=1e-9)


def test_pure_translation_nadir():
    # camera moves right by 0.5 m from 30 m height: content -> left by f*0.5/30
    dx, dy = cam.ego_shift(K, np.eye(3), np.array([0.5, 0, 0]), np.array([0, 0, 1.0]), 30.0)[0]
    assert dx == pytest.approx(-F * 0.5 / 30, rel=1e-9) and dy == pytest.approx(0, abs=1e-9)


def test_zero_motion_zero_shift():
    s = cam.ego_shift(K, np.eye(3), np.zeros(3), np.array([0, 0, 1.0]), 10.0)
    assert np.allclose(s, 0)


def test_forward_motion_expands_off_center():
    # forward flight over ground (along the z axis) gives divergence from the centre; no shift at the centre
    px = np.array([[640.0, 360.0], [1000.0, 360.0]])
    s = cam.ego_shift(K, np.eye(3), np.array([0, 0, 1.0]), np.array([0, 0, 1.0]), 30.0, px)
    assert np.allclose(s[0], 0, atol=1e-9) and s[1][0] > 0


def test_horizon_plane_is_invalid():
    # plane parallel to the ray (camera looks along the ground) -> NaN
    s = cam.ego_shift(K, np.eye(3), np.array([0.1, 0, 0]), np.array([0, 1.0, 0]), 30.0)
    assert np.isnan(s).all()


def test_plane_normal_from_attitude():
    # level flight, nadir camera: normal in camera = (0,0,1), distance = height
    n, d = cam.ground_plane_in_camera(np.eye(3), cam.gimbal_R_bc(NADIR), 25.0)
    assert np.allclose(n, [0, 0, 1], atol=1e-12) and d == pytest.approx(25.0)


def test_plane_distance_slant_for_oblique():
    # camera 45 deg below the horizon: distance along the axis = h / sin(45)
    n, d = cam.ground_plane_in_camera(np.eye(3), cam.gimbal_R_bc(np.pi / 4), 20.0)
    assert d == pytest.approx(20.0)  # d is the distance to the plane ALONG the normal (n·P = d)
    assert n[2] == pytest.approx(np.sin(np.pi / 4))
