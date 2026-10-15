import numpy as np
import pytest

from byeol import camera as cam
from byeol.cameramodel import CameraConfigError, CameraModel, Distortion


def test_from_pixels_basic():
    m = CameraModel.from_pixels(640, 360, fx=457.0, fy=457.0, cx=319.5, cy=179.5)
    assert m.f == 457.0 and m.kind == "rgb"
    assert np.allclose(m.center, [[319.5, 179.5]])


def test_from_mm_matches_pixels():
    # f=4.5 mm, sensor 6.4x3.6 mm, frame 1280x720: fx = 4.5*1280/6.4 = 900, fy = 4.5*720/3.6 = 900
    m = CameraModel.from_mm(1280, 720, focal_mm=4.5, sensor_w_mm=6.4, sensor_h_mm=3.6)
    assert m.fx == pytest.approx(900.0) and m.fy == pytest.approx(900.0)
    assert m.cx == pytest.approx(639.5) and m.cy == pytest.approx(359.5)


def test_from_mm_nonsquare_pixels():
    m = CameraModel.from_mm(640, 480, focal_mm=8.0, sensor_w_mm=6.4, sensor_h_mm=6.0)
    assert m.fx == pytest.approx(800.0) and m.fy == pytest.approx(640.0)


def test_from_fov_roundtrip():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    assert m.hfov_deg == pytest.approx(70.0) and m.fx == m.fy
    assert m.vfov_deg == pytest.approx(np.degrees(2 * np.arctan(360 / m.fy)))
    m2 = CameraModel.from_fov(640, 512, hfov_deg=50.0, vfov_deg=40.0)
    assert m2.hfov_deg == pytest.approx(50.0) and m2.vfov_deg == pytest.approx(40.0)


def test_scaling_resolution_scales_f_proportionally_and_keeps_fov():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    s = m.resized(640, 360)
    assert s.fx == pytest.approx(m.fx / 2) and s.fy == pytest.approx(m.fy / 2)
    assert s.hfov_deg == pytest.approx(m.hfov_deg) and s.vfov_deg == pytest.approx(m.vfov_deg)
    assert s.cx == pytest.approx(319.5) and s.cy == pytest.approx(179.5)  # centre is preserved


def test_scaling_keeps_same_ray_for_same_scene_point():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0, dist=Distortion(k1=-0.1))
    s = m.resized(320, 180)
    # pixel (x, y) of the full frame maps to ((x+0.5)/4-0.5, ...) of the small one
    px = np.array([[100.0, 50.0]])
    px_small = (px + 0.5) / 4 - 0.5
    assert np.allclose(m.ray(px), s.ray(px_small), atol=1e-9)


def test_binning_equals_resize():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    b = m.binned(2)
    assert (b.width, b.height) == (640, 360) and b.fx == pytest.approx(m.fx / 2)


def test_crop_shifts_principal_point_only():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    c = m.cropped(100, 50, 640, 360)
    assert c.fx == m.fx and c.cx == pytest.approx(m.cx - 100) and c.cy == pytest.approx(m.cy - 50)
    assert (c.width, c.height) == (640, 360)
    with pytest.raises(CameraConfigError):
        m.cropped(900, 0, 640, 360)


def test_crop_then_resize_scale_factor():
    m = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    c = m.cropped(0, 0, 640, 360).resized(320, 180)
    assert c.fx == pytest.approx(m.fx / 2)


def test_distortion_roundtrip():
    d = Distortion(k1=-0.25, k2=0.08, p1=0.001, p2=-0.0008, k3=-0.01)
    m = CameraModel.from_fov(1280, 720, hfov_deg=90.0, dist=d)
    px = np.array([[10.0, 20.0], [640.0, 360.0], [1200.0, 700.0], [300.0, 650.0]])
    back = m.project(m.ray(px))
    assert np.allclose(back, px, atol=1e-4)


def test_no_distortion_is_identity_with_pinhole_class():
    m = CameraModel.from_pixels(640, 360, 457.0, 457.0, 320.0, 180.0)
    p = cam.Pinhole(457.0, 320.0, 180.0)
    px = np.array([[10.0, 300.0]])
    assert np.allclose(m.ray(px), p.ray(px))
    P = np.array([[0.1, -0.2, 1.5]])
    assert np.allclose(m.project(P), p.project(P))


def test_ego_shift_works_with_camera_model_and_scales_with_resolution():
    big = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    small = big.resized(640, 360)
    R01 = np.eye(3)
    t = np.array([0.5, 0, 0])
    n = np.array([0, 0, 1.0])
    sb = cam.ego_shift(big, R01, t, n, 30.0)[0]
    ss = cam.ego_shift(small, R01, t, n, 30.0)[0]
    assert np.allclose(sb, 2 * ss, atol=1e-9)


def test_distortion_changes_offcenter_shift_but_not_center_much():
    d = Distortion(k1=-0.2)
    plain = CameraModel.from_fov(1280, 720, hfov_deg=90.0)
    dist = CameraModel.from_fov(1280, 720, hfov_deg=90.0, dist=d)
    R01 = np.eye(3)
    t = np.array([0.5, 0, 0])
    n = np.array([0, 0, 1.0])
    edge = np.array([[1200.0, 360.0]])
    a = cam.ego_shift(plain, R01, t, n, 30.0, edge)[0]
    b = cam.ego_shift(dist, R01, t, n, 30.0, edge)[0]
    assert abs(a[0] - b[0]) > 0.05
    ca, cb = cam.ego_shift(plain, R01, t, n, 30.0)[0], cam.ego_shift(dist, R01, t, n, 30.0)[0]
    assert abs(ca[0] - cb[0]) < 0.01


def test_kind_lwir_and_validation_errors():
    m = CameraModel.from_fov(320, 256, hfov_deg=45.0, kind="lwir")
    assert m.kind == "lwir"
    with pytest.raises(CameraConfigError, match="kind"):
        CameraModel.from_fov(320, 256, hfov_deg=45.0, kind="xray")
    with pytest.raises(CameraConfigError, match="hfov"):
        CameraModel.from_fov(320, 256, hfov_deg=190.0)
    with pytest.raises(CameraConfigError, match="width"):
        CameraModel.from_pixels(0, 256, 1, 1, 0, 0)
    with pytest.raises(CameraConfigError, match="fx"):
        CameraModel.from_pixels(320, 256, -5.0, 5.0, 160, 128)
    with pytest.raises(CameraConfigError, match="focal_mm"):
        CameraModel.from_mm(320, 256, focal_mm=0, sensor_w_mm=6.4, sensor_h_mm=5.1)


def test_dict_roundtrip():
    m = CameraModel.from_fov(640, 512, hfov_deg=32.0, kind="lwir", dist=Distortion(k1=-0.1), name="x")
    m2 = CameraModel.from_dict(m.to_dict())
    assert m2 == m
