import json
from pathlib import Path

import pytest

from byeol.cameramodel import CameraConfigError
from byeol.camprofile import dump_camera, load_camera, load_camera_dict

CONF = Path(__file__).resolve().parents[1] / "configs" / "cameras"


def test_load_fov_mode():
    m = load_camera_dict({"schema": 1, "name": "t", "kind": "rgb", "resolution": {"width": 1280, "height": 720},
                          "intrinsics": {"mode": "fov", "hfov_deg": 70}})
    assert m.hfov_deg == pytest.approx(70.0) and m.name == "t"


def test_load_mm_mode_and_distortion():
    m = load_camera_dict({"schema": 1, "resolution": {"width": 1280, "height": 720},
                          "intrinsics": {"mode": "mm", "focal_mm": 4.5, "sensor_w_mm": 6.4, "sensor_h_mm": 3.6},
                          "distortion": {"k1": -0.1, "p1": 0.001}})
    assert m.fx == pytest.approx(900.0) and m.dist.k1 == -0.1 and m.dist.p1 == 0.001


def test_load_pixels_mode_default_center():
    m = load_camera_dict({"schema": 1, "resolution": {"width": 640, "height": 360},
                          "intrinsics": {"mode": "pixels", "fx": 457, "fy": 457}})
    assert m.cx == 319.5 and m.cy == 179.5


@pytest.mark.parametrize("bad,match", [
    ({"schema": 2}, "schema"),
    ({"schema": 1}, "resolution"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}}, "intrinsics"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "magic"}}, "mode"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "fov"}}, "hfov_deg"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "fov", "hfov_deg": 70, "fx": 1}}, "fx"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "fov", "hfov_deg": 70}, "kindd": "rgb"}, "kindd"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "fov", "hfov_deg": 70}, "distortion": {"k9": 1}}, "k9"),
    ({"schema": 1, "resolution": {"width": 640, "height": "x"}, "intrinsics": {"mode": "fov", "hfov_deg": 70}}, "height"),
    ({"schema": 1, "resolution": {"width": 640, "height": 360}, "intrinsics": {"mode": "fov", "hfov_deg": 270}}, "hfov"),
])
def test_validation_messages(bad, match):
    with pytest.raises(CameraConfigError, match=match):
        load_camera_dict(bad)


def test_json_roundtrip(tmp_path):
    m = load_camera_dict({"schema": 1, "name": "j", "kind": "lwir", "resolution": {"width": 320, "height": 256},
                          "intrinsics": {"mode": "fov", "hfov_deg": 45}, "distortion": {"k1": -0.05}})
    p = tmp_path / "c.json"
    dump_camera(m, p)
    assert json.loads(p.read_text())["schema"] == 1
    m2 = load_camera(p)
    assert m2.fx == pytest.approx(m.fx) and m2.kind == "lwir" and m2.dist.k1 == -0.05


def test_missing_file_message(tmp_path):
    with pytest.raises(CameraConfigError, match="not found"):
        load_camera(tmp_path / "nope.yaml")


def test_yaml_roundtrip(tmp_path):
    pytest.importorskip("yaml")
    m = load_camera_dict({"schema": 1, "resolution": {"width": 640, "height": 512},
                          "intrinsics": {"mode": "fov", "hfov_deg": 32}, "kind": "lwir"})
    p = tmp_path / "c.yaml"
    dump_camera(m, p)
    assert load_camera(p).fx == pytest.approx(m.fx)


def test_all_shipped_profiles_are_valid():
    pytest.importorskip("yaml")
    files = sorted(CONF.glob("*.yaml"))
    assert len(files) >= 6
    kinds = set()
    for f in files:
        m = load_camera(f)
        kinds.add(m.kind)
        assert m.name == f.stem
    assert kinds == {"rgb", "lwir"}
