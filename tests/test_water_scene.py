import numpy as np
import pytest

pytest.importorskip("cv2")
from byeol import synth
from byeol.cameramodel import CameraModel
from byeol.water_scene import WaterScene, place_target, target_truth

K = CameraModel.from_pixels(640, 360, fx=457.0, fy=457.0, cx=320.0, cy=180.0)


@pytest.fixture(scope="module")
def seq():
    return synth.generate(synth.SynthConfig(mode="hover", duration=6.0, seed=1))


def test_target_truth_center_at_placed_pixel(seq):
    i = 60
    tgt = place_target(seq, K, float(seq.frame_t[i]), pixel=(300.0, 200.0), rng=np.random.default_rng(0))
    gt = target_truth(seq, K, tgt, i)
    assert np.allclose(gt["center"], (300.0, 200.0), atol=1e-3) and gt["present"]
    assert 8 <= gt["d"] <= 25           # person ~1.2 m from 30 m


def test_target_moves_slowly_in_world(seq):
    tgt = place_target(seq, K, 2.0, rng=np.random.default_rng(0), speed=0.4)
    assert np.linalg.norm(tgt.pos(3.0) - tgt.pos(2.0)) < 0.4 + 0.3 * 2 * np.pi / tgt.sway_period + 1e-6


def test_scene_is_deterministic_and_target_visible(seq):
    i = 60
    tgt = place_target(seq, K, float(seq.frame_t[i]), rng=np.random.default_rng(0), amp=20.0)
    a, ga = WaterScene(K, (640, 360), contrast=0.5, seed=2).render_scene(seq, i, tgt)
    b, gb = WaterScene(K, (640, 360), contrast=0.5, seed=2).render_scene(seq, i, tgt)
    assert np.array_equal(a, b) and np.allclose(ga["center"], gb["center"])
    cx, cy = (round(v) for v in ga["center"])
    assert a[cy - 1:cy + 2, cx - 1:cx + 2].mean() < a.mean() - 10        # dark blob at the ground-truth centre


def test_waves_and_glints_do_not_change_truth(seq):
    i = 40
    tgt = place_target(seq, K, float(seq.frame_t[i]), rng=np.random.default_rng(0))
    _, g1 = WaterScene(K, (640, 360), seed=2, wave_frac=0.0, glint_rate=0.0).render_scene(seq, i, tgt)
    img, g2 = WaterScene(K, (640, 360), seed=2, wave_frac=1.0, glint_rate=8.0).render_scene(seq, i, tgt)
    assert np.allclose(g1["center"], g2["center"]) and img.dtype == np.float32
