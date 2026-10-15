import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from byeol import camera as cam
from byeol import synth
from byeol.vision_baseline import WaterRenderer, phase_shift

K = cam.Pinhole(f=457.0, cx=320.0, cy=180.0)


def test_phase_shift_sign_and_value_on_textured_ground():
    seq = synth.generate(synth.SynthConfig(mode="linear", duration=6.0, seed=1, gimbal_deg=90.0, wobble_accel=0.0))
    r = WaterRenderer(K, (640, 360), contrast=25.0, sensor_noise=0.0, seed=0)
    a, b = r.render(seq, 40), r.render(seq, 41)
    dx, dy, resp = phase_shift(a, b)
    tdx, tdy = seq.truth_shift(K, 40, 41)
    assert np.hypot(dx - tdx, dy - tdy) < 0.5 and resp > 0.3


def test_low_contrast_noisy_water_has_low_response():
    seq = synth.generate(synth.SynthConfig(mode="linear", duration=6.0, seed=1, gimbal_deg=90.0))
    hi = WaterRenderer(K, (640, 360), contrast=25.0, sensor_noise=1.0, seed=0)
    lo = WaterRenderer(K, (640, 360), contrast=1.0, sensor_noise=2.0, seed=0)
    r_hi = phase_shift(hi.render(seq, 40), hi.render(seq, 41))[2]
    r_lo = phase_shift(lo.render(seq, 40), lo.render(seq, 41))[2]
    assert r_lo < r_hi
