import numpy as np
import pytest

from byeol import camera as cam
from byeol import rotation as rot
from byeol import synth

K = cam.Pinhole(f=900.0, cx=640.0, cy=360.0)
G = 9.80665


def mk(mode="hover", **kw):
    return synth.generate(synth.SynthConfig(mode=mode, duration=6.0, seed=1, **kw))


def test_deterministic_by_seed():
    a, b = mk(), mk()
    assert np.array_equal(a.gyro, b.gyro) and np.array_equal(a.accel, b.accel)
    c = synth.generate(synth.SynthConfig(mode="hover", duration=6.0, seed=2))
    assert not np.array_equal(a.gyro, c.gyro)


def test_static_accel_magnitude_is_g_in_hover():
    s = mk(gyro_noise=0, accel_noise=0, accel_bias=0, gyro_bias=0)
    mag = np.linalg.norm(s.accel, axis=1)
    assert np.abs(mag.mean() - G) < 0.3


def test_noise_free_gyro_integrates_to_truth_attitude():
    s = mk("gust", gyro_noise=0, gyro_bias=0, gyro_bias_walk=0)
    t0, t1 = 1.0, 5.0
    q = np.array([1.0, 0, 0, 0])
    idx = np.where((s.imu_t >= t0) & (s.imu_t < t1))[0]
    for i in idx[:-1]:
        dt = s.imu_t[i + 1] - s.imu_t[i]
        q = rot.q_mul(q, rot.q_exp(0.5 * (s.gyro[i] + s.gyro[i + 1]) * dt))
    R_true = s.R_wb(s.imu_t[idx[0]]).T @ s.R_wb(s.imu_t[idx[-1]])
    err = np.linalg.norm(rot.q_log(rot.q_mul(rot.q_conj(rot.R_to_q(R_true)), q)))
    assert err < 1e-4


def test_truth_shift_matches_independent_world_projection():
    s = mk("linear")
    i0, i1 = 40, 41
    dx, dy = s.truth_shift(K, i0, i1)
    # independent path: ray -> ground point in world -> projection into frame 1
    t0, t1 = s.frame_t[i0], s.frame_t[i1]
    Rwc0, Rwc1 = s.R_wb(t0) @ cam.gimbal_R_bc(s.gimbal(t0)), s.R_wb(t1) @ cam.gimbal_R_bc(s.gimbal(t1))
    p0, p1 = s.pos(t0), s.pos(t1)
    ray_w = Rwc0 @ K.ray(K.center)[0]
    X = p0 + ray_w * (-p0[2] / ray_w[2])
    Pc1 = Rwc1.T @ (X - p1)
    uv = np.array([K.cx + K.f * Pc1[0] / Pc1[2], K.cy + K.f * Pc1[1] / Pc1[2]])
    assert np.allclose([dx, dy], uv - K.center[0], atol=1e-9)


def test_linear_mode_speed_and_shift_scale():
    s = mk("linear", speed=5.0, gimbal_deg=90.0, wobble_accel=0.0)
    v = np.linalg.norm(s.vel(3.0))
    assert v == pytest.approx(5.0, abs=0.3)
    dx, dy = s.truth_shift(K, 40, 41)
    # ~ f * v * dt / h = 900*5/30/30 = 5 px per frame (magnitude)
    assert 3.5 < np.hypot(dx, dy) < 6.5


def test_dropouts_remove_samples():
    full = mk()
    gap = mk(dropout_prob=0.05)
    assert len(gap.imu_t) < len(full.imu_t)
    assert np.all(np.diff(gap.imu_t) > 0)


def test_vibration_raises_gyro_rms():
    base = mk("hover", gyro_noise=0, gyro_bias=0)
    vib = mk("vibration", gyro_noise=0, gyro_bias=0)
    assert np.diff(vib.gyro, axis=0).std() > 3 * np.diff(base.gyro, axis=0).std()


def test_aux_streams_present():
    s = mk()
    assert len(s.height_t) == len(s.height) > 10 and s.aux_vel.shape[1] == 3
    assert len(s.aux_vel_t) == len(s.aux_vel) and len(s.yaw_t) == len(s.yaw)
