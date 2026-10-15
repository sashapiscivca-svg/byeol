import numpy as np
import pytest

from byeol import rotation as rot
from byeol import synth
from byeol.orientation import Mahony

G = 9.80665


def run(seq, f: Mahony, use_yaw=False):
    f.init_from_accel(seq.accel[0])
    errs, tt = [], []
    yi = 0
    for i in range(1, len(seq.imu_t)):
        dt = seq.imu_t[i] - seq.imu_t[i - 1]
        yaw = None
        while use_yaw and yi < len(seq.yaw_t) and seq.yaw_t[yi] <= seq.imu_t[i]:
            yaw = seq.yaw[yi]
            yi += 1
        f.update(dt, seq.gyro[i], seq.accel[i], yaw=yaw)
        Rt = seq.R_wb(seq.imu_t[i])
        Re = f.R
        # tilt error (ignoring heading): angle between "down" directions in body frame
        dn = np.degrees(np.arccos(np.clip((Rt.T[2] @ Re.T[2]), -1, 1)))
        errs.append(dn)
        tt.append(seq.imu_t[i])
    return np.array(tt), np.array(errs)


def test_static_level_stays_level():
    f = Mahony()
    f.init_from_accel(np.array([0, 0, -G]))
    for _ in range(200):
        f.update(0.005, np.zeros(3), np.array([0, 0, -G]))
    r, p, _ = rot.R_to_euler(f.R)
    assert abs(r) < 1e-6 and abs(p) < 1e-6


def test_init_from_accel_gives_tilt():
    R = rot.euler_to_R(0.2, -0.1, 0.0)
    f_spec = R.T @ np.array([0, 0, -G])
    m = Mahony()
    m.init_from_accel(f_spec)
    r, p, _ = rot.R_to_euler(m.R)
    assert r == pytest.approx(0.2, abs=1e-6) and p == pytest.approx(-0.1, abs=1e-6)


def test_hover_tilt_error_small_with_biased_noisy_gyro():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=40.0, seed=3))
    t, e = run(seq, Mahony())
    assert np.median(e[t > 10]) < 1.0 and np.percentile(e[t > 10], 95) < 2.5


def test_gyro_bias_estimated():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=60.0, seed=4, gyro_bias_walk=0.0))
    m = Mahony()
    run(seq, m)
    b_true = seq.gyro_bias_true.mean(axis=0)
    assert np.abs(m.bias[:2] - b_true[:2]).max() < 0.012


def test_yaw_aid_pulls_heading():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=40.0, seed=5))
    m = Mahony()
    run(seq, m, use_yaw=True)
    ytrue = rot.R_to_euler(seq.R_wb(seq.imu_t[-1]))[2]
    assert abs(rot.wrap_pi(rot.R_to_euler(m.R)[2] - ytrue)) < 0.1


def test_accel_gate_ignores_large_dynamic_accel():
    m = Mahony()
    m.init_from_accel(np.array([0, 0, -G]))
    # an abrupt implausible accelerometer reading (3 g sideways) must not skew the tilt
    for _ in range(100):
        m.update(0.005, np.zeros(3), np.array([3 * G, 0, -G]))
    r, p, _ = rot.R_to_euler(m.R)
    assert abs(p) < 0.01 and abs(r) < 0.01


def test_vibration_does_not_break_tilt():
    seq = synth.generate(synth.SynthConfig(mode="vibration", duration=40.0, seed=6))
    t, e = run(seq, Mahony())
    assert np.median(e[t > 10]) < 1.5
