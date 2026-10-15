import numpy as np

from byeol import synth
from byeol.orientation import Mahony
from byeol.velocity import VelocityKF


def run(seq, aid=True, prior_sigma=None, with_cov=False):
    m = Mahony()
    m.init_from_accel(seq.accel[0], yaw=seq.yaw[0])
    kf = VelocityKF()
    ai = yi = 0
    err, trP = [], []
    for i in range(1, len(seq.imu_t)):
        dt = seq.imu_t[i] - seq.imu_t[i - 1]
        yaw = None
        while yi < len(seq.yaw_t) and seq.yaw_t[yi] <= seq.imu_t[i]:
            yaw = seq.yaw[yi]
            yi += 1
        m.update(dt, seq.gyro[i], seq.accel[i], yaw=yaw)
        kf.predict(dt, m.R, seq.accel[i])
        while aid and ai < len(seq.aux_vel_t) and seq.aux_vel_t[ai] <= seq.imu_t[i]:
            kf.update_velocity(seq.aux_vel[ai], 0.2)
            ai += 1
        if prior_sigma is not None and i % 20 == 0:
            kf.update_velocity(np.zeros(3), prior_sigma)
        err.append(np.linalg.norm(kf.v - seq.vel(seq.imu_t[i])))
        trP.append(np.trace(kf.P[:3, :3]))
    return np.array(err), np.array(trP), seq.imu_t[1:]


def test_aided_velocity_beats_raw_measurement_noise():
    seq = synth.generate(synth.SynthConfig(mode="linear", duration=40.0, seed=1, yaw_noise=0))
    # FC velocity is given in NED: the filter heading is taken from the FC (yaw), otherwise the coordinate frames diverge
    err, _, t = run(seq)
    assert np.median(err[t > 10]) < 0.4


def test_uncertainty_grows_without_aid_and_shrinks_with_it():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=20.0, seed=2))
    _, P0, _ = run(seq, aid=False)
    _, P1, _ = run(seq, aid=True)
    assert P0[-1] > 1.5 * P0[0]
    assert P1[-1] < P0[-1] / 5


def test_zero_prior_bounds_hover_error():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=40.0, seed=3))
    err, _, t = run(seq, aid=False, prior_sigma=0.5)
    assert np.median(err[t > 5]) < 0.4


def test_update_reduces_covariance_and_pulls_state():
    kf = VelocityKF()
    kf.P[:3, :3] = np.eye(3) * 4
    tr0 = np.trace(kf.P[:3, :3])
    kf.update_velocity(np.array([1.0, 0, 0]), 0.2)
    assert np.trace(kf.P[:3, :3]) < tr0 and kf.v[0] > 0.9
