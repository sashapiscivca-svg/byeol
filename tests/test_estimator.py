import numpy as np
import pytest

from byeol import camera as cam
from byeol import rotation as rot
from byeol import synth
from byeol.estimator import EgoConfig, EgoEstimator, HeightSource
from byeol.replay import replay

G = 9.80665
K = cam.Pinhole(f=900.0, cx=640.0, cy=360.0)


def make(**kw):
    return EgoEstimator(EgoConfig(camera=K, **kw))


def feed_static(est, secs, gyro, gimbal=np.pi / 2, h=30.0, rate=200.0, accel=(0, 0, -G), v=None, rotate_accel=False):
    v = [0.0, 0.0, 0.0] if v is None else v
    n = int(secs * rate)
    est.push_gimbal(0.0, gimbal)
    est.push_height(0.0, h)
    if v is not None:
        est.push_velocity(0.0, np.asarray(v, float), 0.05)
    Rb = np.eye(3)  # R_wb: when rotating, the accelerometer must be consistent with the rotation (otherwise the filter treats it as bias)
    for i in range(n):
        t = i / rate
        f = Rb.T @ np.asarray(accel, float) if rotate_accel else np.asarray(accel, float)
        est.push_imu(t, np.asarray(gyro, float), f)
        Rb = Rb @ rot.rotvec_to_R(np.asarray(gyro, float) / rate) if rotate_accel else Rb
        if i % 20 == 0:
            est.push_height(t, h)
            if v is not None:
                est.push_velocity(t, np.asarray(v, float), 0.05)


def test_pure_roll_rate_matches_analytic():
    # body roll (body x axis = -y of the nadir camera): content moves along +x of the frame
    est = make()
    feed_static(est, 2.0, [0.3, 0, 0.0], rotate_accel=True)
    r = est.frame_shift(1.0, 1.0 + 1 / 30)
    expect = K.f * np.tan(0.3 / 30)
    assert r.valid and r.dx == pytest.approx(expect, rel=1e-2) and abs(r.dy) < 1e-2


def test_pure_pitch_rate_matches_analytic():
    est = make()
    feed_static(est, 2.0, [0.0, 0.25, 0.0], rotate_accel=True)
    r = est.frame_shift(1.0, 1.0 + 1 / 30)
    assert r.dy == pytest.approx(K.f * np.tan(0.25 / 30), rel=1e-2) and abs(r.dx) < 1e-2


def test_gyro_bias_is_subtracted_after_convergence():
    est = make()
    feed_static(est, 20.0, [0.0, 0.05, 0.0], accel=(0, 0, -G))  # spurious "rotation" is actually bias? see below
    # IMU is stationary but the gyro reads 0.05 rad/s: orientation (per accelerometer) does not change => this is bias
    r = est.frame_shift(19.0, 19.0 + 1 / 30)
    assert abs(r.dx) < 0.3  # without compensation it would be 1.5 px


def test_pure_translation_with_velocity_aid():
    est = make()
    # level flight at 6 m/s along body x; nadir camera => body x = -y of frame (content moves DOWN the frame... sign is checked)
    est.push_heading(0.0, 0.0)
    feed_static(est, 3.0, [0, 0, 0], v=[6.0, 0, 0])
    r = est.frame_shift(2.0, 2.0 + 1 / 30)
    # camera moves forward (body x); in the nadir frame body x = -y_camera => camera moves in -y => content in +y
    assert r.dy == pytest.approx(K.f * 6.0 / 30 / 30, rel=0.03) and abs(r.dx) < 0.05


def test_gimbal_change_shifts_content():
    est = make()
    est.push_gimbal(0.0, np.deg2rad(60))
    est.push_gimbal(2.0, np.deg2rad(60))
    feed_static(est, 2.0, [0, 0, 0], gimbal=np.deg2rad(60))
    est.push_gimbal(1.0, np.deg2rad(60))
    est.push_gimbal(1.0 + 1 / 30, np.deg2rad(60.5))
    r = est.frame_shift(1.0, 1.0 + 1 / 30)
    # camera tilts down towards nadir: content moves up (dy<0), magnitude ~ f * dθ
    assert r.dy < 0 and abs(r.dy) == pytest.approx(K.f * np.deg2rad(0.5), rel=0.1)


def test_no_imu_coverage_is_invalid():
    est = make()
    est.push_gimbal(0.0, 1.2)
    est.push_height(0.0, 20.0)
    r = est.frame_shift(0.0, 0.03)
    assert not r.valid and r.reason == "no_imu"


def test_nan_and_outlier_samples_are_dropped():
    est = make()
    feed_static(est, 1.0, [0, 0, 0])
    est.push_imu(1.0, np.array([np.nan, 0, 0]), np.array([0, 0, -G]))
    est.push_imu(1.005, np.array([200.0, 0, 0]), np.array([0, 0, -G]))
    est.push_imu(1.01, np.zeros(3), np.array([0, 0, -G]))
    r = est.frame_shift(0.9, 1.0)
    assert r.valid and est.stats["dropped"] == 2


def test_missing_height_is_invalid():
    est = make()
    est.push_gimbal(0.0, 1.2)
    for i in range(200):
        est.push_imu(i / 200, np.zeros(3), np.array([0, 0, -G]))
    assert est.frame_shift(0.5, 0.53).reason == "no_height"


def test_hover_replay_accuracy_and_calibration():
    seq = synth.generate(synth.SynthConfig(mode="hover", duration=40.0, seed=11))
    out = replay(seq, EgoConfig(camera=K), stride=1)
    ok = out["valid"] & (out["t"] > 8)
    err = np.hypot(out["dx"] - out["tdx"], out["dy"] - out["tdy"])[ok]
    assert np.sqrt((err**2).mean()) < 0.6
    assert (err <= 1.0).mean() > 0.9


def test_linear_replay_beats_zero_baseline():
    seq = synth.generate(synth.SynthConfig(mode="linear", duration=40.0, seed=12, gimbal_deg=90.0))
    out = replay(seq, EgoConfig(camera=K), stride=1)
    ok = out["valid"] & (out["t"] > 8)
    e_phys = np.hypot(out["dx"] - out["tdx"], out["dy"] - out["tdy"])[ok]
    e_zero = np.hypot(out["tdx"], out["tdy"])[ok]
    assert np.sqrt((e_phys**2).mean()) < 0.3 * np.sqrt((e_zero**2).mean())


def test_dropouts_inflate_uncertainty_not_break():
    base = synth.generate(synth.SynthConfig(mode="hover", duration=30.0, seed=13))
    gap = synth.generate(synth.SynthConfig(mode="hover", duration=30.0, seed=13, dropout_prob=0.05))
    o0 = replay(base, EgoConfig(camera=K))
    o1 = replay(gap, EgoConfig(camera=K))
    assert o1["valid"].mean() > 0.95
    assert np.nanmedian(o1["sigma"]) >= np.nanmedian(o0["sigma"])
    e1 = np.hypot(o1["dx"] - o1["tdx"], o1["dy"] - o1["tdy"])[o1["valid"] & (o1["t"] > 8)]
    assert np.sqrt((e1**2).mean()) < 1.5


def test_covariance_is_roughly_calibrated():
    seq = synth.generate(synth.SynthConfig(mode="mixed", duration=60.0, seed=14))
    out = replay(seq, EgoConfig(camera=K))
    ok = out["valid"] & (out["t"] > 10)
    d = np.stack([out["dx"] - out["tdx"], out["dy"] - out["tdy"]], axis=1)[ok]
    cov = out["cov"][ok]
    m2 = np.einsum("ni,nij,nj->n", d, np.linalg.inv(cov), d)
    frac = (m2 < 5.99).mean()  # 95% for chi-square with 2 degrees of freedom
    assert 0.85 < frac <= 1.0


def test_result_tuple_interface_matches_dal():
    est = make()
    feed_static(est, 1.0, [0, 0, 0])
    dx, dy, conf = est.frame_shift(0.5, 0.53).as_tuple()
    assert 0.0 <= conf <= 1.0 and np.isfinite(dx) and np.isfinite(dy)
    assert rot.wrap_pi(0.0) == 0.0


# ---------- design decisions D-012/D-013: FC velocity is mandatory; height source is a parameter ----------
def feed_no_velocity(est, secs=2.0, rate=200.0, h=30.0):
    est.push_gimbal(0.0, np.pi / 2)
    for i in range(int(secs * rate)):
        t = i / rate
        est.push_imu(t, np.zeros(3), np.array([0, 0, -G]))
        if i % 20 == 0:
            est.push_height(t, h)


def test_without_velocity_is_invalid_by_default():
    est = make()
    feed_no_velocity(est)
    r = est.frame_shift(1.0, 1.0 + 1 / 30)
    assert not r.valid and r.reason == "no_velocity" and np.isnan(r.dx)


def test_zero_velocity_prior_does_not_make_it_valid():
    est = make(zero_vel_sigma=0.5)
    feed_no_velocity(est)
    assert est.frame_shift(1.0, 1.0 + 1 / 30).reason == "no_velocity"


def test_research_flag_allows_running_without_velocity():
    est = make(require_velocity=False)
    feed_no_velocity(est)
    assert est.frame_shift(1.0, 1.0 + 1 / 30).valid


def test_stale_velocity_is_invalid():
    est = make()
    est.push_velocity(0.0, np.zeros(3), 0.1)
    feed_no_velocity(est, secs=3.0)
    est.push_velocity(0.0, np.zeros(3), 0.1)  # stale: time 0.0 while the frame is at 2.5 s
    assert est.frame_shift(2.5, 2.5 + 1 / 30).reason == "stale_velocity"


def test_height_source_primary_vs_ignored():
    est = make(height_source=HeightSource.RANGEFINDER)
    est.push_gimbal(0.0, np.pi / 2)
    est.push_velocity(0.0, np.zeros(3), 0.1)
    for i in range(400):
        t = i / 200
        est.push_imu(t, np.zeros(3), np.array([0, 0, -G]))
        if i % 20 == 0:
            est.push_velocity(t, np.zeros(3), 0.1)
            est.push_height(t, 30.0, HeightSource.FUSED_FC)  # wrong source: ignored
    r = est.frame_shift(1.0, 1.03)
    assert r.reason == "no_height" and est.stats["height_ignored"] > 0


def test_height_fallback_used_when_primary_missing():
    est = make(height_source=HeightSource.RANGEFINDER, height_fallback=HeightSource.FUSED_FC)
    est.push_gimbal(0.0, np.pi / 2)
    for i in range(400):
        t = i / 200
        est.push_imu(t, np.zeros(3), np.array([0, 0, -G]))
        if i % 20 == 0:
            est.push_velocity(t, np.zeros(3), 0.1)
            est.push_height(t, 30.0, HeightSource.FUSED_FC)
    r = est.frame_shift(1.0, 1.03)
    assert r.valid and r.height_source == "FUSED_FC"


def test_height_trust_scales_uncertainty():
    def sig(trust):
        est = make(height_trust=trust)
        feed_static(est, 3.0, [0, 0, 0], v=[5.0, 0, 0])
        return est.frame_shift(2.0, 2.0 + 1 / 30).sigma

    assert sig(0.2) > sig(1.0)


def test_height_trust_validation():
    with pytest.raises(ValueError, match="height_trust"):
        make(height_trust=0.0)


def test_estimator_accepts_camera_model_and_scales_with_resolution():
    from byeol.cameramodel import CameraModel

    big = CameraModel.from_fov(1280, 720, hfov_deg=70.0)
    out = {}
    for name, c in (("big", big), ("small", big.resized(640, 360))):
        est = EgoEstimator(EgoConfig(camera=c))
        feed_static(est, 3.0, [0, 0, 0], v=[5.0, 0, 0])
        out[name] = est.frame_shift(2.0, 2.0 + 1 / 30)
    assert out["big"].dy == pytest.approx(2 * out["small"].dy, rel=1e-3)
