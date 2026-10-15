"""Measure the cost of byeol on THIS machine (not on a Pi!): µs per IMU sample, µs per frame_shift query, peak memory.

Reproduce (from the repository root): python scripts/bench_budget.py
The Pi Zero 2 W figure is only a multiplier-based estimate (see docs/TECHNICAL_REPORT.md); no on-hardware measurement was made.
"""
import resource
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from byeol import synth
from byeol.cameramodel import CameraModel
from byeol.estimator import EgoConfig, EgoEstimator

K = CameraModel.from_pixels(640, 360, 457.0, 457.0, 320.0, 180.0)
seq = synth.generate(synth.SynthConfig(mode="mixed", duration=20.0, seed=1))
est = EgoEstimator(EgoConfig(camera=K))
est.push_gimbal(0.0, seq.gimbal(0.0))
est.push_velocity(0.0, [0.0, 0.0, 0.0], 0.2)
for t, h in zip(seq.height_t, seq.height):
    if t < 1.0:
        est.push_height(t, h)
t_imu = 0.0
n = 0
for t, g, a in zip(seq.imu_t, seq.gyro, seq.accel):
    s = time.perf_counter()
    est.push_imu(t, g, a)
    t_imu += time.perf_counter() - s
    n += 1
est.push_height(20.0, 30.0)
t_fr = 0.0
m = 0
for t in seq.frame_t[2:]:
    est.push_velocity(float(t), [0.0, 0.0, 0.0], 0.2)
    s = time.perf_counter()
    est.frame_shift(float(t) - 1 / 30, float(t))
    t_fr += time.perf_counter() - s
    m += 1
us_imu, us_fr = 1e6 * t_imu / n, 1e6 * t_fr / m
load = (200 * us_imu + 30 * us_fr) / 1e6
print(f"push_imu: {us_imu:.0f} µs/sample; frame_shift: {us_fr:.0f} µs/query")
print(f"load at 200 Hz IMU + 30 fps: {100 * load:.1f}% of one core (this machine)")
print(f"estimate for Pi Zero 2 W (x8..x15): {100 * load * 8:.0f}%..{100 * load * 15:.0f}% of one core")
print(f"process peak memory (RSS, MB): {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024:.0f}")
