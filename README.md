# byeol — physics-based camera ego-motion for search-and-rescue drones

**English** · [Українська](README.uk.md)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)
![Dependencies: numpy](https://img.shields.io/badge/core%20deps-numpy-informational.svg)
![Status: research preview](https://img.shields.io/badge/status-research%20preview-orange.svg)

![byeol estimate vs ground truth, and error vs water texture](docs/assets/byeol-overview.png)

*Left: the frame-to-frame image shift that byeol predicts from IMU, velocity and height (orange, with its ±2σ band) follows the ground truth (black) through gusts. Right: as the water surface loses texture, the image-based estimate (phase correlation) degrades by orders of magnitude while the physics estimate does not change. Synthetic data only.*

> **Responsible use.** byeol is built to help find people in danger — after floods, at sea, in forests, in disaster zones. Please read [ETHICS.md](ETHICS.md) before using it. It states what we built this for and what we ask you not to use it for.

**byeol** (별, Korean for *star*) estimates how the image of a drone camera moves between two video frames — the apparent pixel shift `(dx, dy)` with a 2×2 covariance — **from the drone's own sensors**: the IMU, the velocity estimate of the flight controller, the height above the surface and the gimbal angle.

Image-based motion estimates such as phase correlation or optical flow work poorly over open water, where there is little texture, sun glint and waves. Tracking a person in the water is hardest there, and that is where search and rescue happens. byeol gets the camera motion from physics instead, so the estimate does not depend on what the camera sees. A visual tracker can then use it to keep a candidate in frame.

| | |
|---|---|
| **What it is** | A deterministic estimator: Mahony attitude filter + velocity Kalman filter + exact plane-induced homography + linearised uncertainty propagation |
| **What it is not** | Not a learned model. There are **no trained weights**. It does not control the drone and it is not part of any flight-safety loop. |
| **Dependencies** | `numpy` only for the core. OpenCV/SciPy/PyYAML are optional (evaluation, calibration, YAML profiles). |
| **Target hardware** | Raspberry Pi Zero 2 W class. This is an estimate only: about 3.7 % of one desktop core, not yet measured on a Pi. |
| **Maturity** | Research preview (v0.1.0). Validated **only on synthetic data**. No real flights yet. See [Limitations](#limitations). |
| **License** | [MIT](LICENSE). Code, configs, results and the synthetic dataset are all MIT. |

## Contents

- [Quick start](#quick-start)
- [How it works](#how-it-works)
- [Results (synthetic)](#results-synthetic)
- [Release contents](#release-contents)
- [Dataset: byeol-synth-v1](#dataset-byeol-synth-v1)
- [Limitations](#limitations)
- [Documentation](#documentation)
- [Citing](#citing) · [Contributing](#contributing) · [License](#license)

## Quick start

```bash
git clone https://github.com/sashapiscivca-svg/byeol.git && cd byeol
pip install -e .            # core: numpy only
pip install -e ".[dev]"     # + tests, OpenCV, SciPy, PyYAML
python -m pytest tests -q   # ~120 tests, 1–4 min
```

Minimal use. All inputs are SI units, world frame NED, body frame FRD, and one clock for every stream:

```python
import numpy as np
from byeol.cameramodel import CameraModel
from byeol.estimator import EgoConfig, EgoEstimator, HeightSource

cam = CameraModel.from_fov(640, 360, hfov_deg=70)          # or byeol.camprofile.load_camera("configs/cameras/....yaml")
est = EgoEstimator(EgoConfig(camera=cam, height_source=HeightSource.FUSED_FC))

# Feed the streams in time order as they arrive:
est.push_imu(t, gyro_frd_rad_s, accel_frd_m_s2)              # 100–400 Hz
est.push_velocity(t, v_ned_m_s, sigma=0.2)                   # REQUIRED, 5–10 Hz (flight-controller EKF)
est.push_height(t, h_m)                                      # 5–10 Hz, height above the imaged surface
est.push_gimbal(t, theta_rad)                                # 0 = forward/horizon, pi/2 = nadir
est.push_heading(t, yaw_rad)                                 # optional, 2–5 Hz

r = est.frame_shift(t_prev_frame, t_cur_frame)
if r.valid:
    print(r.dx, r.dy, r.cov, r.conf)                         # px, px, 2x2 px^2, (0, 1]
else:
    print("no estimate:", r.reason)                          # e.g. no_velocity, no_height, horizon
```

`r.valid == False` is a deliberate, honest refusal. byeol does not return a quietly bad estimate. The full interface covers frames, signs, units, failure codes and fusion with phase correlation. See [docs/INTEGRATION.md](docs/INTEGRATION.md).

Reproduce the published numbers:

```bash
python scripts/make_dataset.py --check   # regenerates byeol-synth-v1 in memory and verifies it bit-exactly
python scripts/eval_dataset.py           # runs the estimator on the dataset files (~8 min, one core)
python scripts/eval_ego.py               # full E-001/E-002 evaluation incl. phase-correlation baseline (needs OpenCV)
```

## How it works

```
 IMU (gyro, accel) ──► Mahony attitude filter ──► tilt (plane normal) ─────────────────┐
        │                  (gyro bias estimate)                                         │
        └──► gyro integration (bias-corrected) ──► relative rotation R01 ──────────────┤
 FC velocity (NED) ──► velocity Kalman filter ──► ∫ v dt = camera translation t01 ─────┤──► exact plane homography ──► (dx, dy)
 height above surface ──────────────────────────► plane distance d ───────────────────┤      for reference pixel(s)
 gimbal angle ─────────────────────────────────► camera-to-body rotation ─────────────┘
                                                                    numerical Jacobian (10 params) ──► 2×2 covariance, conf
```

1. **Attitude.** A Mahony complementary filter integrates the gyro, corrects tilt from the accelerometer and estimates the gyro bias. The accelerometer correction is gated when `|f| ≠ g` and low-pass filtered against vibration.
2. **Velocity.** A 6-state linear Kalman filter estimates world velocity and accelerometer bias. It is updated by the flight controller's velocity estimate. **This input is mandatory**: double-integrating an accelerometer drifts within seconds. Without it the result is `valid=False`.
3. **Geometry.** For each reference pixel: back-project the ray in frame *t₀*, intersect it with the surface plane (height `h`, normal from attitude and gimbal), transform into frame *t₁* and project. This is the exact plane-induced homography with no small-angle linearisation. It handles rotation, translation and gimbal motion in one code path.
4. **Uncertainty.** A numerical Jacobian over 10 parameters (rotation 3, translation 3, height 1, plane normal 3) propagates gyro noise, residual bias, scale and time-sync error, IMU gaps, velocity covariance, and height and tilt error, plus a model floor. `conf = 1 / (1 + λmax(cov))`.

Details, equations and design rationale: [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md), [docs/DESIGN_DECISIONS.md](docs/DESIGN_DECISIONS.md).

## Results (synthetic)

All numbers below come from **synthetic data** produced by byeol's own generator. The setup: 640×360 camera, f = 457 px, height 30 m, gimbal 75°, IMU 200 Hz, 30 fps, 3 seeds × 40 s per scenario, first 10 s discarded. They show how the method behaves under controlled conditions. **They are not a measure of real-flight performance.**

**Physics vs. ground truth (experiment E-001).** Errors are in pixels at the image centre. "Zero" is the "no camera motion" baseline.

| Scenario | RMSE, stride 1 | ≤1 px, stride 1 | RMSE, stride 5 | ≤1 px, stride 5 | Zero baseline RMSE (stride 5) |
|---|---|---|---|---|---|
| hover (wobble) | 0.08 | 100 % | 0.35 | 100 % | 8.30 |
| cruise 5 m/s | 0.08 | 100 % | 0.34 | 99.6 % | 14.54 |
| gusts up to 4 m/s² | 0.14 | 100 % | 0.67 | 85.9 % | 19.82 |
| vibration 70 Hz | 0.15 | 100 % | 0.43 | 99.7 % | 8.30 |
| IMU noise ×10 | 0.40 | 99.8 % | 1.08 | 59.2 % | 23.57 |
| gyro bias 0.05 rad/s | 0.32 | 100 % | 1.58 | 54.9 % | 14.54 |
| IMU dropouts 3 % | 0.77 | 88.1 % | 2.53 | 37.5 % | 23.57 |
| cruise, **no FC velocity** | — (`valid=False` by design) | 0 % | — | 0 % | 14.54 |

**Physics vs. image-based phase correlation over synthetic water (E-002, stride 1).** Physics does not depend on texture: 0.19 px RMSE and 100 % of frames ≤1 px at every contrast. Phase correlation degrades as contrast falls: contrast 25 gives 0.34 px / 97.5 %, contrast 1.5 gives 0.71 px / 86.5 %, contrast 0.5 gives 1.31 px / 44.4 %, and contrast 0.3 gives 8.19 px / 24.2 %.

**Where byeol does *not* help.** On well-textured scenes the image is already good. At 30 fps with a tracker, the inter-frame motion is small enough that "no ego-motion" works as well. With 3 % IMU dropouts byeol's honest low-confidence output loses to a well-gated image estimate. These are reported in full in [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) §5.

Raw results: [`experiments/`](experiments/).

## Release contents

```
src/byeol/          the estimator (numpy) + synthetic generator, metrics, camera models, unit/frame/time converters
configs/cameras/    7 parametric camera templates (RGB and thermal LWIR); illustrative, not vendor specifications
scripts/            evaluation, sensitivity analysis, calibration CLI, dataset builder, dataset evaluator, CPU benchmark
experiments/        saved results of E-001…E-006 (JSON + Markdown tables)
data/byeol-synth-v1 synthetic benchmark: 21 sequences, raw sensor streams + ground truth (15 MB, MIT)
tests/              unit, analytic and replay tests
docs/               technical report, integration guide, design decisions, module map, third-party data review (EN + UK)
```

There is **no trained model** to download: byeol is the algorithm. The Hugging Face model repository holds the model card ([MODEL_CARD.md](MODEL_CARD.md)) and camera profiles. The dataset is published as a separate Hugging Face dataset.

## Dataset: byeol-synth-v1

A deterministic, seeded benchmark: 7 flight scenarios × 3 seeds × 40 s. Each sequence holds raw IMU (200 Hz), flight-controller velocity (10 Hz) and heading (5 Hz), height (10 Hz), gimbal angle per frame, frame timestamps (30 fps), and ground truth. The ground truth covers position, velocity, attitude quaternion, true height, true gyro bias, and the true pixel shift for stride 1 and 5. The dataset is regenerated bit-exactly by `scripts/make_dataset.py`. Dataset card: [data/byeol-synth-v1/README.md](data/byeol-synth-v1/README.md).

The same files are published on the Hugging Face Hub as [alexshynkarenk0/byeol-synth-v1](https://huggingface.co/datasets/alexshynkarenk0/byeol-synth-v1) (MIT, 15 MB, tag `v1`); the model card and camera profiles are at [alexshynkarenk0/byeol](https://huggingface.co/alexshynkarenk0/byeol) (no weights: byeol is an algorithm).

```python
from huggingface_hub import snapshot_download
path = snapshot_download("alexshynkarenk0/byeol-synth-v1", repo_type="dataset", revision="v1")
# python scripts/eval_dataset.py --data <path>
```

## Limitations

Read these before relying on byeol. The full list is in [MODEL_CARD.md](MODEL_CARD.md) and the technical report.

- **No real-world validation.** byeol has had no real flights, no real IMU and no real video. Every number comes from our own generator, which shares geometric assumptions with the estimator. So the results are an **upper bound**.
- **Flat-surface model.** Waves, terrain, the shoreline and objects above the plane are not modelled. Over water the surface itself moves.
- **Mandatory flight-controller velocity.** Without it byeol refuses to produce an output (`valid=False`).
- **Not modelled:** rolling shutter, gimbal lag and gimbal oscillation, IMU–camera lever arm, camera–IMU time offsets above about 3 ms, fisheye lenses.
- **Noise parameters are assumptions.** They are not calibrated on hardware. The Raspberry Pi budget is an extrapolation, not a measurement.
- **byeol is an aid, not an authority.** It feeds a tracker. A person decides. It must never be placed in a flight-control or safety loop.

## Documentation

| Document | English | Українська |
|---|---|---|
| Technical report (method, experiments, limitations) | [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) | [docs/TECHNICAL_REPORT.uk.md](docs/TECHNICAL_REPORT.uk.md) |
| Integration guide (API, frames, units, failure codes) | [docs/INTEGRATION.md](docs/INTEGRATION.md) | [docs/INTEGRATION.uk.md](docs/INTEGRATION.uk.md) |
| Design decisions D-001…D-021 | [docs/DESIGN_DECISIONS.md](docs/DESIGN_DECISIONS.md) | [docs/DESIGN_DECISIONS.uk.md](docs/DESIGN_DECISIONS.uk.md) |
| Model card | [MODEL_CARD.md](MODEL_CARD.md) | [MODEL_CARD.uk.md](MODEL_CARD.uk.md) |
| Dataset card | [data/byeol-synth-v1/README.md](data/byeol-synth-v1/README.md) | (bilingual) |
| Third-party datasets & simulators review | [docs/THIRD_PARTY_DATA.md](docs/THIRD_PARTY_DATA.md) | (bilingual) |
| Module map | [docs/MODULES.md](docs/MODULES.md) | (bilingual) |
| Responsible use | [ETHICS.md](ETHICS.md) | [ETHICS.uk.md](ETHICS.uk.md) |

## Citing

```bibtex
@software{byeol_2026,
  title   = {byeol: physics-based camera ego-motion for search-and-rescue drones},
  url     = {https://github.com/sashapiscivca-svg/byeol},
  author  = {{The byeol Authors}},
  year    = {2026},
  version = {0.1.0},
  license = {MIT}
}
```

See also [CITATION.cff](CITATION.cff).

## Contributing

We welcome contributions. Real flight logs recorded under a permissive license are the most valuable thing you can give. They are the one thing this project lacks. See [CONTRIBUTING.md](CONTRIBUTING.md). To report security issues or misuse, see [SECURITY.md](SECURITY.md).

## License

[MIT](LICENSE) © 2026 The byeol Authors. byeol is part of the *uju* family of open tools for search-and-rescue drones, alongside *hae* (person detection) and *dal* (tracking). Those projects are released separately.
