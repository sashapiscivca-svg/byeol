---
license: mit
language:
  - en
  - uk
tags:
  - drone
  - uav
  - imu
  - ego-motion
  - search-and-rescue
  - visual-tracking
  - uncertainty-estimation
  - physics
  - numpy
  - no-weights
datasets:
  - byeol-synth-v1
---

# Model card: byeol v0.1.0

**English** · [Українська](MODEL_CARD.uk.md)

> **This model has no weights.** byeol is a deterministic estimator. The "model" is the algorithm in [`src/byeol`](src/byeol) together with its configuration. This card follows the usual model-card structure so that users can judge where it is fit for use and where it is not.

## Model details

| | |
|---|---|
| **Developed by** | The byeol Authors (the *uju* open search-and-rescue drone project) |
| **Version / date** | 0.1.0 · 2026-10-15 |
| **Type** | Physics-based state estimator: Mahony attitude filter, velocity Kalman filter, plane-induced homography, linearised uncertainty propagation |
| **Learned parameters** | None. Tunable noise parameters are set by hand from sensor assumptions, not fitted to data. Two fusion coefficients (0.3/resp, gate 0.1) were fitted on synthetic renders (E-004). |
| **Input** | IMU, flight-controller velocity (NED), height above surface, gimbal angle, optional heading, camera model |
| **Output** | Inter-frame image shift `(dx, dy)` in px, 2×2 covariance, confidence, validity flag and refusal reason |
| **Dependencies** | numpy (core). Optional: OpenCV, SciPy, PyYAML. |
| **License** | MIT |
| **Documentation** | [Technical report](docs/TECHNICAL_REPORT.md) · [Integration guide](docs/INTEGRATION.md) · [Design decisions](docs/DESIGN_DECISIONS.md) |

## Intended use

**Primary use.** byeol gives a visual tracker on a search-and-rescue drone an estimate of camera motion that does not depend on scene texture. The target case is open water after floods or at sea, where image-based methods fail. byeol is one input to a human-in-the-loop system: it helps the system keep a candidate person in frame, and a rescuer decides what to do.

**Also suitable for:**

- research and teaching on visual-inertial geometry and uncertainty;
- benchmarking ego-motion methods on the byeol-synth-v1 dataset;
- wildlife and livestock counting flights;
- disaster mapping.

**Intended users:** volunteer and professional rescue teams with technical support, researchers, and drone developers.

## Out-of-scope and prohibited uses

- **Flight control or safety.** byeol must never command an aircraft or sit in a stabilisation, collision-avoidance or failsafe loop. It has no such certification and was not designed for it.
- **Sole basis for decisions about people.** byeol only estimates camera motion. It does not detect people, and its output must not be the only basis for any decision about them.
- **Uses that conflict with the project's purpose.** These include weapons, targeting, military reconnaissance, persecution, mass or covert surveillance, and identifying individuals. The MIT license does not legally restrict use, but we ask everyone to respect the responsible-use statement in [ETHICS.md](ETHICS.md).

## How it works (summary)

1. A Mahony filter estimates attitude and gyro bias.
2. The bias-corrected gyro increment gives the inter-frame rotation.
3. The velocity Kalman filter, updated with FC velocity, gives the translation.
4. An exact plane homography (from height and plane normal) maps the reference pixel from frame t₀ to frame t₁.
5. A numerical Jacobian over 10 error sources gives the covariance.

Details are in the [technical report](docs/TECHNICAL_REPORT.md) §2.

## Evaluation

**Data:** the synthetic byeol-synth-v1 benchmark (7 scenarios × 3 seeds × 40 s). Setup: 640×360 camera, f = 457 px, 30 m altitude, gimbal 75°, IMU 200 Hz, 30 fps. The first 10 s of each sequence are excluded.

**Metrics:**

- RMSE of the pixel error at the image centre;
- the share of frames with error ≤1 px (invalid frames count as misses);
- 95 % covariance coverage;
- a "no motion" baseline for comparison.

| Scenario | RMSE stride 1 / 5 (px) | ≤1 px stride 1 / 5 | 95 % coverage stride 1 / 5 |
|---|---|---|---|
| hover | 0.08 / 0.35 | 100 % / 100 % | 1.00 / 1.00 |
| cruise 5 m/s | 0.08 / 0.34 | 100 % / 99.6 % | 1.00 / 1.00 |
| gusts 4 m/s² | 0.14 / 0.67 | 100 % / 85.9 % | 1.00 / 1.00 |
| vibration 70 Hz | 0.15 / 0.43 | 100 % / 99.7 % | 1.00 / 1.00 |
| IMU noise ×10 | 0.40 / 1.08 | 99.8 % / 59.2 % | 0.99 / 0.99 |
| gyro bias 0.05 rad/s | 0.32 / 1.58 | 100 % / 54.9 % | 0.94 / **0.83** |
| IMU dropouts 3 % | 0.77 / 2.53 | 88.1 % / 37.5 % | 0.97 / 0.96 |

**Comparison with image-based phase correlation over synthetic water** (stride 1, share of frames ≤1 px):

| Texture contrast | Physics | Phase correlation |
|---|---|---|
| 25 | 100 % | 97.5 % |
| 1.5 | 100 % | 86.5 % |
| 0.5 | 100 % | 44.4 % |
| 0.3 | 100 % | 24.2 % |

**Compute:** 3.7 % of one x86 core at 200 Hz IMU and 30 fps. The estimate for a Raspberry Pi Zero 2 W is 29–55 % of one core. **It has not been measured on that hardware.**

**Disaggregated results** cover stride, flight regime, sensor degradation, camera resolution (thermal and RGB) and lens distortion. They are in the technical report §5 and in [`experiments/`](experiments/).

## Limitations and risks

| Limitation | Consequence | Mitigation |
|---|---|---|
| **No real-world validation.** All results are synthetic, from a generator that shares the estimator's geometric assumptions. | Real accuracy will be lower, and by an unknown amount. | Treat the numbers as an upper bound. Validate on your own logs before operational use. Contribute logs ([RECORDING_PROTOCOL.md](docs/RECORDING_PROTOCOL.md)). |
| The surface is assumed to be a flat plane. | Waves, terrain, shorelines and tall objects cause unmodelled error. Moving water is not compensated. | Use nadir or near-nadir views where possible. Mind the stated uncertainty. |
| Mandatory FC velocity | No output without it | Stream FC velocity at 5–10 Hz |
| Rolling shutter, gimbal dynamics, lever arm, fisheye lenses and large camera–IMU time offsets are not modelled. | Bias on real hardware | Calibrate the time offset. Use a stabilised gimbal. Set the distortion model. |
| Covariance is too small under a large unknown gyro bias at large strides (coverage 0.83). | Overconfidence in rare cases | Let the gyro bias converge (about 10–20 s) before relying on the output. Use short strides. |
| With IMU dropouts, the estimate is honestly low-confidence. A tracker that cannot use σ may perform worse than an image-only one (E-006). | Tracking loss | Gate on `conf` (about 0.1) and propagate σ into the tracker. |
| Noise defaults and height σ values are assumptions. | Miscalibrated uncertainty on real sensors | Calibrate from static recordings and datasheets. |

**Failure behaviour.** byeol refuses explicitly with `valid=False` and a reason code. It does not degrade silently. A system built on byeol must handle refusals by falling back to an image estimate with its own gate, or to zero motion.

**A missed person can mean a lost life.** Do not present byeol-assisted tracking to operators as more reliable than it has been shown to be in your own conditions.

## Ethical considerations

- byeol processes no images of people and stores no personal data. It works only with motion sensors.
- The project is intended for civilian humanitarian use. Our responsible-use statement is in [ETHICS.md](ETHICS.md).
- Data: no third-party datasets were used. The published dataset is fully synthetic and contains no people, locations or personal data.

## Environmental impact

There is no training, so no training compute was used. Running all experiments in this release takes about one hour on one CPU core.

## Citation

See [CITATION.cff](CITATION.cff).
