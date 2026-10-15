# Design decisions

**English** · [Українська](DESIGN_DECISIONS.uk.md)

This file records the main design decisions in byeol. For each one it gives the reason and the alternatives we considered. Identifiers (D-xxx) are referenced from code comments and from the [technical report](TECHNICAL_REPORT.md). Experiment IDs (E-xxx) refer to §5 of the report.

| ID | Decision | Status |
|---|---|---|
| [D-001](#d-001) | Pure deterministic physics, no learned component | accepted |
| [D-002](#d-002) | Mahony complementary filter for attitude, not a full EKF | accepted |
| [D-003](#d-003) | Frames and signs: NED / FRD / OpenCV camera | accepted |
| [D-004](#d-004) | Exact plane-induced homography, not linearised flow | accepted |
| [D-005](#d-005) | Own consistent synthetic generator, not a full simulator | accepted |
| [D-006](#d-006) | Translation from flight-controller velocity | accepted |
| [D-007](#d-007) | Uncertainty by linear propagation with a numerical Jacobian | accepted |
| [D-008](#d-008) | Inverse-variance fusion with phase correlation, σ = 0.3/resp | accepted |
| [D-009](#d-009) | Evaluation camera 640×360, f = 457 px | accepted |
| [D-010](#d-010) | Handling of IMU gaps, outliers and NaN | accepted |
| [D-011](#d-011) | Acceptance rules for third-party datasets and simulators | accepted |
| [D-012](#d-012) | FC velocity is a mandatory input; without it `valid=False` | accepted |
| [D-013](#d-013) | Height source is a parameter | accepted |
| [D-014](#d-014) | TartanAir and Blackbird excluded | accepted |
| [D-015](#d-015) | PX4 SITL / Gazebo as a later validation step | planned |
| [D-016](#d-016) | Real flights as the first real-world test | planned |
| [D-017](#d-017) | Fully configurable camera model | accepted |
| [D-018](#d-018) | YAML camera profiles and chessboard calibration | accepted |
| [D-019](#d-019) | SI / NED / single clock inside; converters at the boundary | accepted |
| [D-020](#d-020) | Bridge to the dal tracker for end-to-end evaluation | accepted (not released) |
| [D-021](#d-021) | Proposals for the dal tracker | proposal |

---

### D-001

**Pure deterministic physics, no learned component.**

- **Why:**
  - No openly licensed flight data with synchronised video and IMU matches the scenario of a rescue drone over water.
  - Training on our own synthetic data would only learn the generator's assumptions.
  - Determinism keeps the system auditable, and it runs on numpy on a Pi-class computer.
- **Alternatives:** a small MLP correction to `(dx, dy)` trained on synthetic data was rejected as circular, with no real evidence behind it. Training on TUM VI or TII Racing was deferred until real logs exist.
- **Open:** once real logs exist, the residual error against optical flow could become the target of a small learned correction. That correction would sit outside any control loop.

### D-002

**Mahony complementary filter with gyro-bias estimation. A full EKF is not needed now.**

- **Why:**
  - The inter-frame shift is set mainly by the gyro *increment* over about 33 ms. Absolute tilt only sets the plane normal, a weak effect.
  - Mahony is cheap, stable and needs no noise-matrix tuning.
  - The accelerometer is low-pass filtered (τ = 0.08 s) and gated by `| |f| − g |`. Without the low-pass filter, 70 Hz vibration caused about 3° of tilt error. With it, the error is below 1.5°.
- **Alternatives:**
  - EKF/MEKF: more accurate uncertainty, but 5–10× the cost. Deferred.
  - Madgwick: equivalent quality, but bias estimation is less direct.

### D-003

**World NED, body FRD, camera x-right / y-down / z-forward (OpenCV). Gimbal 0 = horizon, π/2 = nadir.**

- **Output sign:** the shift of image content, matching OpenCV `phaseCorrelate` and the dal tracker. A synthetic-frame test checks the sign.
- **Why:** PX4 and ArduPilot use NED/FRD, so this needs the fewest conversions on board.
- **Alternative:** ENU/FLU (ROS REP-103) is supported through converters (D-019).

### D-004

**Exact plane-induced homography.**

The method: pixel → ray → intersection with the plane `nᵀP = d` → transform into frame 1 → project.

- **Why:**
  - One code path handles rotation, translation and gimbal change.
  - There is no small-angle limit, so it works at five-frame strides and in gusts.
  - Rays towards the horizon naturally give NaN.
- **Alternative:** linearised flow is used only as an analytic test oracle.
- **Limitation:** the surface is assumed flat.

### D-005

**Own consistent synthetic generator (`synth.py`).**

Attitude follows the thrust direction, so the accelerometer and gyro never contradict each other. Vibration and noise are applied to the sensors only.

- **Why:** pure numpy, seed-deterministic and fast. The ground truth is computed independently of `camera.py`.
- **Alternatives:**
  - PX4 SITL + Gazebo: deferred (D-015).
  - MuJoCo: no ready drone model with IMU noise.
- **Risk:** the noise and dynamics models are ours, so the results are an upper bound.

### D-006

**Translation through the flight controller's velocity estimate. `VelocityKF` uses it as a measurement.**

- **Why:**
  - Double integration of the accelerometer drifts: a bias of 0.05 m/s² gives 0.5 m/s of error after 10 s.
  - The FC already runs a deterministic EKF fusing GPS, optical flow and barometer.
- **Alternative:** velocity from the image. Rejected, because it brings back exactly the unreliability over water that byeol exists to avoid.

### D-007

**`cov = J S Jᵀ + floor² I`, where J is a forward-difference Jacobian over 10 parameters. `conf = 1/(1+λmax)`.**

- **Why:**
  - It is cheap: 11 shift evaluations.
  - It works for any geometry.
  - Its calibration is testable through 95 % coverage. Results: 0.94–1.00 at stride 1 and 0.96–1.00 at stride 5. The exception is a large unmodelled gyro bias at stride 5, which gives 0.83.
- **Alternative:** an analytic Jacobian, which is faster but harder to maintain. Deferred until on-target measurements exist.

### D-008

**`σ_v = max(0.3/resp, 0.2)` px. Phase correlation with `resp < 0.1` is rejected. A 99 % Mahalanobis gate applies.**

- **Why:** on synthetic water, median error × resp ≈ 0.31–0.37 (E-004).
- **Limitation:** the coefficients were fitted on our renders and must be recalibrated on real footage.
- **Alternative:** hard switching on a `resp` threshold. It is worse at intermediate values.

### D-009

**Evaluation setup: 640×360, f = 457 px (HFOV ≈ 70°), gimbal 75°, height 30 m, 30 fps, IMU 200 Hz. The first 10 s of each sequence are excluded.**

- **Why:** this is a typical working resolution for an on-board tracker on a Pi-class computer.
- **Scaling:** pixel errors scale linearly with f. Results are reported at strides 1 and 5, because at stride 1 the "no motion" baseline is already fairly good in hover.

### D-010

**IMU sample handling.**

- **Dropped:** samples with NaN or Inf, `|ω| > 35 rad/s`, or non-increasing time.
- **Gaps:** an interval over 30 ms is a gap. The last rate is held, and the gap time is added to the uncertainty (`gap_rate_sigma` 0.3 rad/s).
- **Extrapolation:** a query more than 50 ms past the last IMU sample returns `no_imu`.
- **Why:** byeol must not invent data or break the pipeline, and it should state the larger uncertainty honestly.
- **Alternative:** linear interpolation across gaps. It is worse during jerks.

### D-011

**Third-party data: only CC0, CC BY, MIT, Apache or BSD licences are accepted.**

- **Rejected:** NC, ND, SA combined with NC, and "no licence stated".
- **Simulators** with proprietary assets are allowed only as external tools. They are never embedded or redistributed.
- **Current use:** no third-party dataset is shipped with byeol or used in this release.
- **Full review:** [THIRD_PARTY_DATA.md](THIRD_PARTY_DATA.md).

### D-012

**FC velocity is mandatory.**

If no velocity sample has arrived, or the last one is older than `vel_max_age` = 1 s, `frame_shift` returns `valid=False` with reason `no_velocity` or `stale_velocity`.

- **Why:** without velocity, a 5 m/s flight gives 2.46 px RMSE against 2.93 px for "no motion" (E-001), which is almost no information. An honest refusal is safer than a quietly bad estimate that a consumer might trust.
- **Rejected:** returning an estimate with a large covariance. Consumers may ignore `cov`.
- **Kept:** `require_velocity=False` exists for research only.

### D-013

**Height source is a parameter: `HeightSource` = {FUSED_FC, RANGEFINDER, BARO, MANUAL}.**

The configuration also takes a fallback source, a trust factor and a maximum age.

- **Why:** rangefinders give spurious returns over water and are the most accurate sensor over land. The code must not depend on one particular hardware choice.
- **Assumption:** the default σ values (0.4 / 0.1 / 1.0 / 2.0 m) are assumptions to calibrate.

### D-014

**TartanAir and Blackbird are excluded.**

- **TartanAir:** its development was funded by a military research agency. That conflicts with the project's responsible-use principles ([ETHICS.md](../ETHICS.md)), which ask that components be free of military origin or purpose.
- **Blackbird:** the repository is MIT, but no licence for the data itself was found.

### D-015

**PX4 SITL with Gazebo Harmonic is planned as an intermediate validation step.**

Both are BSD-3 / Apache-2.0. The plan: standard PX4 models, a custom water texture, recording under the public recording protocol, then the same evaluation. It is not started yet.

### D-016

**Real flights with synchronised IMU and video are the first real-world test of byeol together with a tracker.**

A draft recording protocol is published as [RECORDING_PROTOCOL.md](RECORDING_PROTOCOL.md). This is the only way to close the synthetic-to-real gap and to calibrate the noise parameters.

### D-017

**`CameraModel` can be defined in pixels, millimetres or field of view.**

It also takes a resolution, Brown–Conrady distortion, and the `rgb`/`lwir` type. `resized`, `binned` and `cropped` follow OpenCV's pixel-centre convention.

- **Why:** different regions and teams use different cameras, resolutions and crops, and a hard-coded f breaks portability.
- **Thermal sensors:** errors are also reported in mrad (E-005).
- **Limitation:** inverse distortion by fixed-point iteration converges only for monotonic models. Fisheye lenses are not supported.

### D-018

**Seven parametric YAML templates (`schema: 1`), with strict validation. Unknown keys are errors, so typos fail loudly.**

- **Formats:** YAML (needs pyyaml) or JSON (no extra dependency).
- **Calibration:** a chessboard CLI (`scripts/calibrate_camera.py`, OpenCV). On synthetic boards, fx comes out within 0.5 % (exact corners) and 2 % (rendered images).
- **Template values** are illustrative. No manufacturer specifications are claimed.

### D-019

**Inside byeol: SI units, NED/FRD and one clock. Converters run at the boundary.**

The converters are `units`, `frames` and `timebase` (UTC↔GPS with leap seconds, GPS week + TOW, `ClockMap`).

- **Why:** some regions use feet, knots and miles, others metres and km/h. ROS tools use ENU/FLU, while PX4 and ArduPilot use NED/FRD. GPS time is 18 s ahead of UTC (since 2017-01-01).
- **Limitations:**
  - The leap-second table needs updating if IERS announces new leap seconds.
  - GPS week rollover is not handled.
  - Geodetic transforms are not needed and not implemented.

### D-020

**A bridge to the sibling dal tracker.**

The bridge subclasses dal's classical tracker so it can override ego-motion with the modes image, byeol and fused, without modifying dal. It is used for the end-to-end evaluation E-006.

It depends on dal, which has not been published yet, so the bridge is **not part of this release**. The results are published in `experiments/eval_dal_bridge.*` for transparency.

### D-021

**Proposals for the dal tracker, based on E-006 and verified only on synthetic data.**

1. Raise the ego-motion confidence gate from 0.02 to about 0.1.
2. Add a pluggable `ego_fn(prev, cur) -> (dx, dy, conf)`.
3. Pass the ego-motion σ into the tracker's Kalman filter. Without it, low-confidence ego-motion during IMU dropouts is either ignored or trusted as exact.
