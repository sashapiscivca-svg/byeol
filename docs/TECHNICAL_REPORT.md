# byeol: Physics-Based Camera Ego-Motion for Search-and-Rescue Drones — Technical Report

**English** · [Українська](TECHNICAL_REPORT.uk.md)

Version 0.1.0 · 2026-10-15 · License: MIT

## Abstract

Visual trackers on drones must separate the motion of a target from the motion of the camera. The usual image-based estimates of camera motion (phase correlation, optical flow) fail exactly where maritime search and rescue happens: over open water, where the scene has little texture, sun glint and moving waves. byeol estimates the apparent inter-frame pixel shift of image content **from the drone's own sensors**. It uses the gyroscope and accelerometer, the flight controller's velocity estimate, the height above the surface and the gimbal angle. It returns the shift with a calibrated 2×2 covariance.

The estimator is deterministic and has no learned parameters. It runs in pure numpy at about 3.7 % of one desktop CPU core for 200 Hz IMU and 30 fps. On a synthetic benchmark it reaches 0.08–0.40 px RMSE at one-frame stride and 0.34–1.58 px at five-frame stride under nominal and degraded IMU conditions (640×360, f = 457 px, 30 m altitude). Its accuracy does not depend on texture: over synthetic water with low contrast, phase correlation is correct to ≤1 px in only 24–44 % of frames, while physics is correct in 100 %. We also report where the method does not help or is worse. All results are synthetic. No real-flight validation has been done yet.

## 1. Problem and scope

**Input** (one shared clock, SI units):

- IMU: angular rate (rad/s) and specific force (m/s²), in the FRD body frame, at 100–400 Hz.
- Flight-controller (FC) velocity estimate in NED (m/s), at ≥1 Hz (5–10 Hz recommended). **Mandatory.**
- Height above the imaged surface (m), from a configurable source (fused FC altimeter, rangefinder, barometer, manual).
- Gimbal pitch angle θ (0 = forward horizontal, π/2 = nadir).
- Optional FC heading.
- Camera model: intrinsics, resolution, distortion.

**Output** for a pair of frame timestamps (t₀, t₁): `EgoResult(dx, dy, cov, conf, valid, reason, sigma)`.

- `(dx, dy)` is the displacement in pixels of image content from frame t₀ to frame t₁, at a reference pixel (by default the image centre).
- `cov` is a 2×2 covariance in px².
- `conf = 1/(1+λmax(cov))`.
- `valid=False` with a machine-readable `reason` whenever an honest estimate is impossible.

**Out of scope, by design.** byeol never commands the aircraft and is never part of the flight-control or safety loop. Stabilisation and failsafes stay with a deterministic, certified flight stack such as PX4 or ArduPilot. byeol only *reads* data. It does not decide whether a downstream consumer should trust it. It returns `valid`, `conf` and `cov`.

## 2. Method

### 2.1 Frames and conventions

- World frame: NED. Body frame: FRD. Camera frame: x right, y down, z along the optical axis (OpenCV).
- Camera-to-body rotation for gimbal pitch θ, with columns written in FRD: `x_c = (0, 1, 0)`, `y_c = (−sin θ, 0, cos θ)`, `z_c = (cos θ, 0, sin θ)`. At nadir, the top of the image points forward along the body.
- The accelerometer measures specific force `f = Rᵀ(a − g)`. At rest and level, `f = (0, 0, −9.81)`.
- Output sign: +dx means content moved right, +dy means content moved down. This matches OpenCV `phaseCorrelate` and the `dal` tracker's `estimate_shift`. A synthetic-frame test checks this.

### 2.2 Attitude: Mahony complementary filter (D-002)

The filter state is a quaternion `q_wb` plus a gyro bias `b`. On each IMU sample:

1. Integrate the bias-corrected gyro rate.
2. Correct tilt with a PI term on the error between measured and predicted "down". The accelerometer passes through a low-pass filter with τ = 0.08 s, which suppresses vibration. Its weight is gated by `| |f| − g |`, so dynamic acceleration and vibration do not corrupt tilt. Without the low-pass filter, 70 Hz / 2.5 m/s² vibration caused about 3° of tilt error. With it, the error stays below 1.5°.
3. Optionally correct yaw from the FC heading.
4. Update the bias through the integral term.

The inter-frame rotation does **not** come from the filtered attitude. It comes from a separate cumulative quaternion of bias-corrected gyro increments. Over 33 ms the pixel shift depends mostly on that rotation increment. The absolute attitude only sets the surface-plane normal, a weak effect (1° of tilt is about 0.02 rad).

### 2.3 Translation: velocity Kalman filter (D-006, D-012)

A linear Kalman filter with six states (world velocity, accelerometer bias) runs as follows:

- **Predict** with the rotated specific force.
- **Update** with the FC velocity measurement and its σ.

Camera translation between frames is the trapezoidal integral of filtered velocity over [t₀, t₁]. Translation from the accelerometer alone drifts. A bias of 0.05 m/s² produces 0.5 m/s of velocity error in 10 s. FC velocity is therefore mandatory: when it is absent, or older than `vel_max_age` = 1 s, the result is `valid=False` with reason `no_velocity` or `stale_velocity`. For research use, a weak zero-velocity prior is available (`zero_vel_sigma`, for hover). It never makes a result valid.

### 2.4 Pixel shift: exact plane-induced homography (D-004)

The imaged surface is modelled as a plane `nᵀP = d` in the camera-0 frame. The normal `n` comes from attitude and gimbal, and `d` from height. For each reference pixel `u₀`:

```
ray      r  = K⁻¹ ũ₀                              (undistorted, if a distortion model is set)
point    P₀ = r · d / (nᵀ r)                      (intersection with the plane; NaN → "horizon" if nᵀr ≤ 0)
frame 1  P₁ = R₀₁ᵀ (P₀ − t₀₁)
pixel    u₁ = π(K P₁)                             (re-distorted)
shift    (dx, dy) = u₁ − u₀
```

`R₀₁ = R_bc(θ₀)ᵀ · ΔR_b · R_bc(θ₁)` combines the gyro-integrated body rotation with any change of gimbal angle. `t₀₁` is the integrated translation expressed in camera-0 coordinates. This form makes no small-angle or small-translation approximation. It therefore stays accurate at five-frame strides and during gusts, and a ray towards the horizon naturally yields NaN. The classical linearised flow (`dx ≈ −f(ω_y + v_x/Z)`) is used only as an analytic check in the tests.

### 2.5 Uncertainty: linearised propagation (D-007)

Uncertainty is propagated through ten parameters: rotation (3), translation (3), relative height (1) and plane normal (3). Their covariance `S` is block-diagonal:

- **Rotation.** It combines:
  - white gyro noise: `gyro_density² · Δt`;
  - residual bias: `σ_b² · Δt²`, where `σ_b` decays from 0.015 to 0.003 rad/s with τ = 12 s, and yaw stays pessimistic until a heading is seen;
  - gyro scale error: `(0.005 · ω Δt)²`;
  - camera–IMU sync error: `(0.003 s · ω)²`;
  - unobserved time inside IMU gaps: `(0.3 rad/s · t_gap)²`.
- **Translation.** `Δt² · (P_v + σ_model²)`, where `P_v` is the velocity covariance from the Kalman filter.
- **Height.** Relative 3 % combined with an absolute σ per source: fused FC 0.4 m, rangefinder 0.1 m, barometer 1.0 m, manual 2.0 m. The absolute σ is divided by a user trust factor. These σ values are **assumptions** and must be calibrated on hardware.
- **Normal.** Tilt σ = 0.03 rad.

The Jacobian `J` (2×10) is computed by forward differences, which costs 11 shift evaluations. The covariance is then:

```
cov = J S Jᵀ + floor² I       (floor = 0.05 px: rolling shutter, residual distortion, non-flat ground)
```

The 95 % ellipse coverage on synthetic data is 0.94–1.00 at stride 1. The estimate is conservative, with one documented exception (§5.1).

### 2.6 Gaps, outliers and invalid input (D-010)

- Rejected IMU samples: NaN or Inf values, `|ω| > 35 rad/s` (gyro saturation), and non-increasing timestamps.
- An interval over 30 ms is treated as a gap. The last rate is held (zero-order hold), and the gap time is added to the uncertainty.
- A query that extrapolates more than 50 ms past the last IMU sample returns `no_imu`.

Every refusal has a reason: `no_imu`, `no_height`, `low_height` (<1 m), `no_gimbal`, `no_velocity`, `stale_velocity`, `horizon`.

### 2.7 Optional fusion with phase correlation (D-008)

`byeol.fuse.fuse(physics, (dx_v, dy_v, resp))` combines the two estimates by inverse variance, using an empirical vision error model `σ_v = max(0.3/resp, 0.2)` px. Phase correlation with `resp < 0.1` is rejected. A Mahalanobis gate at 99 % (χ²₂ = 9.21) handles inconsistent estimates: the one with the larger σ is kept out. The coefficients were fitted on synthetic renders (E-004) and **must be recalibrated on real footage**.

### 2.8 Camera models (D-017, D-018)

`CameraModel` can be defined in three ways: in pixels (fx, fy, cx, cy), in millimetres (focal length and sensor size), or by field of view (HFOV, with optional VFOV). Each also takes a resolution, a Brown–Conrady distortion model (k1, k2, p1, p2, k3, as in OpenCV) and a type (`rgb` or `lwir`).

`resized`, `binned` and `cropped` follow OpenCV's pixel-centre convention. Distortion is applied only to the reference pixels, which costs almost nothing. Seven YAML templates and a chessboard calibration CLI are included. The template numbers are **illustrative** and are not vendor specifications. For thermal cameras with large pixels, we recommend reporting error in **milliradians** (`RMSE / f · 1000`), because the "≤1 px" criterion becomes unfairly lenient at low resolution.

## 3. Synthetic data generator (D-005)

`byeol.synth` generates physically consistent trajectories:

- World acceleration is a smooth sum of wobble sinusoids and Gaussian gusts.
- Attitude follows the thrust direction: `z_b = (g − a)/|g − a|`. The accelerometer therefore reads exactly `Rᵀ(a − g)`, with no contradiction between gyro and accelerometer.
- Truth is integrated at 1 kHz.
- Sensor errors include white noise, an initial bias, a bias random walk, an accelerometer bias, vibration (sensors only, assuming an ideal gimbal), IMU dropouts, a drifting noisy height, noisy FC velocity at 10 Hz and FC heading at 5 Hz.

The ground-truth shift is computed with **independent code**: ray → world plane → projection. It does not use `camera.ego_shift`, and a test checks the two against each other. **Known circularity:** the generator and the estimator share the flat-plane and pinhole assumptions. This is the main reason the results are an upper bound.

## 4. Evaluation protocol

- **Camera.** 640×360, fx = fy = 457 px (HFOV ≈ 70°), principal point at the centre, no distortion. The reference pixel is the image centre.
- **Flight.** Height 30 m, gimbal 75°, IMU 200 Hz, 30 fps, FC velocity 10 Hz (σ = 0.2 m/s), heading 5 Hz (σ = 0.03 rad), height 10 Hz (σ = 0.3 m plus drift).
- **Runs.** Three seeds (100–102) × 40 s per scenario. The first 10 s of each sequence are excluded while the filter converges.
- **Strides.** 1 frame (33 ms) and 5 frames (167 ms).
- **Metrics:**
  - RMSE, median and p95 of the Euclidean error in pixels, over valid frames;
  - the share of *all* frames with error ≤1 px (invalid frames count as misses);
  - 95 % coverage: the share of frames whose error falls inside the 95 % ellipse of the stated covariance (the ideal is 0.95);
  - the median σ;
  - a "zero" baseline that assumes no camera motion.

**Scenarios:**

| Scenario | Description |
|---|---|
| hover | wobble up to about 0.35 m/s² |
| cruise | 5 m/s, random heading |
| gust | 3 gusts up to 4 m/s² |
| vibration | 70 Hz, 0.15 rad/s and 2.5 m/s² in the IMU |
| noise ×10 | 10× gyro and accelerometer noise; the estimator is told about the higher noise |
| drift | gyro bias 0.05 rad/s, 5× the default; the estimator is *not* told |
| dropouts | 3 % chance per sample of losing 2–10 IMU samples |
| no FC velocity | cruise without the FC velocity input |

The exact sensor streams of the first seven scenarios are published as the **byeol-synth-v1** dataset.

## 5. Results

All results are synthetic. Reproduction commands are in §8.

### 5.1 E-001 — Physics vs. ground truth

**Stride 1 (33 ms)**

| Scenario | RMSE px | median | p95 | ≤1 px | zero RMSE | zero ≤1 px | 95 % coverage | median σ |
|---|---|---|---|---|---|---|---|---|
| hover | 0.08 | 0.06 | 0.13 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |
| cruise 5 m/s | 0.08 | 0.06 | 0.15 | 1.000 | 2.93 | 0.098 | 1.00 | 0.31 |
| gust | 0.14 | 0.09 | 0.28 | 1.000 | 4.35 | 0.246 | 1.00 | 0.30 |
| vibration | 0.15 | 0.13 | 0.24 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |
| noise ×10 | 0.40 | 0.33 | 0.69 | 0.998 | 5.03 | 0.052 | 0.99 | 0.40 |
| drift | 0.32 | 0.20 | 0.73 | 1.000 | 2.93 | 0.098 | 0.94 | 0.31 |
| dropouts 3 % | 0.77 | 0.25 | 1.48 | 0.881 | 5.03 | 0.052 | 0.97 | 0.49 |
| cruise, no FC velocity (default) | — | — | — | 0.000 | 2.93 | 0.098 | — | — |
| cruise, no FC velocity (research mode¹) | 2.46 | 2.16 | 3.31 | 0.000 | 2.93 | 0.098 | 0.98 | 1.61 |
| hover, no FC velocity, prior v≈0 (research¹) | 0.10 | 0.09 | 0.18 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |

**Stride 5 (167 ms)**

| Scenario | RMSE px | median | p95 | ≤1 px | zero RMSE | zero ≤1 px | 95 % coverage | median σ |
|---|---|---|---|---|---|---|---|---|
| hover | 0.35 | 0.27 | 0.60 | 1.000 | 8.30 | 0.013 | 1.00 | 0.85 |
| cruise 5 m/s | 0.34 | 0.24 | 0.71 | 0.996 | 14.54 | 0.004 | 1.00 | 0.90 |
| gust | 0.67 | 0.44 | 1.37 | 0.859 | 19.82 | 0.018 | 1.00 | 0.86 |
| vibration | 0.43 | 0.33 | 0.79 | 0.997 | 8.30 | 0.013 | 1.00 | 0.85 |
| noise ×10 | 1.08 | 0.88 | 1.89 | 0.592 | 23.57 | 0.002 | 0.99 | 1.07 |
| drift | 1.58 | 0.90 | 3.66 | 0.549 | 14.54 | 0.004 | **0.83** | 0.90 |
| dropouts 3 % | 2.53 | 1.40 | 4.75 | 0.375 | 23.57 | 0.002 | 0.96 | 3.01 |
| cruise, no FC velocity (research¹) | 12.38 | 10.88 | 16.68 | 0.000 | 14.54 | 0.004 | 0.96 | 7.88 |
| hover, no FC velocity, prior v≈0 (research¹) | 0.49 | 0.42 | 0.85 | 0.995 | 8.30 | 0.013 | 1.00 | 0.87 |

¹ `require_velocity=False`. This setting is not available in normal use. It is shown only to explain why FC velocity is mandatory.

Findings:

1. **Accuracy.** With FC velocity, stride-1 RMSE is 0.08–0.40 px, and 99.8–100 % of frames are within 1 px in every regime except IMU dropouts (88 %). The stride-5 results are the more informative ones, because the zero baseline is already fairly good at stride 1 during hover.
2. **FC velocity is essential.** Without it, in forward flight, physics (2.46 px) is barely better than assuming no motion (2.93 px). The default therefore refuses to produce an output.
3. **Covariance is conservative.** Coverage is 0.94–1.00 at stride 1 and 0.96–1.00 at stride 5, with one exception: a large unmodelled gyro bias at stride 5 gives 0.83. In that case the stated σ is too small.

### 5.2 E-002 — Physics vs. image phase correlation on synthetic water

Synthetic water textures of decreasing contrast (8-bit, sensor noise 1.5 levels) are used, with OpenCV `phaseCorrelate` and a Hann window as the image-based baseline.

| Texture contrast | physics RMSE / ≤1 px | phase corr. RMSE / ≤1 px | fused RMSE / ≤1 px |
|---|---|---|---|
| 25 (strong) | 0.19 / 100 % | 0.34 / 97.5 % | 0.27 / 99.2 % |
| 1.5 | 0.19 / 100 % | 0.71 / 86.5 % | 0.33 / 99.0 % |
| 0.5 | 0.19 / 100 % | 1.31 / 44.4 % | 0.29 / 99.4 % |
| 0.3 | 0.19 / 100 % | 8.19 / 24.2 % | 0.23 / 100 % |

At stride 5, phase correlation produced catastrophic outliers even on strong texture: RMSE 28 px with a median of 0.81 px. We did not investigate the cause. Physics had no such outliers (0.83 px). Fusion gives a small gain only where the image is good. Its parameters were fitted on the same renderer, so the fused numbers are optimistic.

### 5.3 E-003 — Compute cost

On a development x86 machine, `push_imu` takes 111 µs per sample and `frame_shift` takes 482 µs per query. That is **3.7 % of one core** at 200 Hz IMU and 30 fps. RSS is 93 MB, mostly the interpreter and numpy.

Applying an 8–15× slowdown factor estimates 29–55 % of one of the four Raspberry Pi Zero 2 W cores. **This has not been measured on hardware.** Planned reductions if needed: 100 Hz IMU, covariance every third frame, and an analytic Jacobian.

### 5.4 E-004 — Phase-correlation error model

Data: 1335 synthetic frames at three contrasts. Median error by phase-correlation response:

| `resp` | median error |
|---|---|
| 0.1–0.2 | 2.19 px |
| 0.2–0.4 | 1.27 px |
| 0.4–0.7 | 0.58 px |

The product of median error and `resp` is about 0.31–0.37, which gives `σ_v ≈ 0.3/resp`. This relationship may differ on real water.

### 5.5 E-005 — Sensitivity to camera parameters

Ground truth uses the true camera, and the estimator gets a perturbed model. Modes: cruise and gust, 2 seeds × 30 s.

| Variant | Stride 1 RMSE px (mrad) | Stride 5 RMSE px (mrad) | Stride 5 ≤1 px |
|---|---|---|---|
| exact f | 0.108 (0.24) | 0.529 (1.16) | 94.5 % |
| f ±5 % | 0.185–0.191 | 0.899–0.929 | 78 % |
| f ±10 % | 0.323–0.330 | 1.564–1.599 | 38–39 % |
| 1920×1080, f = 1371 | 0.325 (0.24) | 1.587 (1.16) | 60 % |
| thermal 640×512, f = 1116 | 0.265 (0.24) | 1.292 (1.16) | 70 % |
| thermal 160×120, f = 172 | 0.041 (0.24) | 0.199 (1.16) | 100 % |
| distortion k1 = −0.12, modelled, edge pixel | 0.193 | 0.938 | 81 % |
| distortion k1 = −0.12, **ignored**, edge pixel | 0.707 | 3.670 | 16 % |

Conclusions:

- An error in f scales the shift error proportionally, so calibrate f to better than ±5 % for large strides.
- In milliradians the error is the same at every resolution, so cameras should be compared in mrad.
- Wide-angle lenses with off-centre reference pixels need the distortion model.
- The principal-point test (+10, −6 px) showed no effect only because the reference pixel was the centre. That is a weak experiment, not evidence of insensitivity.

### 5.6 E-006 — End-to-end: tracker with image ego-motion vs. byeol ego-motion

This experiment uses the classical tracker of the sibling project **dal**, which is **not part of this release**. The results are published for transparency and cannot be reproduced from this repository alone. Full tables: [`experiments/eval_dal_bridge.md`](../experiments/eval_dal_bridge.md).

Setup:

- Scene: synthetic water with waves and glint, and a person-sized dark blob (about 17 px, amplitude 2 levels against sensor noise 1.5).
- Tracker: initialised on the true box.
- Data: 3 seeds and 19 scenarios.
- Metric: F1 by box-centre hit.

Modes:

| Mode | Ego-motion source |
|---|---|
| `image` | dal as is |
| `image_g10` | dal as is, with a stricter confidence gate of 0.1 |
| `byeol` | byeol, falling back to image when byeol is invalid |
| `fused` | the fusion of §2.7 |
| `zero` | no ego-motion |
| `oracle` | the true shift |

Results:

- **byeol helps when the image cannot see the water.** At contrast 0.2 and 10 fps, F1 is 0.24 ± 0.20 for `image` and 0.98 ± 0.02 for `byeol`, which equals `oracle`. In hover, gust, vibration and noise ×10 flights at c = 0.2, `image` scores 0.10–0.27 and `byeol` scores 0.97–1.00.
- **At contrast 0.5, most of the gain comes from a simple gate.** `image` scores 0.65–0.69, `image_g10` 0.97–1.00 and `byeol` 0.98–1.00. Only in gusts does physics clearly win over the gated image: 1.00 against 0.87 ± 0.18.
- **byeol does not help** at:
  - contrast ≥ 0.8, where the target itself is the limit and `oracle` ≈ `image`;
  - 30 fps, where `zero` scores 0.99 and ego-motion is not needed at all.
- **byeol is worse with 3 % IMU dropouts.** F1 is 0.77 ± 0.27 at c = 0.5 and 0.15 at c = 0.2. byeol correctly reports low confidence (σ 4–14 px). The bridge then falls back to a poor image estimate, and the tracker's Kalman filter cannot use an ego-motion σ.
- **Without FC velocity** byeol refuses, and the tracker behaves exactly like `image`. The fallback is safe but gains nothing.

The results suggest three changes to the tracker (D-021): a confidence gate of about 0.1, a pluggable ego-motion function, and passing the ego-motion σ into the tracker's Kalman filter.

## 6. Limitations

1. **Synthetic only.** No real flights, IMU logs or video have been used. The noise models, wind, vibration spectra and water rendering are our own assumptions. The generator and the estimator share geometric assumptions, so the results are an **upper bound**.
2. **Flat surface.** Waves, swell, terrain relief, shorelines, vegetation and objects above the plane are not modelled. Over water, byeol estimates camera motion relative to a world-fixed plane, not relative to drifting water.
3. **Not modelled:** rolling shutter, gimbal dynamics (only the commanded angle θ is used), IMU–camera lever arm, fisheye lenses (inverse distortion converges only for moderate k1), and camera–IMU time offsets beyond the assumed 3 ms σ. A 5 ms sync error at 0.5 rad/s gives about 1.1 px.
4. **Uncertainty is miscalibrated for large unknown gyro bias** at large strides (coverage 0.83).
5. **Assumptions in defaults.** Height σ per source, noise densities and fusion coefficients are not measured on hardware.
6. **Evaluation size.** Each scenario has 3 seeds × 25–40 s. No confidence intervals were computed. In E-006, differences smaller than the seed standard deviation should not be treated as meaningful.
7. **Compute on the target** is extrapolated, not measured.
8. **Time systems.** The leap-second table ends at 2017-01-01 and must be updated if IERS announces new leap seconds. GPS week rollover is not handled.

## 7. Future work

1. **Real flights** recorded under a public protocol ([RECORDING_PROTOCOL.md](RECORDING_PROTOCOL.md)). They would allow calibrating the noise parameters, the height σ and the camera–IMU offset, and running the end-to-end evaluation on real water.
2. PX4 SITL with Gazebo as an intermediate, more realistic simulator.
3. Measurement on a Raspberry Pi Zero 2 W. If needed: an analytic Jacobian, 100 Hz IMU, or an EKF in place of Mahony.
4. An interactive configuration utility for different flight controllers (MAVLink messages, rates, frames).
5. Possibly, a small learned correction for wind and vibration, outside any control loop. This would only come after real logs exist (D-001).

## 8. Reproducibility

| ID | What | Command (from repository root) | Output |
|---|---|---|---|
| — | Tests | `python -m pytest tests -q` | — |
| E-001/E-002 | Accuracy and phase-correlation baseline | `python scripts/eval_ego.py` (`--quick` for a smoke run; needs OpenCV) | `experiments/eval_ego.json`, `.md` |
| E-001 | Same, from published dataset files | `python scripts/eval_dataset.py` | stdout table |
| — | Rebuild / verify the dataset | `python scripts/make_dataset.py [--check]` | `data/byeol-synth-v1/` |
| E-003 | CPU cost | `python scripts/bench_budget.py` | stdout |
| E-004 | Vision σ model | `python scripts/fit_vision_sigma.py` | stdout |
| E-005 | Camera sensitivity | `python scripts/eval_sensitivity.py` (overwrites `experiments/eval_sensitivity.json`) | `experiments/eval_sensitivity.json`, `.md` |
| E-006 | Tracker end-to-end | requires the unreleased `dal` tracker | `experiments/eval_dal_bridge.json`, `.md` |

The reference environment is Python 3.13, numpy 2.4, OpenCV 5.0, SciPy 1.15, on x86-64 Linux. The dataset is bit-reproducible on that platform. Different numpy builds or CPUs may differ in the last floating-point bits. Use `make_dataset.py --check` to verify.

## References

- R. Mahony, T. Hamel, J.-M. Pflimlin. *Nonlinear complementary filters on the special orthogonal group.* IEEE Transactions on Automatic Control, 53(5), 2008. The byeol filter is inspired by this paper; the code is original.
- H. C. Longuet-Higgins, K. Prazdny. *The interpretation of a moving retinal image.* Proc. R. Soc. Lond. B, 1980. This is the linearised flow used as a test oracle.
- R. Hartley, A. Zisserman. *Multiple View Geometry in Computer Vision*, 2nd ed., 2004. Plane-induced homography.
- D. C. Brown. *Decentering distortion of lenses.* Photogrammetric Engineering, 1966. The distortion model, as in OpenCV.
