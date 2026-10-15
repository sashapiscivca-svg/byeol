# Integration guide

**English** · [Українська](INTEGRATION.uk.md)

This guide covers what you need to connect byeol to a real drone or to recorded logs: the inputs, coordinate frames, units, the output contract and failure modes. For the method and its validation, see [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md).

> byeol only **reads** sensor data. Never feed its output back into flight control or safety logic.

## 1. Installation

```bash
pip install -e .              # numpy only
pip install -e ".[yaml]"      # + YAML camera profiles
pip install -e ".[eval]"      # + OpenCV/SciPy for evaluation, calibration and the phase-correlation baseline
```

Python ≥ 3.10. The core imports only numpy, so it runs on a Raspberry Pi with the system numpy.

## 2. Inputs

All inputs share **one clock** in seconds, usually the flight controller's boot time. If the camera uses another clock, map it with `byeol.timebase.ClockMap` or set `EgoConfig.cam_imu_offset`, defined as `t_imu = t_camera + offset`.

| Call | Units / frame | Rate | Required | Notes |
|---|---|---|---|---|
| `push_imu(t, gyro, accel)` | rad/s and m/s² (specific force), body **FRD** | 100–400 Hz | yes | Samples with NaN/Inf, `\|ω\| > 35 rad/s` or non-increasing time are dropped (`est.stats["dropped"]`). |
| `push_velocity(t, v_ned, sigma)` | m/s, world **NED**; σ in m/s | 5–10 Hz (≥1 Hz) | **yes** | FC EKF velocity, e.g. MAVLink `LOCAL_POSITION_NED`. If older than `vel_max_age` (1 s), the result is `stale_velocity`. |
| `push_height(t, h, source)` | m above the **imaged surface** | 5–10 Hz | yes | `source` ∈ `HeightSource`. Samples from sources other than the primary/fallback are ignored and counted. |
| `push_gimbal(t, theta)` | rad; 0 = forward horizontal, π/2 = nadir | per frame or on change | yes | A fixed camera still needs one call with its mount angle. |
| `push_heading(t, yaw)` | rad, NED (clockwise from north) | 2–5 Hz | recommended | Aligns the filter's world yaw with the NED frame of the FC velocity. |

### Height source (`HeightSource`)

| Value | Use | Default absolute σ (assumption) |
|---|---|---|
| `FUSED_FC` | FC fused altitude. Primary over water, where rangefinders give spurious returns. | 0.4 m |
| `RANGEFINDER` | Lidar or sonar over land, when available | 0.1 m |
| `BARO` | Barometer. Must be corrected to the surface level, because it measures height above the take-off point. | 1.0 m |
| `MANUAL` | Constant height entered by the operator | 2.0 m |

Configure the height inputs with these `EgoConfig` fields:

- `height_source`: the primary source.
- `height_fallback`: the backup, e.g. `RANGEFINDER` primary with `FUSED_FC` fallback.
- `height_trust` ∈ (0, 1]: the σ used is `σ_abs / trust`.
- `height_max_age`: how old a height sample may be.

The default σ values are **not measured**. Calibrate them for your hardware.

### Converters for foreign formats

The estimator accepts only SI, NED/FRD and one clock. Convert at the boundary, in the code that reads the external format:

- `byeol.units`: `feet_to_m`, `knots_to_ms`, `kmh_to_ms`, `mph_to_ms`, `fpm_to_ms`, `cms_to_ms` (MAVLink cm/s), `cdeg_to_rad` (MAVLink centidegrees), `mm_to_m`, and `to_si(x, unit)`.
- `byeol.frames`: `ned_to_enu` / `enu_to_ned`, `frd_to_flu` / `flu_to_frd`, `yaw_ned_to_enu` / `yaw_enu_to_ned`, and rotation matrices between the NED/FRD and ENU/FLU conventions (ROS REP-103).
- `byeol.timebase`: UTC/Unix ↔ GPS seconds with leap seconds, GPS week + TOW, and `ClockMap.fit(t_this, t_other)` for a linear clock mapping estimated from paired events.

## 3. Camera

```python
from byeol.cameramodel import CameraModel, Distortion
cam = CameraModel.from_pixels(1280, 720, fx=915.0, fy=915.0, cx=639.5, cy=359.5,
                              dist=Distortion(k1=-0.12))
cam = CameraModel.from_mm(640, 512, focal_mm=13.0, sensor_w_mm=7.68, sensor_h_mm=6.14, kind="lwir")
cam = CameraModel.from_fov(640, 360, hfov_deg=70)

from byeol.camprofile import load_camera, dump_camera
cam = load_camera("configs/cameras/generic_lwir_320x256_hfov45.yaml")   # .json needs no extra dependency
```

- **Pixel coordinates are in the frame for which the model is defined.** If your tracker works on a down-scaled frame, pass `cam.resized(w, h)`, or `cam.binned(b)` / `cam.cropped(x0, y0, w, h)`. Focal length in pixels scales and the FOV is preserved.
- Calibrate with a chessboard: `python scripts/calibrate_camera.py --help` (needs OpenCV). It writes a YAML/JSON profile. For thermal cameras, use a board with contrast in the long-wave infrared.
- If your frames are already undistorted, set zero distortion.
- The profiles in `configs/cameras/` are **illustrative templates**, not vendor data.

## 4. Output contract

```python
r = est.frame_shift(t0, t1)    # t0 < t1, both camera timestamps
```

| Field | Meaning |
|---|---|
| `dx`, `dy` | Pixel displacement of image content from frame t0 to frame t1 (+dx right, +dy down), at the reference pixel. By default this is the image centre. With `EgoConfig.pixels=(N,2)` it is the mean over the given pixels. |
| `cov` | 2×2 covariance, px² |
| `sigma` | `sqrt(trace(cov)/2)`, px |
| `conf` | `1 / (1 + λmax(cov))` ∈ (0, 1]. **Not comparable** to phase-correlation `resp`. |
| `valid` | `False` → `dx = dy = NaN`, `cov = 1e6·I`, `conf = 0` |
| `reason` | `ok` or a refusal code (below) |
| `height_source` | which height source was used |
| `as_tuple()` | `(dx, dy, conf)`, the same shape as `dal.ego.estimate_shift` |

To use the shift to predict where a point moves: `u₁ ≈ u₀ + (dx, dy)` near the reference pixel. For points far from the centre with a tilted camera, pass the points of interest in `pixels`.

### Refusal codes

| `reason` | Cause | Typical fix |
|---|---|---|
| `no_imu` | Fewer than 3 IMU samples, t0 before the history, t1 more than 50 ms past the last IMU sample, or t1 ≤ t0 | Check the IMU stream and the clock alignment |
| `no_height` | No sample from the primary or fallback source within `height_max_age` | Check the `HeightSource` configuration |
| `low_height` | h < 1 m | Expected on the ground or at take-off |
| `no_gimbal` | `push_gimbal` was never called | Call it at least once, even for a fixed mount |
| `no_velocity` / `stale_velocity` | FC velocity absent, or older than 1 s | Stream FC velocity at 5–10 Hz |
| `horizon` | The reference ray does not hit the surface plane (camera above the horizon) | Tilt the gimbal down or use other reference pixels |

The fusion helper adds three more codes: `fused`, `vision_only` and `vision_gate`.

## 5. Fusion with an image-based estimate (optional)

```python
from byeol.fuse import fuse
r2 = fuse(r, (dx_v, dy_v, resp))   # phase correlation result in the same pixel scale
```

Inverse-variance fusion with `σ_v = max(0.3/resp, 0.2)` px. Vision with `resp < 0.1` is ignored. If the two estimates are inconsistent at the 99 % level, the one with the smaller σ is kept. If physics is invalid, vision is used alone. The coefficients were fitted on synthetic water, so **recalibrate them on your footage**.

## 6. Recommended consumer behaviour

1. Use `valid` and `conf`, and never ignore `cov`. In synthetic end-to-end tests, a tracker that accepted low-confidence ego-motion as exact drifted away. A confidence gate of about 0.1 and passing σ into the tracker's Kalman filter avoid this ([TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) §5.6).
2. Treat `valid=False` as "no information". Fall back to your image estimate only if it has its own sound confidence gate, and otherwise assume zero motion.
3. At high frame rates (≥30 fps), ego-motion compensation may be unnecessary. Measure before relying on it.

## 7. Tuning parameters (`EgoConfig`)

| Group | Parameters (defaults) |
|---|---|
| Noise | `gyro_density` 3e-4 rad/s/√Hz, `bias_sigma` 0.003, `bias_sigma0` 0.015 rad/s, `bias_tau` 12 s, `scale_sigma` 0.005, `time_sigma` 0.003 s, `gap_rate_sigma` 0.3 rad/s, `tilt_sigma` 0.03 rad, `height_rel_sigma` 0.03, `vel_model_sigma` 0.05 m/s, `floor_px` 0.05 |
| Robustness | `max_gap` 0.03 s, `max_extrap` 0.05 s, `max_rate` 35 rad/s, `history_s` 3 s, `min_height` 1 m, `height_tau` 1 s, `vel_max_age` 1 s, `height_max_age` 2 s |
| Filters | `mahony={kp, ki, kp_yaw, ki_yaw, gate, accel_tau}`, `kf={accel_noise, bias_walk, p_v0, p_b0}` passed to `VelocityKF` |
| Research only | `require_velocity=False`, `zero_vel_sigma` |

Estimate `gyro_density` and the accelerometer noise from a static recording, or take them from the IMU datasheet. Measure `cam_imu_offset` with a rotation calibration manoeuvre, as described in [RECORDING_PROTOCOL.md](RECORDING_PROTOCOL.md).

## 8. Replaying logs

`scripts/eval_dataset.py` is the reference loop. It merges all streams by timestamp and pushes them in order. Height is pushed before velocity at equal time, and the gimbal is pushed just before each frame query. The same loop works for real logs recorded in the format described in [RECORDING_PROTOCOL.md](RECORDING_PROTOCOL.md).
