# byeol Flight-Recording Protocol — Call for Contributions

**English** · [Українська](RECORDING_PROTOCOL.uk.md)

> **Status: Draft v0.1 — not yet validated on hardware; verify MAVLink/uORB message names against your flight-controller docs.** We will revise this protocol after the first real recordings. Feedback on anything that is impractical on your platform is very welcome.

## Why we are asking

Every result for byeol so far is synthetic (see [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md), §6). The single most valuable contribution to this project is **real flight data**: IMU, flight-controller (FC) state and video, recorded together on one clock, with a calibrated camera.

Such recordings would let us:

- compare byeol's `(dx, dy)` against optical ground truth from the video and against image-based estimates;
- calibrate the noise parameters, the per-source height σ and the camera–IMU time offset, which are currently assumptions;
- find the conditions in which byeol fails, and publish them.

We are looking for recordings from volunteer drone teams, search-and-rescue units and researchers. You do not need to run byeol on board. byeol never commands the aircraft, and recording for it requires no change to your flight stack beyond logging. This document describes what to record, how to synchronise and calibrate, which file layout to use, and how to license and share the result.

Please read [ETHICS.md](../ETHICS.md) and §8 below before you record anything.

## 1. Signals and rates

All units SI. Body frame FRD, world frame NED (see [INTEGRATION.md](INTEGRATION.md) for conventions).

| Signal | Minimum | Recommended | Notes |
|---|---|---|---|
| Gyroscope (body FRD, rad/s) | 200 Hz | 400–1000 Hz raw (before decimation) | Store raw and FC-filtered streams separately; record the FC filter settings |
| Accelerometer (specific force, m/s²) | 200 Hz | same as gyro | |
| FC EKF velocity (NED, m/s) + its covariance or quality flag | 5 Hz | 10–30 Hz | Mandatory byeol input ([TECHNICAL_REPORT.md](TECHNICAL_REPORT.md), §2.3) |
| FC EKF attitude (quaternion) and heading | 20 Hz | 50 Hz | For comparison with byeol's attitude filter and for checking axis conventions |
| Height: FC fused altitude | 5 Hz | 10–30 Hz | Plus barometer and rangefinder (with its quality indicator) as separate streams, if present |
| Gimbal angle (if gimballed) | 10 Hz | 30 Hz + the commanded angle | For a fixed mount, measure and record the mounting angle |
| GPS position/velocity, GPS time | 5 Hz | 10 Hz | Used only as velocity reference and for time alignment |
| Frame trigger (exposure time) | every frame | every frame | See §2 |
| Video | 25–30 fps | constant fps, no automatic changes, no in-camera stabilisation | **Turn off electronic image stabilisation (EIS).** It breaks the camera geometry |

Also record, if available: IMU temperature, battery voltage, motor RPM or PWM (for vibration analysis) and flight mode.

**Where to find these signals.** The names below are starting points only. They differ between firmware versions, so check them against the documentation of the exact firmware you fly.

- **PX4 (ULog, uORB topics):** raw IMU in `sensor_gyro` / `sensor_accel` (or `sensor_combined`), velocity in `vehicle_local_position`, attitude in `vehicle_attitude`, rangefinder in `distance_sensor`, gimbal in `gimbal_device_attitude_status`, trigger events in `camera_capture` / `camera_trigger`. Enable high-rate IMU logging in the logger profile.
- **ArduPilot (DataFlash):** `IMU`, `ATT`, EKF velocity in `XKF1`, `BARO`, `RFND`, camera events in `CAM` / `TRIG`. Enable raw IMU logging where your version supports it.
- **MAVLink telemetry (tlog):** `HIGHRES_IMU` or `SCALED_IMU`, `LOCAL_POSITION_NED`, `ATTITUDE_QUATERNION`, `DISTANCE_SENSOR`, `GIMBAL_DEVICE_ATTITUDE_STATUS`, `CAMERA_TRIGGER`. Telemetry links usually cannot carry IMU at 200 Hz; prefer the on-board log.

## 2. Frame-to-IMU time synchronisation

This is the most critical part. A synchronisation error `t_sync` produces a pixel error of approximately

```
error ≈ f · |ω| · t_sync
```

For example, f = 457 px, ω = 0.5 rad/s and t_sync = 5 ms give about 1.1 px, which is larger than byeol's entire error budget in calm flight.

1. **Hardware trigger (preferred).** The FC generates the camera trigger pulse and logs its timestamp on its own clock. The camera stores each frame with that same timestamp.
2. **Without a hardware trigger.** Record the frame arrival time on the companion computer (monotonic clock) and measure the capture latency. In addition, perform a **calibration event** at the beginning and at the end of every flight: rotate the drone or the gimbal sharply about one axis for about 1 s. The cross-correlation between gyro angular rate and optical flow then gives the camera–IMU offset and clock drift. byeol's `byeol.timebase.ClockMap` fits `t_other = scale · t_this + offset` from such event pairs, and `EgoConfig.cam_imu_offset` takes the result.
3. **One clock for everything.** Use the FC boot clock or the companion-computer clock. Map to GPS time or UTC only for georeferencing, via `byeol.timebase`. Record both clocks and the moments at which they were aligned.
4. **Report** in the metadata: the estimated `cam_imu_offset`, `ClockMap.scale` and the residual (ms).

## 3. Camera calibration

- **Before the flight**, calibrate with a chessboard using `scripts/calibrate_camera.py`. Use 15–25 frames at the same resolution, the same (fixed!) focus and the same camera mode as in flight. The reprojection RMS should be below 0.5–1 px. Example:

  ```bash
  python scripts/calibrate_camera.py --images "calib/*.jpg" --cols 9 --rows 6 --square-mm 30 --kind rgb \
      --name my_cam --out configs/cameras/my_cam.yaml
  ```

  `--cols` and `--rows` count the *inner* corners. Include the profile with your recording and note the date and ambient temperature. For a different resolution use `CameraModel.resized()`.
- **Thermal cameras** need a board with contrast in the long-wave infrared (heated, or built from materials with different emissivity). If calibration is impossible, set the HFOV from the manufacturer's documentation and mark the profile as "not calibrated" in the metadata.
- **Field checks.** Check the camera model against a distant horizontal line or an object of known size. Check height by flying over an area of known dimensions (for example, road markings) and comparing the scale.
- Record focus settings and temperature changes that may affect f. **Do not change the zoom during a recording.**

The technical report (§5.5) shows why this matters: an error of ±5 % in f produces a proportional error in the shift.

## 4. Log format

One directory per recording, named `YYYYMMDD-HHMM-<id>/`:

```
YYYYMMDD-HHMM-<id>/
├── meta.yaml
├── imu.csv
├── fc_state.csv
├── frames.csv
├── events.csv
├── video.mp4            (or frames/000000.png, ...)
├── camera.yaml          (camera profile from §3)
├── LICENSE              (data licence, see §9)
└── raw/                 (original FC log, unmodified)
```

- **`meta.yaml`**: protocol version (`0.1`), date, **coarse** location (region or country only, see §8), scene type (water / textured ground / forest / mixed), camera (reference to the profile), gimbal or fixed mount and angle, flight heights, FC firmware version and its filter parameters, hardware, `cam_imu_offset` and `ClockMap` results, data licence, contributor name or team (as you wish to be credited), and SHA-256 hashes of all files.
- **`imu.csv`**: `t_s, gx, gy, gz, ax, ay, az, temp_c` (SI, FRD; `t_s` on the single clock).
- **`fc_state.csv`**: `t_s, vn, ve, vd, v_var, qw, qx, qy, qz, yaw, h_fused, h_baro, h_range, h_range_quality, gimbal_pitch, mode`. Leave a field empty where the signal is not available.
- **`frames.csv`**: `frame_idx, t_exposure_s, t_received_s, file`. The video is `video.mp4` or a PNG sequence, at constant fps.
- **`events.csv`**: `t_s, event` (start and end of calibration motions, take-off, landing, gusts and other notable events).
- **`raw/`**: the original FC log (ULog, DataFlash or MAVLink tlog), unmodified.

All times are in seconds (float64). All quantities are SI, NED/FRD. Convert source units (feet, knots, integer MAVLink fields) before writing, using `byeol.units`, or state the units clearly in `meta.yaml`.

## 5. Flight scenarios

A minimal set:

1. Hover for 60 s at 10, 30 and 60 m.
2. Straight-line flight at 3, 5 and 10 m/s at the same heights.
3. Yaw turns, and roll/pitch rocking.
4. Gusts and jerks (in wind).
5. Different gimbal angles (45°, 60°, 90°).
6. Scenes: water (low texture), textured ground, forest.
7. If you run an image-based tracker, also record a run with image ego-motion disabled, as a separate run, so that the comparison is fair.

Partial contributions are still useful. A single hover and a single straight-line flight over water, properly synchronised and calibrated, would already be the first real test of byeol.

## 6. Ground truth

- **Over textured ground:** optical flow over highly textured, high-contrast areas; ground markings; GPS-RTK if available.
- **Over water** there is no reliable optical ground truth. There we compare against the flight velocity and check internal consistency, and we say so in any published results.

## 7. Processing

The planned pipeline is: a log reader for the format of §4 → `EgoEstimator`, fed in time order (as `byeol.replay` does for synthetic data) → tables like those of `scripts/eval_ego.py`. A reader for real logs is not yet part of the release; it will be written against the first contributed recordings.

When results are published, the weak spots will be published too, in particular all conditions in which byeol returns `valid=False` and all conditions in which it is worse than the alternatives.

## 8. Privacy and ethics

byeol exists to help find people in danger. The data used to test it must not harm anyone. In addition to [ETHICS.md](../ETHICS.md):

- **Record only where there are no bystanders**, or with the informed consent of everyone who may appear in the frame.
- **Never record people for identification.** If a person is needed in the scene (for example, a volunteer playing a person in the water), they must have agreed to it, and the recording must not be used to identify anyone.
- **No biometrics.** Do not collect or annotate faces, gait or other biometric features.
- **Coarse location only** in metadata: region or country, not coordinates of private property or of sensitive sites.
- If the location is sensitive, **strip EXIF and GPS tags from published video and images**, and remove or coarsen the GPS fields in the published logs. The raw FC log contains GPS positions; if that is a concern, tell us and we will agree how to publish a reduced version.
- Fly legally: follow local aviation regulations and airspace restrictions, and obtain any permits you need. Do not fly over people or in active emergency operations only to collect data.
- Do not contribute recordings from military or law-enforcement surveillance operations.

## 9. Licensing and sharing

**Contributed data will be redistributed openly**, as part of a public dataset alongside this repository (and possibly mirrored on dataset hosting such as Hugging Face). Contribute only data you have the right to share, and only if you are comfortable with it being public and permanent.

- **Recommended licences:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) (attribution required) or [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/) (public domain dedication). Put the licence text or a link in a `LICENSE` file inside the recording directory and state it in `meta.yaml`.
- **Credit.** We will credit contributors as written in `meta.yaml`. You may also contribute anonymously.
- **How to submit.** Open an issue describing the recording (platform, scene, duration, size, licence). We will agree on how to transfer the files. See [CONTRIBUTING.md](../CONTRIBUTING.md).
- **Withdrawal.** If you later find a privacy problem in data you contributed, tell us and we will remove it from future releases. Copies already downloaded by others cannot be recalled.

## Checklist

- [ ] EIS off, fixed focus, constant fps, no zoom changes
- [ ] Camera calibrated (`camera.yaml`), date and temperature recorded
- [ ] Raw IMU ≥ 200 Hz and FC velocity ≥ 5 Hz logged on board
- [ ] Hardware trigger, or calibration motions at start and end of each flight
- [ ] Directory layout and CSV columns as in §4; units SI, NED/FRD
- [ ] No bystanders, or consent from everyone in frame; coarse location; EXIF/GPS stripped if sensitive
- [ ] Licence chosen (CC BY 4.0 or CC0) and stated in `LICENSE` and `meta.yaml`
