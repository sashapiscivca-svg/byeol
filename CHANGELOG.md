# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses [Semantic Versioning](https://semver.org/).

## [0.1.0] — 2026-10-15

First public release (research preview). Validated on synthetic data only.

### Added

- `EgoEstimator`: physics-based inter-frame pixel shift with 2×2 covariance from IMU, flight-controller velocity (mandatory), height (configurable source), gimbal angle and optional heading; explicit refusal codes.
- Mahony attitude filter with gyro-bias estimation, velocity Kalman filter, exact plane-induced homography, linearised uncertainty propagation.
- Optional inverse-variance fusion with image phase correlation (`byeol.fuse`).
- Configurable `CameraModel` (pixels / mm / FOV, distortion, resize / bin / crop), 7 camera templates, chessboard calibration CLI.
- Unit, frame (NED/ENU, FRD/FLU) and time-system (UTC/GPS, leap seconds, clock mapping) converters.
- Synthetic generator, evaluation scripts and saved results for experiments E-001…E-006.
- **byeol-synth-v1** dataset: 21 synthetic sequences with raw sensor streams and ground truth, bit-reproducible.
- Bilingual documentation (English / Ukrainian): README, technical report, integration guide, design decisions, model card, dataset card, recording protocol, responsible-use statement.

### Not included

- The bridge to the `dal` tracker used for E-006 (depends on an unreleased project); its results are published in `experiments/`.
