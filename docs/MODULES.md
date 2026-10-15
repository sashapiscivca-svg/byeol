# Module map · Карта модулів

The core (`src/byeol`) depends only on numpy. OpenCV and SciPy are needed only for the image baseline, the water scene and calibration. Every module and script must be listed here; `tests/test_docs_map.py` checks this.

Ядро (`src/byeol`) залежить лише від numpy. OpenCV і SciPy потрібні тільки для бази «з картинки», сцени води й калібрування. Кожен модуль і скрипт має бути в цьому списку; це перевіряє `tests/test_docs_map.py`.

## Package `src/byeol` · Пакет

| Module | Purpose (EN) | Призначення (UK) |
|---|---|---|
| `estimator.py` | `EgoEstimator`, `EgoConfig`, `EgoResult`, `HeightSource`: streaming IMU/height/gimbal/velocity/heading → `frame_shift(t0, t1)` with covariance | Потоковий оцінювач: IMU/висота/підвіс/швидкість/курс → `frame_shift(t0, t1)` з коваріацією |
| `orientation.py` | `Mahony` complementary filter: tilt, gyro bias, optional heading | Комплементарний фільтр `Mahony`: нахил, зсув нуля гіро, опційний курс |
| `velocity.py` | `VelocityKF`: world velocity and accelerometer bias, FC velocity updates | `VelocityKF`: швидкість у світі й зсув нуля акселерометра, оновлення швидкістю від ПК |
| `camera.py` | Simple `Pinhole`, `gimbal_R_bc`, `ground_plane_in_camera`, `ego_shift` (exact plane homography) | Спрощений `Pinhole`, поворот підвісу, площина в камері, `ego_shift` (точна гомографія площини) |
| `cameramodel.py` | `CameraModel` (pixels / mm / FOV, resolution, crop, binning, Brown–Conrady distortion, rgb/lwir), `Distortion` | `CameraModel` (пікселі / мм / FOV, роздільність, кроп, біннінг, дисторсія, rgb/lwir) |
| `camprofile.py` | Load/save validated YAML/JSON camera profiles (`configs/cameras/`) | Завантаження/збереження профілів камер YAML/JSON із валідацією |
| `calibrate.py` | Chessboard calibration (OpenCV optional), synthetic chessboard for tests | Калібрування за шахівницею (OpenCV опційно), синтетична шахівниця для тестів |
| `rotation.py` | Quaternions, rotation vectors, Z-Y-X Euler angles, `wrap_pi` | Кватерніони, вектор повороту, кути Ейлера Z-Y-X |
| `units.py` | Unit converters to SI (feet, knots, km/h, mph, MAVLink integer units) | Конвертери одиниць у SI (фути, вузли, км/год, милі/год, цілі одиниці MAVLink) |
| `frames.py` | NED↔ENU, FRD↔FLU, heading and attitude conversions | Перетворення NED↔ENU, FRD↔FLU, курсу й орієнтації |
| `timebase.py` | UTC/Unix↔GPS seconds with leap seconds, GPS week + TOW, `ClockMap` | UTC/Unix↔GPS-секунди з високосними секундами, GPS-тиждень + TOW, `ClockMap` |
| `fuse.py` | Inverse-variance fusion with phase correlation, consistency gate | Злиття з фазовою кореляцією за оберненою дисперсією, брамка узгодженості |
| `synth.py` | Synthetic trajectory + noisy IMU + ground-truth pixel shift | Синтетична траєкторія + IMU із шумом + істинний піксельний зсув |
| `replay.py` | Replays a synthetic sequence through the estimator, errors vs. truth | Програвання синтетичної послідовності через оцінювач, похибки проти істини |
| `metrics.py` | RMSE, share ≤1 px, 95 % coverage, zero baseline | RMSE, частка ≤1 пікс, покриття 95 %, нульова база |
| `vision_baseline.py` | Synthetic water frames + OpenCV phase correlation (evaluation only) | Синтетичні кадри води + фазова кореляція OpenCV (лише для оцінки) |
| `water_scene.py` | Water + waves + glint + person-sized target with ground truth (evaluation only) | Вода + хвилі + відблиски + ціль розміру людини з істиною (лише для оцінки) |

## Scripts `scripts/` · Скрипти

| Script | Purpose (EN) | Призначення (UK) |
|---|---|---|
| `eval_ego.py` | Main evaluation E-001/E-002: scenarios, strides 1 and 5, phase-correlation baseline | Головна оцінка E-001/E-002: сценарії, кроки 1 і 5, база фазової кореляції |
| `eval_sensitivity.py` | E-005: sensitivity to focal length, resolution, distortion, principal point | E-005: чутливість до фокусної, роздільності, дисторсії, головної точки |
| `fit_vision_sigma.py` | E-004: phase-correlation error vs. response (basis of `fuse.py`) | E-004: похибка фазової кореляції проти resp (основа `fuse.py`) |
| `bench_budget.py` | E-003: CPU time and memory on the current machine | E-003: час CPU і пам'ять на поточній машині |
| `calibrate_camera.py` | Chessboard calibration CLI → YAML/JSON camera profile | CLI калібрування шахівницею → профіль камери YAML/JSON |
| `make_dataset.py` | Builds / verifies the byeol-synth-v1 dataset | Створює / перевіряє датасет byeol-synth-v1 |
| `eval_dataset.py` | Runs the estimator on dataset files; reference log-replay loop | Проганяє оцінювач на файлах датасету; еталонний цикл програвання логів |
| `stage_hf.py` | Prepares Hugging Face upload folders (model card and dataset) | Готує теки для завантаження на Hugging Face (картка моделі й датасет) |

## Other · Інше

- `configs/cameras/*.yaml` — parametric camera templates (RGB and LWIR), illustrative values · параметричні шаблони камер, ілюстративні значення.
- `experiments/` — saved results E-001…E-006 · збережені результати.
- `data/byeol-synth-v1/` — synthetic benchmark · синтетичний бенчмарк.
