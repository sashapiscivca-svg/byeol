---
license: mit
language:
  - en
  - uk
pretty_name: byeol-synth-v1
size_categories:
  - n<1K
tags:
  - drone
  - uav
  - imu
  - ego-motion
  - synthetic
  - search-and-rescue
  - benchmark
viewer: false
---

# byeol-synth-v1

**English** · [Українська](#українська)

A deterministic synthetic benchmark for estimating drone camera ego-motion from onboard sensors. It contains 21 sequences (7 flight scenarios × 3 seeds × 40 s) of raw sensor streams (IMU, flight-controller velocity and heading, height, gimbal angle, frame timestamps) with full ground truth, including the true inter-frame pixel shift of the image centre. It is the dataset behind experiment E-001 of the [byeol technical report](../../docs/TECHNICAL_REPORT.md).

> **Synthetic data only.** The sequences come from byeol's own simplified flight and sensor model. They contain no images, no people, no locations and no personal data. Good results on this benchmark are necessary but not sufficient: they do not show real-world performance.

## Contents

| File | Description |
|---|---|
| `manifest.json` | Global metadata (camera, rates, conventions), plus per-sequence id, scenario, seed, generator configuration, sample counts and SHA-256 hashes (file and array content) |
| `<scenario>_s<seed>.npz` | One sequence: compressed NumPy archive of float64 arrays (about 0.6–0.7 MB each, 15 MB in total) |

Scenarios (seeds 100, 101, 102):

| Scenario | Description |
|---|---|
| `hover` | Hover with smooth wobble (world acceleration up to about 0.35 m/s²) |
| `cruise` | Straight flight at 5 m/s, random heading |
| `gust` | Hover with 3 horizontal gusts up to 4 m/s² |
| `vibration` | Hover with 70 Hz vibration in the IMU only (0.15 rad/s gyro, 2.5 m/s² accelerometer); ideal gimbal |
| `noise_x10` | Mixed flight (cruise + gusts + vibration) with 10× gyro and accelerometer white noise |
| `drift` | Cruise with gyro bias std 0.05 rad/s per axis (5× default) and bias random walk 2e-3 rad/s/√s |
| `dropouts` | Mixed flight with IMU dropouts: 3 % chance per sample of losing 2–10 consecutive samples |

## Arrays in each `.npz`

N = number of IMU samples (about 8200; fewer with dropouts), F = number of frames (1200).

| Key | Shape | Units / frame | Meaning |
|---|---|---|---|
| `imu_t` | (N,) | s | IMU timestamps (200 Hz; gaps in `dropouts`) |
| `gyro` | (N, 3) | rad/s, body FRD | Measured angular rate (noise, bias, vibration) |
| `accel` | (N, 3) | m/s², body FRD | Measured specific force `Rᵀ(a − g)`; level at rest reads (0, 0, −9.81) |
| `height_t`, `height` | (H,), (H,) | s, m | Measured height above the surface (10 Hz; noise σ 0.3 m + drift) |
| `vel_t`, `vel_ned` | (V,), (V, 3) | s, m/s NED | Flight-controller velocity estimate (10 Hz; noise σ 0.2 m/s) |
| `yaw_t`, `yaw` | (Y,), (Y,) | s, rad NED | Flight-controller heading (5 Hz; noise σ 0.03 rad) |
| `frame_t` | (F,) | s | Camera frame timestamps (30 fps, starting at 1 s) |
| `gimbal` | (F,) | rad | Gimbal pitch at each frame (0 = horizon, π/2 = nadir); 75° here |
| `gt_pos_ned` | (F, 3) | m, NED | True position at each frame (start at 30 m altitude: z = −30) |
| `gt_vel_ned` | (F, 3) | m/s, NED | True velocity at each frame |
| `gt_q_wb` | (F, 4) | — | True attitude quaternion (w, x, y, z), body → world |
| `gt_height` | (F,) | m | True height above the surface |
| `gt_gyro_bias` | (N, 3) | rad/s | True gyro bias at each IMU sample |
| `gt_shift_s1`, `gt_shift_s5` | (F, 2) | px | True shift of image content at the image centre from frame i−stride to frame i (+dx right, +dy down); NaN for the first `stride` frames |

Camera for the truth shifts: 640×360, fx = fy = 457 px, cx = 320, cy = 180, no distortion. The generator computes truth independently of the estimator's projection code (ray → world plane → projection).

## Usage

```python
import json, numpy as np
man = json.load(open("manifest.json"))
with np.load("hover_s100.npz") as z:
    gyro, accel, t = z["gyro"], z["accel"], z["imu_t"]
    truth = z["gt_shift_s1"]
```

Reference evaluation: `python scripts/eval_dataset.py --data data/byeol-synth-v1` in the [byeol repository](../../README.md). It feeds the streams to the estimator in time order, skips the first `eval_skip_s` = 10 s, and prints RMSE, median, share ≤1 px and the valid share per scenario and stride. Baseline results of byeol v0.1.0:

| Scenario | RMSE stride 1 (px) | ≤1 px stride 1 | RMSE stride 5 (px) | ≤1 px stride 5 |
|---|---|---|---|---|
| hover | 0.08 | 100 % | 0.35 | 100 % |
| cruise | 0.08 | 100 % | 0.34 | 99.6 % |
| gust | 0.14 | 100 % | 0.67 | 85.9 % |
| vibration | 0.15 | 100 % | 0.43 | 99.7 % |
| noise_x10 | 0.40 | 99.8 % | 1.08 | 59.2 % |
| drift | 0.32 | 100 % | 1.58 | 54.9 % |
| dropouts | 0.77 | 88.1 % | 2.53 | 37.5 % |

## Generation and reproducibility

`python scripts/make_dataset.py` regenerates every file from fixed seeds with `byeol.synth`. `python scripts/make_dataset.py --check` recomputes everything in memory and compares the content hashes in `manifest.json`. On the reference platform (x86-64, numpy 2.4, Python 3.13) the dataset reproduces bit-exactly. Other numpy builds or CPUs may differ in the last floating-point bits.

## Known limitations and biases

- **Simplified physics.** Attitude follows the thrust vector; there is no aerodynamic drag, rotor dynamics or real wind field. Vibration is sinusoidal at one frequency. Noise is white plus bias random walk.
- **Shared assumptions.** The ground truth uses the same flat-plane and pinhole model as byeol, so it rewards methods that make the same assumptions. It does not test waves, terrain, rolling shutter, lens distortion, gimbal dynamics or time-sync errors.
- **Small size.** 3 seeds per scenario, one altitude (30 m), one gimbal angle (75°), one camera.
- **No images.** The dataset tests sensor-based ego-motion only. Image-based methods need rendered frames, which `byeol.vision_baseline` and `byeol.water_scene` can produce, but which are not part of this dataset.

## License and citation

MIT, like the rest of byeol. Please cite the byeol software ([CITATION.cff](../../CITATION.cff)) and name the dataset as "byeol-synth-v1".

---

## Українська

Детермінований синтетичний бенчмарк для оцінки руху камери дрона за бортовими датчиками. Він містить 21 послідовність (7 сценаріїв польоту × 3 seed × 40 с) сирих потоків датчиків (IMU, швидкість і курс від польотного контролера, висота, кут підвісу, моменти кадрів) з повною істиною, зокрема істинним піксельним зсувом центру кадру між кадрами. Це датасет експерименту E-001 з [технічного звіту byeol](../../docs/TECHNICAL_REPORT.uk.md).

> **Лише синтетичні дані.** Послідовності створено власною спрощеною моделлю польоту й датчиків byeol. У них немає зображень, людей, місць і персональних даних. Добрий результат на цьому бенчмарку потрібен, але недостатній: він не показує якість у реальному світі.

**Склад.** `manifest.json` містить загальні метадані (камера, частоти, конвенції) і для кожної послідовності її id, сценарій, seed, конфігурацію генератора, кількість відліків і SHA-256 (файлу й вмісту масивів). Файли `<сценарій>_s<seed>.npz` — стиснуті архіви NumPy з масивами float64, приблизно 0,6–0,7 МБ кожен, 15 МБ загалом.

**Сценарії** (seed 100, 101, 102): `hover` — висіння з хитанням; `cruise` — прямий політ 5 м/с; `gust` — висіння з трьома горизонтальними поривами до 4 м/с²; `vibration` — висіння з вібрацією 70 Гц лише в IMU; `noise_x10` — змішаний політ із шумом гіро й акселерометра ×10; `drift` — політ зі зсувом нуля гіро 0,05 рад/с на вісь (×5 від типового) і його блуканням; `dropouts` — змішаний політ із пропусками IMU (3 % імовірність втратити 2–10 відліків поспіль).

**Масиви.** Ключі, форми й одиниці наведено в англійській таблиці вище. Коротко: сирі потоки — `imu_t`, `gyro` (рад/с, FRD), `accel` (питома сила, м/с², FRD), `height_t`/`height` (м), `vel_t`/`vel_ned` (м/с, NED), `yaw_t`/`yaw` (рад), `frame_t` (с), `gimbal` (рад). Істина — `gt_pos_ned`, `gt_vel_ned`, `gt_q_wb` (кватерніон w, x, y, z, тіло → світ), `gt_height`, `gt_gyro_bias`, а також `gt_shift_s1` і `gt_shift_s5`: істинний зсув вмісту кадру в центрі кадру від кадру i−крок до кадру i (+dx вправо, +dy вниз; NaN для перших кадрів). Камера для істини: 640×360, fx = fy = 457 пікс, cx = 320, cy = 180, без дисторсії.

**Використання й базові результати.** Еталонна оцінка: `python scripts/eval_dataset.py --data data/byeol-synth-v1` у [репозиторії byeol](../../README.uk.md). Скрипт подає потоки в оцінювач у порядку часу, відкидає перші 10 с і друкує RMSE, медіану, частку ≤1 пікс і частку валідних оцінок. Результати byeol v0.1.0 — у таблиці вище (числа однакові для обох мов).

**Генерація й відтворюваність.** `python scripts/make_dataset.py` перегенеровує всі файли з фіксованих seed. `--check` перераховує все в пам'яті й звіряє хеші вмісту з `manifest.json`. На еталонній платформі (x86-64, numpy 2.4, Python 3.13) збіг біт-у-біт. Інші збірки numpy чи процесори можуть відрізнятися в останніх бітах.

**Обмеження й упередження.**

- **Спрощена фізика.** Орієнтація випливає з вектора тяги; немає аеродинаміки, динаміки гвинтів і справжнього поля вітру. Вібрація синусоїдальна на одній частоті, шум — білий плюс блукання зсуву нуля.
- **Спільні припущення.** Істина спирається на ту саму модель рівної площини й pinhole-камери, що й byeol, тож вона «винагороджує» методи з тими самими припущеннями. Хвилі, рельєф, роллінг-шаттер, дисторсія, динаміка підвісу й розсинхронізація не перевіряються.
- **Малий обсяг.** 3 seed на сценарій, одна висота (30 м), один кут підвісу (75°), одна камера.
- **Немає зображень.** Датасет перевіряє лише оцінку руху за датчиками.

**Ліцензія й цитування.** MIT, як і весь byeol. Цитуйте програму byeol ([CITATION.cff](../../CITATION.cff)) і називайте датасет «byeol-synth-v1».
