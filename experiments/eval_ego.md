# Ego-motion accuracy (scripts/eval_ego.py)

> All results are on synthetic data. Raw numbers: `eval_ego.json`. Evaluation camera: 640x360, f = 457 px (HFOV ~70°); the first 10 s of each sequence are excluded (filter convergence).

## Physics (byeol) vs ground truth

**Frame stride = 1 (at 30 fps)**

| scenario | RMSE, px | median | p95 | fraction <=1 px | zero: RMSE | zero: <=1 px | 95% coverage | sigma (median) |
|---|---|---|---|---|---|---|---|---|
| hover (wobble) | 0.08 | 0.06 | 0.13 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |
| cruise (5 m/s, gimbal 75°) | 0.08 | 0.06 | 0.15 | 1.000 | 2.93 | 0.098 | 1.00 | 0.31 |
| gust (gusts up to 4 m/s^2) | 0.14 | 0.09 | 0.28 | 1.000 | 4.35 | 0.246 | 1.00 | 0.30 |
| vibration (70 Hz) | 0.15 | 0.13 | 0.24 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |
| noise x10 (IMU noise x10) | 0.40 | 0.33 | 0.69 | 0.998 | 5.03 | 0.052 | 0.99 | 0.40 |
| drift (gyro bias 0.05 rad/s) | 0.32 | 0.20 | 0.73 | 1.000 | 2.93 | 0.098 | 0.94 | 0.31 |
| dropouts (3% IMU samples dropped) | 0.77 | 0.25 | 1.48 | 0.881 | 5.03 | 0.052 | 0.97 | 0.49 |
| cruise NO FC velocity (default: valid=False) | nan | nan | nan | 0.000 | 2.93 | 0.098 | nan | nan |
| cruise NO velocity (research, require_velocity=False) | 2.46 | 2.16 | 3.31 | 0.000 | 2.93 | 0.098 | 0.98 | 1.61 |
| hover NO velocity, prior v=0 (research) | 0.10 | 0.09 | 0.18 | 1.000 | 1.73 | 0.269 | 1.00 | 0.30 |

**Frame stride = 5 (at 30 fps)**

| scenario | RMSE, px | median | p95 | fraction <=1 px | zero: RMSE | zero: <=1 px | 95% coverage | sigma (median) |
|---|---|---|---|---|---|---|---|---|
| hover (wobble) | 0.35 | 0.27 | 0.60 | 1.000 | 8.30 | 0.013 | 1.00 | 0.85 |
| cruise (5 m/s, gimbal 75°) | 0.34 | 0.24 | 0.71 | 0.996 | 14.54 | 0.004 | 1.00 | 0.90 |
| gust (gusts up to 4 m/s^2) | 0.67 | 0.44 | 1.37 | 0.859 | 19.82 | 0.018 | 1.00 | 0.86 |
| vibration (70 Hz) | 0.43 | 0.33 | 0.79 | 0.997 | 8.30 | 0.013 | 1.00 | 0.85 |
| noise x10 (IMU noise x10) | 1.08 | 0.88 | 1.89 | 0.592 | 23.57 | 0.002 | 0.99 | 1.07 |
| drift (gyro bias 0.05 rad/s) | 1.58 | 0.90 | 3.66 | 0.549 | 14.54 | 0.004 | 0.83 | 0.90 |
| dropouts (3% IMU samples dropped) | 2.53 | 1.40 | 4.75 | 0.375 | 23.57 | 0.002 | 0.96 | 3.01 |
| cruise NO FC velocity (default: valid=False) | nan | nan | nan | 0.000 | 14.54 | 0.004 | nan | nan |
| cruise NO velocity (research, require_velocity=False) | 12.38 | 10.88 | 16.68 | 0.000 | 14.54 | 0.004 | 0.96 | 7.88 |
| hover NO velocity, prior v=0 (research) | 0.49 | 0.42 | 0.85 | 0.995 | 8.30 | 0.013 | 1.00 | 0.87 |

## Physics vs phase correlation on synthetic water

**Stride = 1**

| texture contrast | method | RMSE, px | median | fraction <=1 px | p95 |
|---|---|---|---|---|---|
| 25.0 | physics | 0.19 | 0.14 | 1.000 | 0.37 |
| 25.0 | vision | 0.34 | 0.20 | 0.975 | 0.47 |
| 25.0 | fused | 0.27 | 0.15 | 0.992 | 0.39 |
| 25.0 | (zero) | 5.35 | - | 0.042 | - |
| 25.0 | phase correlation resp>=0.1 | 1.00 | | | |
| 1.5 | physics | 0.19 | 0.14 | 1.000 | 0.37 |
| 1.5 | vision | 0.71 | 0.61 | 0.865 | 1.21 |
| 1.5 | fused | 0.33 | 0.24 | 0.990 | 0.55 |
| 1.5 | (zero) | 5.35 | - | 0.042 | - |
| 1.5 | phase correlation resp>=0.1 | 1.00 | | | |
| 0.5 | physics | 0.19 | 0.14 | 1.000 | 0.37 |
| 0.5 | vision | 1.31 | 1.10 | 0.444 | 2.21 |
| 0.5 | fused | 0.29 | 0.22 | 0.994 | 0.48 |
| 0.5 | (zero) | 5.35 | - | 0.042 | - |
| 0.5 | phase correlation resp>=0.1 | 1.00 | | | |
| 0.3 | physics | 0.19 | 0.14 | 1.000 | 0.37 |
| 0.3 | vision | 8.19 | 1.60 | 0.242 | 3.33 |
| 0.3 | fused | 0.23 | 0.18 | 1.000 | 0.41 |
| 0.3 | (zero) | 5.35 | - | 0.042 | - |
| 0.3 | phase correlation resp>=0.1 | 0.97 | | | |

**Stride = 5**

| texture contrast | method | RMSE, px | median | fraction <=1 px | p95 |
|---|---|---|---|---|---|
| 25.0 | physics | 0.83 | 0.56 | 0.817 | 1.77 |
| 25.0 | vision | 28.47 | 0.81 | 0.685 | 2.00 |
| 25.0 | fused | 0.75 | 0.69 | 0.867 | 1.16 |
| 25.0 | (zero) | 24.37 | - | 0.000 | - |
| 25.0 | phase correlation resp>=0.1 | 0.96 | | | |
| 1.5 | physics | 0.83 | 0.56 | 0.817 | 1.77 |
| 1.5 | vision | 28.49 | 0.89 | 0.565 | 2.80 |
| 1.5 | fused | 0.72 | 0.65 | 0.877 | 1.21 |
| 1.5 | (zero) | 24.37 | - | 0.000 | - |
| 1.5 | phase correlation resp>=0.1 | 0.95 | | | |
| 0.5 | physics | 0.83 | 0.56 | 0.817 | 1.77 |
| 0.5 | vision | 29.17 | 1.46 | 0.283 | 63.11 |
| 0.5 | fused | 0.81 | 0.63 | 0.790 | 1.56 |
| 0.5 | (zero) | 24.37 | - | 0.000 | - |
| 0.5 | phase correlation resp>=0.1 | 0.92 | | | |
| 0.3 | physics | 0.83 | 0.56 | 0.817 | 1.77 |
| 0.3 | vision | 131.68 | 3.17 | 0.113 | 109.37 |
| 0.3 | fused | 0.84 | 0.61 | 0.798 | 1.70 |
| 0.3 | (zero) | 24.37 | - | 0.000 | - |
| 0.3 | phase correlation resp>=0.1 | 0.58 | | | |
