# Sensitivity to camera parameters (scripts/eval_sensitivity.py)

> All results are on synthetic data. Raw numbers: `eval_sensitivity.json`. RMSE in mrad = RMSE / f * 1000 (comparable across resolutions).

**Frame stride = 1**

| variant | RMSE, px | RMSE, mrad | fraction <=1 px | zero: RMSE, px |
|---|---|---|---|---|
| f -10% (640x360) | 0.323 | 0.71 | 0.989 | 3.08 |
| f -5% (640x360) | 0.185 | 0.41 | 0.998 | 3.08 |
| f +0% (640x360) | 0.108 | 0.24 | 1.000 | 3.08 |
| f +5% (640x360) | 0.191 | 0.42 | 0.998 | 3.08 |
| f +10% (640x360) | 0.330 | 0.72 | 0.989 | 3.08 |
| principal point offset by (+10, -6) px | 0.108 | 0.24 | 1.000 | 3.08 |
| resolution: generic_rgb_1080p_hfov70 (1920x1080, f=1371) | 0.325 | 0.24 | 0.977 | 9.24 |
| resolution: generic_rgb_640x360_hfov70 (640x360, f=457) | 0.108 | 0.24 | 1.000 | 3.08 |
| resolution: generic_lwir_640x512_hfov32 (640x512, f=1116) | 0.265 | 0.24 | 0.991 | 7.52 |
| resolution: generic_lwir_320x256_hfov45 (320x256, f=390) | 0.092 | 0.24 | 1.000 | 2.63 |
| resolution: generic_lwir_160x120_hfov50 (160x120, f=172) | 0.041 | 0.24 | 1.000 | 1.16 |
| distortion k1=-0.12: modeled (pixel near edge) | 0.193 | 0.30 | 1.000 | 4.87 |
| distortion k1=-0.12: IGNORED (pixel near edge) | 0.707 | 1.10 | 0.948 | 4.87 |
| distortion k1=-0.12: IGNORED (frame center) | 0.152 | 0.24 | 1.000 | 4.31 |

**Frame stride = 5**

| variant | RMSE, px | RMSE, mrad | fraction <=1 px | zero: RMSE, px |
|---|---|---|---|---|
| f -10% (640x360) | 1.564 | 3.42 | 0.387 | 14.90 |
| f -5% (640x360) | 0.899 | 1.97 | 0.779 | 14.90 |
| f +0% (640x360) | 0.529 | 1.16 | 0.945 | 14.90 |
| f +5% (640x360) | 0.929 | 2.03 | 0.780 | 14.90 |
| f +10% (640x360) | 1.599 | 3.50 | 0.380 | 14.90 |
| principal point offset by (+10, -6) px | 0.529 | 1.16 | 0.945 | 14.90 |
| resolution: generic_rgb_1080p_hfov70 (1920x1080, f=1371) | 1.587 | 1.16 | 0.600 | 44.69 |
| resolution: generic_rgb_640x360_hfov70 (640x360, f=457) | 0.529 | 1.16 | 0.945 | 14.90 |
| resolution: generic_lwir_640x512_hfov32 (640x512, f=1116) | 1.292 | 1.16 | 0.698 | 36.37 |
| resolution: generic_lwir_320x256_hfov45 (320x256, f=390) | 0.452 | 1.16 | 0.957 | 12.73 |
| resolution: generic_lwir_160x120_hfov50 (160x120, f=172) | 0.199 | 1.16 | 1.000 | 5.59 |
| distortion k1=-0.12: modeled (pixel near edge) | 0.938 | 1.47 | 0.807 | 23.47 |
| distortion k1=-0.12: IGNORED (pixel near edge) | 3.670 | 5.74 | 0.158 | 23.47 |
| distortion k1=-0.12: IGNORED (frame center) | 0.743 | 1.16 | 0.900 | 20.84 |
