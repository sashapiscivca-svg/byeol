"""Relationship between phase-correlation response (resp) and its error on synthetic water: basis for fuse.vision_sigma (D-008).

Reproduce (from the repository root): python scripts/fit_vision_sigma.py
For several texture contrasts, prints the median error per resp bin and an estimate of the coefficient c in err_med ≈ c / resp.
See docs/TECHNICAL_REPORT.md.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from byeol import camera as cam
from byeol import synth
from byeol.vision_baseline import WaterRenderer, phase_shift

K = cam.Pinhole(457.0, 320.0, 180.0)
BINS = ((0.1, 0.2), (0.2, 0.4), (0.4, 0.7), (0.7, 1.3))

allrows = []
for contrast in (1.5, 0.7, 0.3):
    seq = synth.generate(synth.SynthConfig(mode="mixed", duration=15, seed=5))
    rend = WaterRenderer(K, (640, 360), contrast=contrast, sensor_noise=1.5, seed=1)
    fr = [rend.render(seq, i) for i in range(len(seq.frame_t))]
    for i in range(5, len(fr)):
        dx, dy, resp = phase_shift(fr[i - 1], fr[i])
        tdx, tdy = seq.truth_shift(K, i - 1, i)
        allrows.append((resp, np.hypot(dx - tdx, dy - tdy)))
a = np.array(allrows)
print(f"frames: {len(a)}")
for lo, hi in BINS:
    m = (a[:, 0] >= lo) & (a[:, 0] < hi)
    if m.sum() > 5:
        med = np.median(a[m, 1])
        mid = np.sqrt(lo * hi)
        print(f"resp [{lo},{hi}): n={m.sum()}, median error {med:.2f} px, c = med*resp ≈ {med * a[m, 0].mean():.2f}")
