"""Run the byeol estimator on the public `byeol-synth-v1` files (no generator involved) and report accuracy.

Reproduce: python scripts/eval_dataset.py [--data data/byeol-synth-v1] [--only hover]
This is also the reference for how to feed recorded streams to `EgoEstimator`: push every stream in time order,
then query `frame_shift(t_prev, t_cur)` for each frame. Metrics skip the first `eval_skip_s` seconds (filter warm-up).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from byeol.cameramodel import CameraModel
from byeol.estimator import EgoConfig, EgoEstimator


def run_sequence(d: dict[str, np.ndarray], cam: CameraModel, strides: tuple[int, ...]) -> dict[int, np.ndarray]:
    """Returns, per stride, rows (t, dx, dy, gt_dx, gt_dy, sigma, valid)."""
    est = EgoEstimator(EgoConfig(camera=cam))
    ev = [(t, 0, i) for i, t in enumerate(d["imu_t"])] + [(t, 1, i) for i, t in enumerate(d["height_t"])]
    ev += [(t, 2, i) for i, t in enumerate(d["vel_t"])] + [(t, 3, i) for i, t in enumerate(d["yaw_t"])]
    ev += [(t, 5, i) for i, t in enumerate(d["frame_t"])]
    ev.sort(key=lambda e: (e[0], e[1]))
    rows: dict[int, list] = {s: [] for s in strides}
    for t, kind, i in ev:
        if kind == 0:
            est.push_imu(t, d["gyro"][i], d["accel"][i])
        elif kind == 1:
            est.push_height(t, d["height"][i])
        elif kind == 2:
            est.push_velocity(t, d["vel_ned"][i], 0.2)
        elif kind == 3:
            est.push_heading(t, d["yaw"][i])
        else:
            est.push_gimbal(t, d["gimbal"][i])
            for s in strides:
                if i >= s:
                    r = est.frame_shift(float(d["frame_t"][i - s]), float(t))
                    gx, gy = d[f"gt_shift_s{s}"][i]
                    rows[s].append((t, r.dx, r.dy, gx, gy, r.sigma, r.valid))
    return {s: np.array(r, dtype=float) for s, r in rows.items()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--data", type=Path, default=ROOT / "data" / "byeol-synth-v1")
    ap.add_argument("--only", default=None, help="evaluate one scenario only (e.g. hover)")
    a = ap.parse_args()
    man = json.loads((a.data / "manifest.json").read_text(encoding="utf-8"))
    c = man["camera"]
    cam = CameraModel.from_pixels(c["width"], c["height"], fx=c["fx"], fy=c["fy"], cx=c["cx"], cy=c["cy"])
    strides = (1, 5)
    pooled: dict[tuple[str, int], list] = {}
    for s in man["sequences"]:
        if a.only and s["scenario"] != a.only:
            continue
        with np.load(a.data / s["file"]) as z:
            d = {k: z[k] for k in z.files}
        for st, rows in run_sequence(d, cam, strides).items():
            pooled.setdefault((s["scenario"], st), []).append(rows[rows[:, 0] >= man["eval_skip_s"]])
    print("| scenario | stride | RMSE, px | median, px | share <= 1 px | valid |")
    print("|---|---|---|---|---|---|")
    for (name, st), parts in pooled.items():
        r = np.concatenate(parts)
        ok = r[:, 6] > 0
        e = np.hypot(r[:, 1] - r[:, 3], r[:, 2] - r[:, 4])
        ev = e[ok]  # same definitions as byeol.metrics.summarize: RMSE over valid frames,
        within1 = np.mean((e <= 1.0) & ok)  # share <= 1 px over all frames (invalid frames count as misses)
        print(f"| {name} | {st} | {np.sqrt(np.mean(ev**2)):.2f} | {np.median(ev):.2f} | {within1:.3f} "
              f"| {ok.mean():.3f} |")


if __name__ == "__main__":
    main()
