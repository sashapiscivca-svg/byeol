"""Sensitivity of byeol ego-motion estimates to camera parameters: f error (±5%, ±10%), resolution/camera type, ignored distortion.

Reproduce (from the repository root): python scripts/eval_sensitivity.py      (~5 min on one core)   [--quick: ~1 min]
The true camera generates the ground truth; the estimator receives a (possibly wrong) model. Metrics: RMSE in pixels of THAT
camera, RMSE in milliradians (RMSE/f*1000, comparable across resolutions), fraction <=1 px.
Output: experiments/eval_sensitivity.json + tables on stdout. See docs/TECHNICAL_REPORT.md.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HOME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOME / "src"))

from byeol import synth
from byeol.cameramodel import CameraModel, Distortion
from byeol.camprofile import load_camera
from byeol.estimator import EgoConfig
from byeol.metrics import summarize
from byeol.replay import replay_multi

CAMS = HOME / "configs" / "cameras"
MODES = ("linear", "gust")


def run(true_cam, est_cam, seeds, dur, pixel=None, strides=(1, 5)):
    outs = {st: [] for st in strides}
    for mode in MODES:
        for sd in range(seeds):
            seq = synth.generate(synth.SynthConfig(mode=mode, duration=dur, seed=300 + sd))
            o = replay_multi(seq, EgoConfig(camera=est_cam, pixels=None if pixel is None else np.array([pixel])),
                             strides, truth_camera=true_cam, pixel=pixel)
            for st in strides:
                outs[st].append(o[st])
    res = {}
    for st in strides:
        pooled = {k: np.concatenate([o[k] for o in outs[st]]) for k in outs[st][0]}
        s = summarize(pooled)
        s["rmse_mrad"] = 1000 * s["rmse"] / true_cam.f
        res[str(st)] = s
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    seeds, dur = (1, 20.0) if a.quick else (2, 30.0)
    base = CameraModel.from_fov(640, 360, hfov_deg=70.0)
    table = {}
    for pct in (-10, -5, 0, 5, 10):
        wrong = replace(base, fx=base.fx * (1 + pct / 100), fy=base.fy * (1 + pct / 100))
        table[f"f {pct:+d}% (640x360)"] = run(base, wrong, seeds, dur)
        print("done f", pct, file=sys.stderr, flush=True)
    for pct in (5,):
        wrong = replace(base, cx=base.cx + 10, cy=base.cy - 6)
        table["principal point offset by (+10, -6) px"] = run(base, wrong, seeds, dur)
    for name in ("generic_rgb_1080p_hfov70", "generic_rgb_640x360_hfov70", "generic_lwir_640x512_hfov32",
                 "generic_lwir_320x256_hfov45", "generic_lwir_160x120_hfov50"):
        c = load_camera(CAMS / f"{name}.yaml")
        table[f"resolution: {name} ({c.width}x{c.height}, f={c.f:.0f})"] = run(c, c, seeds, dur)
        print("done", name, file=sys.stderr, flush=True)
    wide = CameraModel.from_fov(1280, 720, hfov_deg=90.0, dist=Distortion(k1=-0.12, k2=0.02))
    plain = replace(wide, dist=Distortion())
    edge = (1100.0, 360.0)
    table["distortion k1=-0.12: modeled (pixel near edge)"] = run(wide, wide, seeds, dur, pixel=edge)
    table["distortion k1=-0.12: IGNORED (pixel near edge)"] = run(wide, plain, seeds, dur, pixel=edge)
    table["distortion k1=-0.12: IGNORED (frame center)"] = run(wide, plain, seeds, dur)
    for st in ("1", "5"):
        print(f"\n**Frame stride = {st}**\n\n| variant | RMSE, px | RMSE, mrad | fraction <=1 px | zero: RMSE, px |\n|---|---|---|---|---|")
        for k, v in table.items():
            s = v[st]
            print(f"| {k} | {s['rmse']:.3f} | {s['rmse_mrad']:.2f} | {s['within1']:.3f} | {s['zero_rmse']:.2f} |")
    out = HOME / "experiments" / "eval_sensitivity.json"
    out.write_text(json.dumps({"command": " ".join(sys.argv), "seeds": seeds, "duration": dur, "table": table},
                              ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
