"""Evaluate byeol: (dx, dy) accuracy vs ground truth on synthetic sequences + comparison with phase correlation.

Reproduce (from the repository root):
             python scripts/eval_ego.py                       (full, ~10 min on one core)
             python scripts/eval_ego.py --quick               (quick, ~2 min)
Output: experiments/eval_ego.json and Markdown tables on stdout.
Evaluation camera: CameraModel 640x360, f = 457 px (HFOV ~70 degrees). The first 10 s of each sequence are excluded
(filter convergence). See docs/TECHNICAL_REPORT.md.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np

HOME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOME / "src"))

from byeol import fuse as fusemod
from byeol import synth
from byeol.cameramodel import CameraModel
from byeol.estimator import EgoConfig, EgoResult
from byeol.metrics import summarize
from byeol.replay import replay_multi

K = CameraModel.from_pixels(640, 360, fx=457.0, fy=457.0, cx=320.0, cy=180.0)

# (name, SynthConfig, EgoConfig, replay)
SCENARIOS = {
    "hover (wobble)": (dict(mode="hover"), {}, {}),
    "cruise (5 m/s, gimbal 75°)": (dict(mode="linear"), {}, {}),
    "gust (gusts up to 4 m/s^2)": (dict(mode="gust"), {}, {}),
    "vibration (70 Hz)": (dict(mode="vibration"), {}, {}),
    "noise x10 (IMU noise x10)": (dict(mode="mixed", gyro_noise=3e-3, accel_noise=4e-2), dict(gyro_density=3e-3), {}),
    "drift (gyro bias 0.05 rad/s)": (dict(mode="linear", gyro_bias=0.05, gyro_bias_walk=2e-3), {}, {}),
    "dropouts (3% IMU samples dropped)": (dict(mode="mixed", dropout_prob=0.03), {}, {}),
    "cruise NO FC velocity (default: valid=False)": (dict(mode="linear"), {}, dict(aid_velocity=False)),
    "cruise NO velocity (research, require_velocity=False)": (
        dict(mode="linear"), dict(require_velocity=False), dict(aid_velocity=False)),
    "hover NO velocity, prior v=0 (research)": (
        dict(mode="hover"), dict(zero_vel_sigma=0.5, require_velocity=False), dict(aid_velocity=False)),
}


def pool(outs: list[dict]) -> dict:
    return {k: np.concatenate([o[k] for o in outs]) for k in outs[0]}


def run_physics(seeds: int, duration: float, strides=(1, 5)) -> dict:
    res = {}
    for name, (sc, ec, rp) in SCENARIOS.items():
        per = {st: [] for st in strides}
        for sd in range(seeds):
            seq = synth.generate(synth.SynthConfig(duration=duration, seed=100 + sd, **sc))
            out = replay_multi(seq, EgoConfig(camera=K, **ec), strides, **rp)
            for st in strides:
                per[st].append(out[st])
        res[name] = {str(st): summarize(pool(per[st])) for st in strides}
        print(f"[{time.strftime('%H:%M:%S')}] {name} done", file=sys.stderr, flush=True)
    return res


def run_vision(duration: float, contrasts, strides=(1, 5)) -> dict:
    from byeol.vision_baseline import WaterRenderer, phase_shift

    res = {}
    seq = synth.generate(synth.SynthConfig(mode="mixed", duration=duration, seed=7))
    phys = replay_multi(seq, EgoConfig(camera=K), strides)
    for c in contrasts:
        rend = WaterRenderer(K, (640, 360), contrast=c, sensor_noise=1.5, seed=3)
        frames = [rend.render(seq, i) for i in range(len(seq.frame_t))]
        res[str(c)] = {}
        for st in strides:
            p = phys[st]
            vis = {k: np.zeros(len(p["t"])) for k in ("dx", "dy", "resp")}
            fz = {k: np.zeros(len(p["t"])) for k in ("dx", "dy", "sigma")}
            fz_cov = np.zeros((len(p["t"]), 2, 2))
            for n, i in enumerate(p["frame"]):
                dx, dy, resp = phase_shift(frames[i - st], frames[i])
                vis["dx"][n], vis["dy"][n], vis["resp"][n] = dx, dy, resp
                r_p = EgoResult(p["dx"][n], p["dy"][n], p["cov"][n], p["conf"][n], bool(p["valid"][n]), "ok", p["sigma"][n])
                r_f = fusemod.fuse(r_p, (dx, dy, resp))
                fz["dx"][n], fz["dy"][n], fz["sigma"][n] = r_f.dx, r_f.dy, r_f.sigma
                fz_cov[n] = r_f.cov
            base = {k: p[k] for k in ("t", "tdx", "tdy")}
            sig_v = np.array([min(fusemod.vision_sigma(r), 1e3) for r in vis["resp"]])
            o_vis = dict(base, dx=vis["dx"], dy=vis["dy"], valid=np.ones(len(p["t"]), bool),
                         cov=np.einsum("n,ij->nij", sig_v**2, np.eye(2)), sigma=sig_v)
            o_fz = dict(base, dx=fz["dx"], dy=fz["dy"], valid=np.isfinite(fz["dx"]), cov=fz_cov, sigma=fz["sigma"])
            res[str(c)][str(st)] = {
                "physics": summarize(p), "vision": summarize(o_vis), "fused": summarize(o_fz),
                "vision_resp_ge_0.1": float((vis["resp"][p["t"] >= 10] >= fusemod.RESP_MIN).mean()),
            }
        print(f"[{time.strftime('%H:%M:%S')}] contrast {c} done", file=sys.stderr, flush=True)
    return res


def fmt_phys(res: dict) -> str:
    lines = []
    for st in ("1", "5"):
        lines += [f"\n**Frame stride = {st} (at 30 fps)**\n",
                  "| scenario | RMSE, px | median | p95 | fraction <=1 px | zero: RMSE | zero: <=1 px | 95% coverage | sigma (median) |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for name, d in res.items():
            s = d[st]
            lines.append(f"| {name} | {s['rmse']:.2f} | {s['median']:.2f} | {s['p95']:.2f} | {s['within1']:.3f} | "
                         f"{s['zero_rmse']:.2f} | {s['zero_within1']:.3f} | {s.get('calib95', float('nan')):.2f} | "
                         f"{s.get('sigma_med', float('nan')):.2f} |")
    return "\n".join(lines)


def fmt_vision(res: dict) -> str:
    lines = []
    for st in ("1", "5"):
        lines += [f"\n**Stride = {st}**\n",
                  "| texture contrast | method | RMSE, px | median | fraction <=1 px | p95 |", "|---|---|---|---|---|---|"]
        for c, d in res.items():
            x = d[st]
            for m in ("physics", "vision", "fused"):
                y = x[m]
                lines.append(f"| {c} | {m} | {y['rmse']:.2f} | {y['median']:.2f} | {y['within1']:.3f} | {y['p95']:.2f} |")
            lines.append(f"| {c} | (zero) | {x['physics']['zero_rmse']:.2f} | - | {x['physics']['zero_within1']:.3f} | - |")
            lines.append(f"| {c} | phase correlation resp>=0.1 | {x['vision_resp_ge_0.1']:.2f} | | | |")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--skip-vision", action="store_true")
    ap.add_argument("--out", default=str(HOME / "experiments" / "eval_ego.json"))
    a = ap.parse_args()
    seeds, dur = (1, 25.0) if a.quick else (a.seeds, a.duration)
    out = {"command": " ".join(sys.argv), "seeds": seeds, "duration": dur, "camera": dataclasses.asdict(K)}
    out["physics"] = run_physics(seeds, dur)
    print("## Physics (byeol) vs ground truth\n" + fmt_phys(out["physics"]))
    if not a.skip_vision:
        contr = (25.0, 1.5, 0.5, 0.3) if not a.quick else (25.0, 0.5)
        out["vision"] = run_vision(25.0 if not a.quick else 20.0, contr)
        print("\n## Physics vs phase correlation on synthetic water\n" + fmt_vision(out["vision"]))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
