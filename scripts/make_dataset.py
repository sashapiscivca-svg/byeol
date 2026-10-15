"""Build the public synthetic benchmark `byeol-synth-v1` (deterministic, seeded).

Reproduce: python scripts/make_dataset.py                 (writes data/byeol-synth-v1/, ~1 min)
           python scripts/make_dataset.py --check         (regenerate in memory and verify SHA-256 against the manifest)

Each sequence is one compressed `.npz` with raw sensor streams (exactly what the estimator consumes) and ground
truth. Scenarios and seeds match experiment E-001 (docs/TECHNICAL_REPORT.md), so `scripts/eval_dataset.py` on this
dataset reproduces the E-001 numbers. Conventions: SI units, world NED, body FRD, camera x-right / y-down / z-forward.
"""
from __future__ import annotations

import argparse
import dataclasses
import hashlib
import io
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from byeol import __version__, synth
from byeol.cameramodel import CameraModel
from byeol.rotation import R_to_q

NAME = "byeol-synth-v1"
DURATION = 40.0
SEEDS = (100, 101, 102)
STRIDES = (1, 5)
# Evaluation camera of E-001 (640x360, f = 457 px, HFOV ~70 deg); truth shifts are given for the image centre.
CAMERA = CameraModel.from_pixels(640, 360, fx=457.0, fy=457.0, cx=320.0, cy=180.0)

SCENARIOS = {
    "hover": dict(mode="hover"),
    "cruise": dict(mode="linear"),
    "gust": dict(mode="gust"),
    "vibration": dict(mode="vibration"),
    "noise_x10": dict(mode="mixed", gyro_noise=3e-3, accel_noise=4e-2),
    "drift": dict(mode="linear", gyro_bias=0.05, gyro_bias_walk=2e-3),
    "dropouts": dict(mode="mixed", dropout_prob=0.03),
}
DESCRIPTIONS = {
    "hover": "Hover with smooth wobble (world acceleration up to ~0.35 m/s^2).",
    "cruise": "Straight flight at 5 m/s, random heading.",
    "gust": "Hover with 3 horizontal gusts up to 4 m/s^2.",
    "vibration": "Hover with 70 Hz vibration in the IMU only (0.15 rad/s gyro, 2.5 m/s^2 accel); gimbal is ideal.",
    "noise_x10": "Mixed flight (cruise + gusts + vibration) with 10x gyro/accel white noise.",
    "drift": "Cruise with gyro bias std 0.05 rad/s per axis (5x default) and bias random walk 2e-3 rad/s/sqrt(s).",
    "dropouts": "Mixed flight with IMU dropouts: 3% chance per sample to lose 2-10 consecutive samples.",
}


def build(seq: synth.Sequence) -> dict[str, np.ndarray]:
    ft = seq.frame_t
    out = {
        "imu_t": seq.imu_t, "gyro": seq.gyro, "accel": seq.accel,
        "height_t": seq.height_t, "height": seq.height,
        "vel_t": seq.aux_vel_t, "vel_ned": seq.aux_vel,
        "yaw_t": seq.yaw_t, "yaw": seq.yaw,
        "frame_t": ft, "gimbal": np.array([seq.gimbal(t) for t in ft]),
        "gt_gyro_bias": seq.gyro_bias_true,
        "gt_pos_ned": np.array([seq.pos(t) for t in ft]),
        "gt_vel_ned": np.array([seq.vel(t) for t in ft]),
        "gt_q_wb": np.array([R_to_q(seq.R_wb(t)) for t in ft]),
        "gt_height": np.array([seq.height_true(t) for t in ft]),
    }
    for st in STRIDES:
        sh = np.full((len(ft), 2), np.nan)
        for i in range(st, len(ft)):
            sh[i] = seq.truth_shift(CAMERA, i - st, i)
        out[f"gt_shift_s{st}"] = sh
    return out


def to_bytes(arrays: dict[str, np.ndarray]) -> bytes:
    buf = io.BytesIO()
    np.savez_compressed(buf, **arrays)
    return buf.getvalue()


def sequences():
    for name, sc in SCENARIOS.items():
        for seed in SEEDS:
            cfg = synth.SynthConfig(duration=DURATION, seed=seed, **sc)
            yield f"{name}_s{seed}", name, cfg, build(synth.generate(cfg))


def arrays_digest(arrays: dict[str, np.ndarray]) -> str:
    """Hash of array contents (independent of zip timestamps/compression)."""
    h = hashlib.sha256()
    for k in sorted(arrays):
        a = np.ascontiguousarray(arrays[k])
        h.update(k.encode() + str(a.dtype).encode() + str(a.shape).encode() + a.tobytes())
    return h.hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=ROOT / "data" / NAME)
    ap.add_argument("--check", action="store_true", help="verify content hashes against an existing manifest")
    a = ap.parse_args()
    if a.check:
        man = json.loads((a.out / "manifest.json").read_text(encoding="utf-8"))
        want = {s["id"]: s["content_sha256"] for s in man["sequences"]}
        bad = [sid for sid, _, _, arr in sequences() if arrays_digest(arr) != want.get(sid)]
        print("OK: all sequences reproduce bit-exactly" if not bad else f"MISMATCH: {bad}")
        sys.exit(1 if bad else 0)
    a.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for sid, name, cfg, arr in sequences():
        blob = to_bytes(arr)
        (a.out / f"{sid}.npz").write_bytes(blob)
        rows.append({
            "id": sid, "scenario": name, "file": f"{sid}.npz", "seed": cfg.seed,
            "description": DESCRIPTIONS[name],
            "n_imu": len(arr["imu_t"]), "n_frames": len(arr["frame_t"]),
            "file_sha256": hashlib.sha256(blob).hexdigest(), "content_sha256": arrays_digest(arr),
            "synth_config": dataclasses.asdict(cfg),
        })
        print(f"{sid}: {len(blob) / 1e6:.2f} MB")
    manifest = {
        "name": NAME, "version": "1.0.0", "license": "MIT", "generator": f"byeol {__version__} (src/byeol/synth.py)",
        "duration_s": DURATION, "warmup_s": synth.SynthConfig().warmup, "eval_skip_s": 10.0,
        "camera": {"width": 640, "height": 360, "fx": 457.0, "fy": 457.0, "cx": 320.0, "cy": 180.0,
                   "distortion": [0.0] * 5, "reference_pixel": [320.0, 180.0]},
        "rates_hz": {"imu": 200.0, "frames": 30.0, "height": 10.0, "fc_velocity": 10.0, "fc_heading": 5.0},
        "conventions": {"world": "NED", "body": "FRD", "camera": "x right, y down, z optical axis",
                        "gimbal": "rad; 0 = forward horizontal, pi/2 = nadir",
                        "quaternion": "w, x, y, z; rotates body to world (R_wb)",
                        "shift": "px; apparent motion of image content from frame i-stride to frame i; "
                                 "+dx right, +dy down; NaN for the first `stride` frames",
                        "time": "s, single clock for all streams"},
        "sequences": rows,
    }
    (a.out / "manifest.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {len(rows)} sequences to {a.out}")


if __name__ == "__main__":
    main()
