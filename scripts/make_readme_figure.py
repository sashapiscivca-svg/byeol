"""Draw docs/assets/byeol-overview.png (the README figure) from the public synthetic data and experiments/eval_ego.json.

    python scripts/make_readme_figure.py        # needs matplotlib
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from eval_dataset import run_sequence  # noqa: E402

from byeol.cameramodel import CameraModel  # noqa: E402


def main() -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    res = json.loads((ROOT / "experiments" / "eval_ego.json").read_text())
    c = res["camera"]
    cam = CameraModel(c["width"], c["height"], c["fx"], c["fy"], c["cx"], c["cy"])
    seq = dict(np.load(ROOT / "data" / "byeol-synth-v1" / "gust_s100.npz"))
    t, dx, dy, gx, gy, sg, val = np.asarray(run_sequence(seq, cam, (5,))[5], dtype=float).T
    m = (t >= 12) & (t <= 32) & (val > 0)

    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={"width_ratios": [1.6, 1]})
    a = ax[0]
    a.plot(t[m], gx[m], color="#222", lw=2.2, label="ground truth")
    a.plot(t[m], dx[m], color="#e8590c", lw=1.2, label="byeol (IMU + velocity + height)")
    a.fill_between(t[m], dx[m] - 2 * sg[m], dx[m] + 2 * sg[m], color="#e8590c", alpha=0.18, label="byeol ±2σ")
    a.set_xlabel("time, s")
    a.set_ylabel("horizontal image shift between frames, px")
    a.set_title("Gusts up to 4 m/s², 5-frame stride: estimate vs truth")
    a.legend(frameon=False, loc="upper left", fontsize=9)

    b = ax[1]
    textures = ["25.0", "1.5", "0.5", "0.3"]
    for method, colour, label in (("vision", "#1c7ed6", "phase correlation (image)"), ("physics", "#e8590c", "byeol (physics)"),
                                  ("fused", "#2b8a3e", "fused")):
        b.plot(range(4), [res["vision"][x]["1"][method]["rmse"] for x in textures], "o-", color=colour, lw=2, label=label)
    b.set_xticks(range(4), ["rich\n(25)", "1.5", "0.5", "calm water\n(0.3)"])
    b.set_yscale("log")
    b.set_ylabel("RMSE, px (log)")
    b.set_xlabel("water texture contrast")
    b.set_title("As texture fades, image methods fail")
    b.legend(frameon=False, fontsize=9)
    fig.text(0.01, 0.01, "All data synthetic (byeol-synth-v1, seed 100); reproduce with scripts/eval_dataset.py and scripts/eval_ego.py.",
             fontsize=8, color="#666")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    out = ROOT / "docs" / "assets" / "byeol-overview.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
