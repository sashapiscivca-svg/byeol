"""Shift estimation metrics: RMSE (px), fraction of frames with error <= 1 px, uncertainty calibration, zero baseline."""
from __future__ import annotations

import numpy as np


def summarize(out: dict[str, np.ndarray], t_min: float = 10.0) -> dict[str, float]:
    sel = out["t"] >= t_min
    n = int(sel.sum())
    valid = out["valid"][sel]
    ex = (out["dx"] - out["tdx"])[sel]
    ey = (out["dy"] - out["tdy"])[sel]
    e = np.hypot(ex, ey)
    z = np.hypot(out["tdx"], out["tdy"])[sel]
    ev = e[valid]
    res = {
        "n": n,
        "valid_frac": float(valid.mean()) if n else float("nan"),
        "rmse": float(np.sqrt((ev**2).mean())) if len(ev) else float("nan"),
        "median": float(np.median(ev)) if len(ev) else float("nan"),
        "p95": float(np.percentile(ev, 95)) if len(ev) else float("nan"),
        "within1": float(((e <= 1.0) & valid).mean()) if n else float("nan"),
        "zero_rmse": float(np.sqrt((z**2).mean())) if n else float("nan"),
        "zero_within1": float((z <= 1.0).mean()) if n else float("nan"),
    }
    if len(ev) and "cov" in out:
        d = np.stack([ex, ey], axis=1)[valid]
        cov = out["cov"][sel][valid]
        m2 = np.einsum("ni,nij,nj->n", d, np.linalg.inv(cov), d)
        res["calib95"] = float((m2 < 5.99).mean())
        res["sigma_med"] = float(np.median(out["sigma"][sel][valid]))
    return res
