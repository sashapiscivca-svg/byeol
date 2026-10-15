"""Inverse-variance fusion of the physics estimate (byeol) with image-based phase correlation (dal.ego).

Phase correlation error model: sigma_v ≈ 0.3 / resp px (per axis), rejected when resp < 0.1. The coefficients were fitted
on a SYNTHETIC water render (docs/TECHNICAL_REPORT.md, E-004); they must be recalibrated for real frames.
Consistency is checked with the Mahalanobis distance (chi-square, 2 DOF, 99%): of two inconsistent measurements, the one with
the larger uncertainty is rejected.
"""
from __future__ import annotations

import numpy as np

from .estimator import EgoResult

RESP_MIN = 0.1
SIGMA_COEF = 0.3
SIGMA_FLOOR = 0.2
GATE_CHI2 = 9.21


def vision_sigma(resp: float) -> float:
    if not resp >= RESP_MIN:
        return float("inf")
    return max(SIGMA_COEF / resp, SIGMA_FLOOR)


def fuse(phys: EgoResult, vision: tuple[float, float, float] | None) -> EgoResult:
    sv = vision_sigma(vision[2]) if vision is not None else float("inf")
    v_ok = np.isfinite(sv) and np.isfinite(vision[0]) and np.isfinite(vision[1])
    if not v_ok:
        return phys
    zv = np.array([vision[0], vision[1]])
    Cv = np.eye(2) * sv**2
    if not phys.valid:
        return EgoResult(zv[0], zv[1], Cv, 1 / (1 + sv**2), True, "vision_only", sv)
    zp = np.array([phys.dx, phys.dy])
    d = zv - zp
    if float(d @ np.linalg.inv(phys.cov + Cv) @ d) > GATE_CHI2:
        return phys if np.sqrt(np.trace(phys.cov) / 2) <= sv else EgoResult(
            zv[0], zv[1], Cv, 1 / (1 + sv**2), True, "vision_gate", sv)
    Pi, Vi = np.linalg.inv(phys.cov), np.linalg.inv(Cv)
    C = np.linalg.inv(Pi + Vi)
    z = C @ (Pi @ zp + Vi @ zv)
    lam = float(np.linalg.eigvalsh(C)[-1])
    return EgoResult(float(z[0]), float(z[1]), C, 1 / (1 + lam), True, "fused", float(np.sqrt(np.trace(C) / 2)))
