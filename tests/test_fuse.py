import numpy as np

from byeol.estimator import EgoResult
from byeol.fuse import fuse, vision_sigma


def phys(dx, dy, s):
    return EgoResult(dx, dy, np.eye(2) * s**2, 1 / (1 + s**2), True, "ok", s)


def test_vision_sigma_decreases_with_response():
    assert vision_sigma(0.6) < vision_sigma(0.2) < vision_sigma(0.12)
    assert vision_sigma(0.01) == float("inf")


def test_fusion_is_between_and_tighter():
    r = fuse(phys(1.0, 0.0, 0.5), (1.4, 0.0, 0.6))
    assert 1.0 < r.dx < 1.4 and r.sigma < 0.5


def test_inconsistent_vision_is_rejected():
    r = fuse(phys(1.0, 0.0, 0.3), (30.0, 5.0, 0.6))
    assert r.dx == 1.0


def test_weak_vision_ignored_physics_kept():
    r = fuse(phys(1.0, 2.0, 0.3), (9.0, 9.0, 0.03))
    assert (r.dx, r.dy) == (1.0, 2.0)


def test_invalid_physics_falls_back_to_vision():
    bad = EgoResult(float("nan"), float("nan"), np.eye(2) * 1e6, 0.0, False, "no_imu", float("inf"))
    r = fuse(bad, (1.0, 2.0, 0.5))
    assert r.valid and (r.dx, r.dy) == (1.0, 2.0)


def test_both_missing_invalid():
    bad = EgoResult(float("nan"), float("nan"), np.eye(2) * 1e6, 0.0, False, "no_imu", float("inf"))
    assert not fuse(bad, None).valid
