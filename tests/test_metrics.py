import numpy as np

from byeol.metrics import summarize


def test_summarize_perfect_and_zero_baseline():
    n = 100
    t = np.linspace(0, 20, n)
    tdx = np.full(n, 3.0)
    out = {"t": t, "dx": tdx.copy(), "dy": np.zeros(n), "tdx": tdx, "tdy": np.zeros(n), "valid": np.ones(n, bool),
           "cov": np.tile(np.eye(2), (n, 1, 1)), "sigma": np.ones(n)}
    s = summarize(out, t_min=5.0)
    assert s["rmse"] == 0 and s["within1"] == 1.0 and s["zero_rmse"] == 3.0 and s["zero_within1"] == 0.0


def test_invalid_frames_count_as_failures_in_within1_but_not_rmse():
    n = 10
    out = {"t": np.arange(n, dtype=float), "dx": np.zeros(n), "dy": np.zeros(n), "tdx": np.zeros(n),
           "tdy": np.zeros(n), "valid": np.array([True] * 8 + [False] * 2), "cov": np.tile(np.eye(2), (n, 1, 1)),
           "sigma": np.ones(n)}
    s = summarize(out, t_min=0.0)
    assert s["valid_frac"] == 0.8 and s["within1"] == 0.8
