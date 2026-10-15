"""Drone velocity estimation (world NED) with a Kalman filter: accelerometer integration + (optional) flight controller (FC) velocity.

State x = [v (3, world), b_a (3, accelerometer bias in body frame)]. Prediction: v += (R (f - b_a) + g) dt.
Measurements: velocity from the flight controller (e.g. MAVLink LOCAL_POSITION_NED), or "zero velocity"
as a weak prior for hover. Without measurements the uncertainty of v grows, and this is honestly propagated into the covariance (D-006).
"""
from __future__ import annotations

import numpy as np

G_VEC = np.array([0.0, 0.0, 9.80665])


class VelocityKF:
    def __init__(self, accel_noise: float = 0.15, bias_walk: float = 5e-3, p_v0: float = 4.0, p_b0: float = 0.01) -> None:
        self.x = np.zeros(6)
        self.P = np.diag([p_v0] * 3 + [p_b0] * 3).astype(float)
        self.sa2 = accel_noise**2  # acceleration noise (std, m/s^2, per sample)
        self.sb2 = bias_walk**2  # bias random walk, (m/s^2)^2 / s

    @property
    def v(self) -> np.ndarray:
        return self.x[:3]

    @property
    def ba(self) -> np.ndarray:
        return self.x[3:]

    def predict(self, dt: float, R_wb: np.ndarray, f: np.ndarray) -> None:
        a_w = R_wb @ (np.asarray(f, float) - self.x[3:]) + G_VEC
        self.x[:3] += a_w * dt
        F = np.eye(6)
        F[:3, 3:] = -R_wb * dt
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * self.sa2 * dt * dt
        Q[3:, 3:] = np.eye(3) * self.sb2 * dt
        self.P = F @ self.P @ F.T + Q

    def update_velocity(self, v_meas: np.ndarray, sigma: float) -> None:
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)
        S = H @ self.P @ H.T + np.eye(3) * sigma**2
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x += Kg @ (np.asarray(v_meas, float) - self.x[:3])
        I_KH = np.eye(6) - Kg @ H
        self.P = I_KH @ self.P @ I_KH.T + Kg @ (np.eye(3) * sigma**2) @ Kg.T
