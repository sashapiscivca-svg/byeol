"""Orientation from IMU: Mahony-type complementary filter (own implementation).

Idea: Mahony, Hamel, Pflimlin, "Nonlinear complementary filters on the special orthogonal group", IEEE TAC 2008
(inspiration only; the code is original). The gyroscope is integrated, the accelerometer corrects tilt (the "down" direction), an optional heading
(from the flight controller/compass) corrects yaw; the gyroscope bias is estimated by an integral term.
The accelerometer correction is attenuated when |f| differs from g (dynamic accelerations, vibration).
World NED, body FRD; quaternion R_wb ([w,x,y,z]). The accelerometer reads specific force f = R^T (a - g); at rest f = (0,0,-g).
"""
from __future__ import annotations

import numpy as np

from . import rotation as rot

G = 9.80665


class Mahony:
    def __init__(self, kp: float = 1.2, ki: float = 0.15, kp_yaw: float = 0.8, ki_yaw: float = 0.005,
                 gate: float = 0.25, accel_tau: float = 0.08) -> None:
        self.kp, self.ki, self.kp_yaw, self.ki_yaw, self.gate = kp, ki, kp_yaw, ki_yaw, gate
        self.accel_tau = accel_tau  # time constant of the accelerometer low-pass filter (suppresses vibration), s
        self._f = None  # smoothed specific force
        self._yaw_meas = None  # latest FC heading (held until a new one arrives)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)  # estimated gyroscope bias, rad/s (body)
        self.last_weight = 0.0

    @property
    def R(self) -> np.ndarray:
        return rot.q_to_R(self.q)

    def init_from_accel(self, f: np.ndarray, yaw: float = 0.0) -> None:
        """Initial roll/pitch from a static accelerometer reading, heading = yaw."""
        self._f = np.asarray(f, float).copy()
        down_b = -np.asarray(f, float)
        down_b /= np.linalg.norm(down_b)
        roll = np.arctan2(down_b[1], down_b[2])
        pitch = np.arctan2(-down_b[0], np.hypot(down_b[1], down_b[2]))
        self.q = rot.R_to_q(rot.euler_to_R(roll, pitch, yaw))

    def update(self, dt: float, gyro: np.ndarray, accel: np.ndarray, yaw: float | None = None) -> np.ndarray:
        R = rot.q_to_R(self.q)
        corr = np.zeros(3)
        a = np.asarray(accel, float)
        if self._f is None:
            self._f = a.copy()
        self._f += (dt / (self.accel_tau + dt)) * (a - self._f)
        accel = self._f
        fn = float(np.linalg.norm(accel))
        w = 0.0
        if fn > 1e-6:
            w = max(0.0, 1.0 - abs(fn - G) / (self.gate * G))
            meas_down = -np.asarray(accel, float) / fn
            est_down = R[2, :]  # R^T e_z
            e = np.cross(meas_down, est_down)
            corr += w * (self.kp * e)
            self.bias -= w * self.ki * e * dt
        self.last_weight = w
        if yaw is not None:
            self._yaw_meas = yaw
        if self._yaw_meas is not None:
            ey = rot.wrap_pi(self._yaw_meas - rot.R_to_euler(R)[2])
            axis = R[2, :]  # world "down" axis expressed in body frame
            corr += self.kp_yaw * ey * axis
            self.bias -= self.ki_yaw * ey * axis * dt
        omega = np.asarray(gyro, float) - self.bias + corr
        self.q = rot.q_normalize(rot.q_mul(self.q, rot.q_exp(omega * dt)))
        return self.q
