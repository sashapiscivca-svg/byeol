"""Rotations: quaternions [w, x, y, z], rotation vectors, Euler angles (aerospace Z-Y-X sequence)."""
from __future__ import annotations

import numpy as np


def rotvec_to_R(v: np.ndarray) -> np.ndarray:
    """Rodrigues: rotation vector (rad) -> 3x3 matrix."""
    th = float(np.linalg.norm(v))
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + (np.sin(th) / th) * K + ((1 - np.cos(th)) / th**2) * K @ K


def q_exp(v: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(v))
    if th < 1e-9:
        q = np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]])
        return q / np.linalg.norm(q)
    s = np.sin(th / 2) / th
    return np.array([np.cos(th / 2), s * v[0], s * v[1], s * v[2]])


def q_log(q: np.ndarray) -> np.ndarray:
    if q[0] < 0:
        q = -q
    n = float(np.linalg.norm(q[1:]))
    if n < 1e-12:
        return 2 * q[1:]
    return q[1:] * (2 * np.arctan2(n, q[0]) / n)


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def q_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_q(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = np.zeros(4)
        q[1 + i] = s / 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return q_normalize(q if q[0] >= 0 else -q)


def euler_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R_wb = Rz(yaw) Ry(pitch) Rx(roll); world NED, body FRD."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)


def wrap_pi(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)
