"""Coordinate frames: byeol internal convention is world NED, body FRD. Converters to/from ENU/FLU (ROS REP-103, ArduPilot-ROS bridges)."""
from __future__ import annotations

import numpy as np

from . import rotation as rot

# C_enu_ned: vector in NED -> the same vector in ENU; C_flu_frd: FRD -> FLU
C_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
C_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def ned_to_enu(v: np.ndarray) -> np.ndarray:
    return C_ENU_NED @ np.asarray(v, float)


def enu_to_ned(v: np.ndarray) -> np.ndarray:
    return C_ENU_NED.T @ np.asarray(v, float)


def frd_to_flu(v: np.ndarray) -> np.ndarray:
    return C_FLU_FRD @ np.asarray(v, float)


def flu_to_frd(v: np.ndarray) -> np.ndarray:
    return C_FLU_FRD.T @ np.asarray(v, float)


def yaw_ned_to_enu(yaw: float) -> float:
    """Heading clockwise from north (NED) -> angle counter-clockwise from east (ENU)."""
    return rot.wrap_pi(np.pi / 2 - yaw)


def yaw_enu_to_ned(yaw: float) -> float:
    return rot.wrap_pi(np.pi / 2 - yaw)


def R_ned_frd_to_enu_flu(R: np.ndarray) -> np.ndarray:
    """Body orientation: R_wb (NED<-FRD) -> (ENU<-FLU)."""
    return C_ENU_NED @ R @ C_FLU_FRD.T


def R_enu_flu_to_ned_frd(R: np.ndarray) -> np.ndarray:
    return C_ENU_NED.T @ R @ C_FLU_FRD
