import numpy as np
import pytest

from byeol import frames as fr
from byeol import rotation as rot


def test_ned_enu_vector_roundtrip_and_values():
    v = np.array([1.0, 2.0, 3.0])  # north, east, down
    e = fr.ned_to_enu(v)
    assert np.allclose(e, [2.0, 1.0, -3.0])
    assert np.allclose(fr.enu_to_ned(e), v)


def test_frd_flu():
    assert np.allclose(fr.frd_to_flu(np.array([1.0, 2.0, 3.0])), [1.0, -2.0, -3.0])
    assert np.allclose(fr.flu_to_frd(fr.frd_to_flu(np.array([1.0, 2.0, 3.0]))), [1.0, 2.0, 3.0])


def test_yaw_conversion():
    # heading 0 (north) in NED = 90 degrees (north) in ENU
    assert fr.yaw_ned_to_enu(0.0) == pytest.approx(np.pi / 2)
    assert fr.yaw_enu_to_ned(fr.yaw_ned_to_enu(0.7)) == pytest.approx(0.7)
    assert fr.yaw_ned_to_enu(np.pi / 2) == pytest.approx(0.0)  # east = 0 in ENU


def test_attitude_conversion_consistent_with_vectors():
    R_ned_frd = rot.euler_to_R(0.1, -0.2, 0.9)
    R_enu_flu = fr.R_ned_frd_to_enu_flu(R_ned_frd)
    b_frd = np.array([1.0, 0.5, -0.3])
    # the same physical body vector expressed in world frame in two ways
    assert np.allclose(R_enu_flu @ fr.frd_to_flu(b_frd), fr.ned_to_enu(R_ned_frd @ b_frd))
    assert np.allclose(fr.R_enu_flu_to_ned_frd(R_enu_flu), R_ned_frd)


def test_yaw_matches_attitude_conversion():
    R = rot.euler_to_R(0.0, 0.0, 0.7)
    yaw_enu = rot.R_to_euler(fr.R_ned_frd_to_enu_flu(R))[2]
    assert yaw_enu == pytest.approx(fr.yaw_ned_to_enu(0.7))
