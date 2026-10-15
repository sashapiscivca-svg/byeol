import numpy as np

from byeol import rotation as rot


def test_rotvec_matches_axis_rotation():
    R = rot.rotvec_to_R(np.array([0.0, 0.0, np.pi / 2]))
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_quat_roundtrip_and_composition():
    a, b = np.array([0.1, -0.2, 0.3]), np.array([0.4, 0.1, -0.2])
    qa, qb = rot.q_exp(a), rot.q_exp(b)
    Rab = rot.q_to_R(rot.q_mul(qa, qb))
    assert np.allclose(Rab, rot.rotvec_to_R(a) @ rot.rotvec_to_R(b), atol=1e-12)
    assert np.allclose(rot.q_log(qa), a, atol=1e-12)


def test_small_angle_exp_is_stable():
    q = rot.q_exp(np.array([1e-12, 0, 0]))
    assert np.isclose(np.linalg.norm(q), 1.0) and q[0] > 0.999


def test_euler_roundtrip():
    R = rot.euler_to_R(0.2, -0.3, 1.0)
    r, p, y = rot.R_to_euler(R)
    assert np.allclose([r, p, y], [0.2, -0.3, 1.0], atol=1e-12)


def test_nose_up_pitch_raises_forward_axis():
    R = rot.euler_to_R(0.0, 0.3, 0.0)  # FRD, NED: nose up => forward axis has an "up" component (z<0)
    assert (R @ [1, 0, 0])[2] < 0


def test_q_conj_inverse():
    q = rot.q_exp(np.array([0.3, 0.2, 0.1]))
    assert np.allclose(rot.q_to_R(rot.q_mul(q, rot.q_conj(q))), np.eye(3), atol=1e-12)
