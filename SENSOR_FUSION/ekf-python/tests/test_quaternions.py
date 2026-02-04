import numpy as np
import pytest

from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    multiply_quaternions,
    inverse_quaternion,
    normalize_quaternion,
    average_quaternions,
    b_to_i_frame_rot_matrix,
)

ATOL = 1e-9


def quat_close(q1, q2, atol=ATOL):
    """Quaternion equality up to sign."""
    return (
        np.allclose(q1, q2, atol=atol)
        or np.allclose(q1, -q2, atol=atol)
    )


def test_identity_quaternion():
    q = IDENTITY_QUATERNION.flatten()
    qn = normalize_quaternion(q)
    assert np.allclose(qn, q)


def test_quaternion_multiply_identity():
    q = np.array([0.9238795, 0.3826834, 0.0, 0.0])
    result = multiply_quaternions(q, IDENTITY_QUATERNION.flatten())
    assert np.allclose(result, q, atol=ATOL)


def test_quaternion_inverse():
    q = np.array([0.9238795, 0.3826834, 0.0, 0.0])
    q_inv = inverse_quaternion(q)
    prod = multiply_quaternions(q, q_inv)
    assert quat_close(prod, IDENTITY_QUATERNION.flatten())


def test_normalize_quaternion_unit_norm():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    qn = normalize_quaternion(q)
    assert np.isclose(np.linalg.norm(qn), 1.0)


def test_normalize_zero_quaternion():
    q = np.zeros(4)
    qn = normalize_quaternion(q)
    assert np.allclose(qn, IDENTITY_QUATERNION)


def test_average_identical_quaternions():
    q = np.array([0.7071068, 0.7071068, 0.0, 0.0])
    q_avg = average_quaternions(q, q)
    assert quat_close(q_avg, normalize_quaternion(q))


def test_average_with_identity():
    q = np.array([0.7071068, 0.7071068, 0.0, 0.0])
    q_avg = average_quaternions(IDENTITY_QUATERNION.flatten(), q)

    # Result should be halfway rotation
    expected_angle = np.pi / 4
    expected = np.array([
        np.cos(expected_angle / 2),
        np.sin(expected_angle / 2),
        0.0,
        0.0,
    ])

    assert quat_close(q_avg, expected)


def test_average_shortest_path():
    q1 = np.array([1.0, 0.0, 0.0, 0.0])
    q2 = -q1  # same rotation, opposite sign
    q_avg = average_quaternions(q1, q2)
    assert quat_close(q_avg, q1)


def test_rotation_matrix_identity():
    q = IDENTITY_QUATERNION
    C = b_to_i_frame_rot_matrix(q)
    assert np.allclose(C, np.eye(3), atol=ATOL)


def test_rotation_matrix_orthonormal():
    q = np.array([[0.9238795], [0.3826834], [0.0], [0.0]])
    C = b_to_i_frame_rot_matrix(q)

    I = C @ C.T
    assert np.allclose(I, np.eye(3), atol=ATOL)
    assert np.isclose(np.linalg.det(C), 1.0, atol=ATOL)


def test_rotation_matrix_known_rotation():
    # 90 deg rotation about X
    q = np.array([
        [np.cos(np.pi / 4)],
        [np.sin(np.pi / 4)],
        [0.0],
        [0.0],
    ])
    C = b_to_i_frame_rot_matrix(q)

    expected = np.array([
        [1, 0,  0],
        [0, 0, -1],
        [0, 1,  0],
    ])

    assert np.allclose(C, expected, atol=ATOL)
