import numpy as np
from numpy.typing import NDArray

IDENTITY_QUATERNION = np.array([[1.0], [0.0], [0.0], [0.0]], dtype=np.float64)

def multiply_quaternions(q1: NDArray[np.float64], q2: NDArray[np.float64]) -> NDArray[np.float64]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)

def inverse_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=np.float64) / np.dot(q, q)

def normalize_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        # Handle very small quaternions (e.g., zero vector)
        return IDENTITY_QUATERNION
    return q / norm

def average_quaternions(q1: NDArray[np.float64], q2: NDArray[np.float64]) -> NDArray[np.float64]:
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)

    # Relative rotation: r = q1^{-1} x q2
    r = multiply_quaternions(inverse_quaternion(q1), q2)

    # Ensure shortest path (important for averaging)
    if r[0] < 0.0:
        r = -r

    r0 = np.clip(r[0], -1.0, 1.0)
    rv = r[1:]

    # ||mu|| = 2 * acos(rtheta)
    mu_norm = 2.0 * np.arccos(r0)

    # Small-angle handling
    if mu_norm < 1e-12:
        return q1.copy()

    # mu = r_vec * (||mu|| / sin(||mu||/2))
    sin_half = np.sin(mu_norm / 2.0)
    mu = rv * (mu_norm / sin_half)

    # Half rotation: ||mu|| / 2
    half_norm = mu_norm / 2.0

    # r_n = [cos(||mu||/4), (mu/||mu||) sin(||mu||/4)]
    axis = mu / mu_norm
    r_n = np.empty(4, dtype=np.float64)
    r_n[0] = np.cos(half_norm / 2.0)
    r_n[1:] = axis * np.sin(half_norm / 2.0)

    # Average quaternion = q1 x r_n
    q_avg = multiply_quaternions(q1, r_n)
    return q_avg / np.linalg.norm(q_avg)

def b_to_i_frame_rot_matrix(q: NDArray[np.float64]):
    q = normalize_quaternion(q) # Ensure it's normalized
    w, x, y, z = q[0, 0], q[1, 0], q[2, 0], q[3, 0]

    # Rotation matrix from body frame to inertial frame (C_b^i)
    C = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])
    return C

def i_to_b_frame_rot_matrix(q: NDArray[np.float64]):
    q_inv = inverse_quaternion(q)
    return b_to_i_frame_rot_matrix(q_inv)