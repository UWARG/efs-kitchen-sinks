import numpy as np
from numpy.typing import NDArray

IDENTITY_QUATERNION = np.array([[1.0], [0.0], [0.0], [0.0]], dtype=np.float64)
GRAVITY_INERTIAL = np.array([[0.0], [0.0], [9.81]], dtype=np.float64)
MAGNETOMETER_INERTIAL = np.array([[1.0], [0.0], [0.0]], dtype=np.float64)

def normalize_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        # Handle very small quaternions (e.g., zero vector)
        return IDENTITY_QUATERNION
    return q / norm

def normalize_vector(v: NDArray[np.float64]) -> NDArray[np.float64]:
    norm = np.linalg.norm(v)
    if norm == 0:
        raise ValueError("Cannot normalize zero vector")
    return v / norm

def skew_symmetric(v: NDArray[np.float64]) -> NDArray[np.float64]:
    if not isinstance(v, np.ndarray):
        raise TypeError("Input must be a numpy ndarray")
    if v.shape != (3, 1):
        raise ValueError(f"Input vector must have shape (3, 1), got {v.shape}")
    return np.array([
        [0, -v[2, 0], v[1, 0]],
        [v[2, 0], 0, -v[0, 0]],
        [-v[1, 0], v[0, 0], 0]
    ], dtype=np.float64)

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
