import numpy as np
from numpy.typing import NDArray

IDENTITY_QUATERNION = np.array([[1.0], [0.0], [0.0], [0.0]], dtype=np.float64)

def normalize_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        # Handle very small quaternions (e.g., zero vector)
        return IDENTITY_QUATERNION
    return q / norm

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
