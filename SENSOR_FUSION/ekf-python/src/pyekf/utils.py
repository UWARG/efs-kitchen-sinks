import numpy as np
from numpy.typing import NDArray

GRAVITY_INERTIAL = np.array([[0.0], [0.0], [9.81]], dtype=np.float64)
MAGNETOMETER_INERTIAL = np.array([[1.0], [0.0], [0.0]], dtype=np.float64)

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

def to_col_vector(v: NDArray[np.float64], cols: int) -> NDArray[np.float64]:
    v = np.asarray(v, dtype=float)
    v_flat = v.flatten()
    if v_flat.size != cols:
        raise ValueError(f"Expected vector with {cols} elements, got shape {v.shape}")
    return v_flat.reshape(cols, 1)

def ensure_symmetric_matrix(M: NDArray[np.float64]) -> NDArray[np.float64]:
    return (M + M.T) / 2.0
