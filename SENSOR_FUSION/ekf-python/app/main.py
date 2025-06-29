import numpy as np
from pyquaternion import Quaternion
from numpy.typing import NDArray
from scipy.linalg import expm

def skew_symmetric(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Create a skew-symmetric matrix [v]_x for a vector v.

    Parameters:
    v (np.ndarray): Input vector of shape (3,)

    Returns:
    np.ndarray: Skew-symmetric matrix of shape (3,3)
    """
    if not isinstance(v, np.ndarray):
        raise TypeError("Input must be a numpy ndarray")
    if v.shape != (3,):
        raise ValueError(f"Input vector must have shape (3,), got {v.shape}")
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def main():
    print("Hello world!")


if __name__ == "__main__":
    main()
