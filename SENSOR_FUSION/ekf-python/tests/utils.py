import numpy as np
from numpy.typing import NDArray

def assert_quaternion_close(actual: NDArray[np.float64], estimate: NDArray[np.float64], atol: float = 0, rtol: float = 1e-7):
    """
    Asserts that two quaternions represent the same rotation, accounting for the double-cover property (q == -q).
    """
    actual_flat = actual.flatten()
    estimate_flat = estimate.flatten()
    if np.dot(actual_flat, estimate_flat) < 0:
        actual_flat = -actual_flat
    np.testing.assert_allclose(actual_flat.reshape(-1, 1), estimate_flat.reshape(-1, 1), atol=atol, rtol=rtol)
