import numpy as np
from numpy.typing import NDArray

from pyekf.quaternions import multiply_quaternions, inverse_quaternion, normalize_quaternion

def assert_quaternion_close(actual: NDArray[np.float64], estimate: NDArray[np.float64], atol: float = 0, rtol: float = 1e-7):
    """
    Asserts that two quaternions represent the same rotation, accounting for the double-cover property (q == -q).
    """
    actual_flat = actual.flatten()
    estimate_flat = estimate.flatten()
    if np.dot(actual_flat, estimate_flat) < 0:
        actual_flat = -actual_flat
    np.testing.assert_allclose(actual_flat.reshape(-1, 1), estimate_flat.reshape(-1, 1), atol=atol, rtol=rtol)

def attitude_error_vector(q_true: NDArray[np.float64], q_est: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Returns the small angle attitude error (3x1, rad) using the same convention as the filters' error state:
    q_true = q_est (x) dq, with dq ~= [1, 0.5 * small_angle_error].
    """
    q_err = normalize_quaternion(multiply_quaternions(inverse_quaternion(q_est), q_true))
    # q and -q are the same rotation, pick the one with positive scalar part
    if q_err[0, 0] < 0.0:
        q_err = -q_err
    return 2.0 * q_err[1:4, 0:1]

def assert_covariance_valid(P: NDArray[np.float64], tol: float = 1e-9):
    """
    Asserts that a covariance matrix is symmetric and positive semi-definite, which must hold for any working Kalman filter.
    """
    np.testing.assert_allclose(P, P.T, atol=tol, err_msg="Covariance matrix is not symmetric")
    min_eigenvalue = np.min(np.linalg.eigvalsh((P + P.T) / 2.0))
    assert min_eigenvalue >= -tol, f"Covariance matrix is not positive semi-definite, min eigenvalue = {min_eigenvalue}"

def assert_error_within_sigma(error: NDArray[np.float64], P_block: NDArray[np.float64], n_sigma: float = 3.0, name: str = "state"):
    """
    Consistency check: each component of the true error (ground truth - estimate) must lie within n_sigma
    standard deviations predicted by the filter's covariance. A filter that is overconfident (P too small) fails this.
    """
    sigma = np.sqrt(np.clip(np.diag(P_block), 0.0, None)).reshape(-1, 1)
    error = error.reshape(-1, 1)
    assert np.all(np.abs(error) <= n_sigma * sigma), (
        f"{name} error outside {n_sigma}-sigma bound:\n  error = {error.flatten()}\n  sigma = {sigma.flatten()}"
    )
