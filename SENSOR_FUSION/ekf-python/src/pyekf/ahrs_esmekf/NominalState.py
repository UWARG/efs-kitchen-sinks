import numpy as np
from numpy.typing import NDArray

from pyekf.utils import (
    to_col_vector,
)
from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    normalize_quaternion,
    multiply_quaternions,
)

class NominalState:
    def __init__(
            self,
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,
        ):

        # convention: on initialization, both prev and new are the same
        self.quaternion_prev = normalize_quaternion(to_col_vector(quaternion_initial, 4))
        self.quaternion_new = self.quaternion_prev.copy()

    def __str__(self):
        return (f"Nominal State:\n"
                f"  Quaternion (q): {self.quaternion_new.flatten()} (w,x,y,z)\n"
)

    def state_extrapolation(
            self,
            gyro_new: NDArray[np.float64],
            gyro_prev: NDArray[np.float64],
            dt: np.float64
        ):

        self.quaternion_prev = self.quaternion_new

        self.quaternion_new = self._extrapolate_quaternion(gyro_new, gyro_prev, dt)
    
    def _extrapolate_quaternion(
            self,
            gyro_new: NDArray[np.float64],
            gyro_prev: NDArray[np.float64],
            dt: np.float64        
        ):
        gyro_bar = (gyro_new + gyro_prev) / 2
        omega_matrix = self._exp_omega_matrix(gyro_bar, dt)

        quaternion_new = np.dot(omega_matrix, self.quaternion_prev)

        # Normalize the new quaternion to account for floating point errors
        return quaternion_new / np.linalg.norm(quaternion_new)

    def _exp_omega_matrix(
            self,
            gyro_bar: NDArray[np.float64],
            dt: np.float64
        ):
        norm_gyro = np.linalg.norm(gyro_bar)
        norm_sigma = 0.5 * dt * norm_gyro

        # Handle the case of very small rotation to avoid division by zero and numerical instability
        if norm_gyro < 1e-9:
            # For very small angles, sin(x)/x approaches 1, and cos(x) approaches 1.
            # The matrix becomes identity for zero rotation.
            return np.eye(4)
        
        gx, gy, gz = gyro_bar[0, 0], gyro_bar[1, 0], gyro_bar[2, 0]
        gyro_mult_matrix = np.array((
            [0, -gx, -gy, -gz],
            [gx, 0, gz, -gy],
            [gy, -gz, 0, gx],
            [gz, gy, -gx, 0]
        )) 

        return (np.cos(norm_sigma) * np.eye(4) + np.sin(norm_sigma)/norm_gyro*gyro_mult_matrix)

    def correct_state(
            self,
            small_angle_error: NDArray[np.float64],
        ):
        quaternion_error = np.vstack([
            [1.0],
            0.5 * small_angle_error
        ])
        # TODO: test experimentally if normalize is needed
        quaternion_corrected: NDArray[np.float64] = normalize_quaternion(multiply_quaternions(self.quaternion_new, quaternion_error))
        # quaternion_corrected: NDArray[np.float64] = normalize_quaternion(multiply_quaternions(quaternion_error, self.quaternion_new)) # check this order

        self.quaternion_new = quaternion_corrected
        self.quaternion_prev = quaternion_corrected.copy()
