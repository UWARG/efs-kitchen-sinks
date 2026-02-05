import numpy as np
from numpy.typing import NDArray

from pyekf.utils import (
    skew_symmetric,
    GRAVITY_INERTIAL,
)
from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    normalize_quaternion,
    b_to_i_frame_rot_matrix,
)

class NominalState:
    def __init__(
            self,
            displacement_initial: NDArray[np.float64] = np.zeros((3, 1)),
            velocity_initial: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
        ):

        # convention: on initialization, both prev and new are the same
        self.displacement_prev = np.asarray(displacement_initial, dtype=float).reshape(3, 1)
        self.velocity_prev = np.asarray(velocity_initial, dtype=float).reshape(3, 1)
        self.quaternion_prev = normalize_quaternion(np.asarray(quaternion_initial, dtype=float)).reshape(4, 1)
        self.displacement_new = self.displacement_prev.copy()
        self.velocity_new = self.velocity_prev.copy()
        self.quaternion_new = self.quaternion_prev.copy()

        self.gravity_inertial = np.asarray(gravity_inertial, dtype=float).reshape(3, 1)

    def __str__(self):
        return (f"Nominal State:\n"
                f"  Displacement (p):   {self.displacement_new.flatten()} m\n"
                f"  Velocity (v):   {self.velocity_new.flatten()} m/s\n"
                f"  Quaternion (q): {self.quaternion_new.flatten()} (w,x,y,z)\n"
)

    def state_extrapolation(
            self,
            gyro_new: NDArray[np.float64],
            gyro_prev: NDArray[np.float64],
            accel_new: NDArray[np.float64],
            accel_prev: NDArray[np.float64],
            dt: np.float64
        ):

        self.quaternion_prev = self.quaternion_new
        self.velocity_prev = self.velocity_new
        self.displacement_prev = self.displacement_new

        self.quaternion_new = self._extrapolate_quaternion(gyro_new, gyro_prev, dt)
        self.velocity_new = self._extrapolate_velocity(accel_new, accel_prev, dt)
        self.displacement_new = self._extrapolate_displacement(dt)
    
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
    
    def _extrapolate_velocity(
            self,
            accel_body_new: NDArray[np.float64],
            accel_body_prev: NDArray[np.float64],
            dt: np.float64
        ):

        accel_inertial_new = np.dot(b_to_i_frame_rot_matrix(self.quaternion_new), accel_body_new)
        accel_inertial_old = np.dot(b_to_i_frame_rot_matrix(self.quaternion_prev), accel_body_prev)
        accel_bar = (accel_inertial_new + accel_inertial_old) / 2
        return (accel_bar + self.gravity_inertial) * dt + self.velocity_prev

    def _extrapolate_displacement(
            self,
            dt: np.float64
        ):

        velocity_bar = (self.velocity_new + self.velocity_prev) / 2
        return velocity_bar * dt + self.displacement_prev

    # TODO: implement
    def correct_state(
            self,
            corrected_error_state: NDArray[np.float64],
        ):
        pass