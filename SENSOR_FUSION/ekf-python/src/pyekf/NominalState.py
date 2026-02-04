import numpy as np
from numpy.typing import NDArray
from pyekf.utils import skew_symmetric, normalize_quaternion, IDENTITY_QUATERNION, GRAVITY_INERTIAL, b_to_i_frame_rot_matrix

class NominalState:
    def __init__(
            self,
            displacement_initial: NDArray[np.float64] = np.zeros((3, 1)),
            velocity_initial: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
        ):

        self.prev_displacement = np.asarray(displacement_initial, dtype=float).reshape(3, 1)
        self.prev_velocity = np.asarray(velocity_initial, dtype=float).reshape(3, 1)
        self.prev_quaternion = normalize_quaternion(np.asarray(quaternion_initial, dtype=float)).reshape(4, 1)
        self.gravity_inertial = np.asarray(gravity_inertial, dtype=float).reshape(3, 1)

    def __str__(self):
        return (f"Nominal State:\n"
                f"  Displacement (p):   {self.prev_displacement.flatten()} m\n"
                f"  Velocity (v):   {self.prev_velocity.flatten()} m/s\n"
                f"  Quaternion (q): {self.prev_quaternion.flatten()} (w,x,y,z)\n"
)

    def update(
            self,
            gyro_new: NDArray[np.float64],
            gyro_prev: NDArray[np.float64],
            accel_new: NDArray[np.float64],
            accel_prev: NDArray[np.float64],
            dt: np.float64
        ):

        quaternion_new = self._update_quaternion(gyro_new, gyro_prev, dt)
        velocity_new = self._update_velocity(quaternion_new, accel_new, accel_prev, dt)
        displacement_new = self._update_displacement(velocity_new, dt)

        self.prev_quaternion = quaternion_new
        self.prev_velocity = velocity_new
        self.prev_displacement = displacement_new
    
    def _update_quaternion(
            self,
            gyro_new: NDArray[np.float64],
            gyro_prev: NDArray[np.float64],
            dt: np.float64        
        ):
        gyro_bar = (gyro_new + gyro_prev) / 2
        omega_matrix = self._exp_omega_matrix(gyro_bar, dt)

        quaternion_new = np.dot(omega_matrix, self.prev_quaternion)

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
    
    def _update_velocity(
            self,
            quaternion_new: NDArray[np.float64],
            accel_body_new: NDArray[np.float64],
            accel_body_prev: NDArray[np.float64],
            dt: np.float64
        ):

        accel_inertial_new = np.dot(b_to_i_frame_rot_matrix(quaternion_new), accel_body_new)
        accel_inertial_old = np.dot(b_to_i_frame_rot_matrix(self.prev_quaternion), accel_body_prev)
        accel_bar = (accel_inertial_new + accel_inertial_old) / 2
        return (accel_bar + self.gravity_inertial) * dt + self.prev_velocity

    def _update_displacement(
            self,
            velocity_new: NDArray[np.float64],
            dt: np.float64    
        ):

        velocity_bar = (velocity_new + self.prev_velocity) / 2
        return velocity_bar * dt + self.prev_displacement
