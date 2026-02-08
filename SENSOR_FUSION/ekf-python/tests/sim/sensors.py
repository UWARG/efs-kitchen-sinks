import numpy as np
from numpy.typing import NDArray

from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL, to_col_vector
from pyekf.quaternions import rotate_vector, inverse_quaternion

class SensorSimulator:
    def __init__(
        self, 
        trajectory: object,
        gyro_cov: float = 0,
        accel_cov: float = 0,
        magnetometer_cov: float = 0,
        gyro_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        accel_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        magnetometer_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
        magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,
        seed: int = None
    ):
        self.traj = trajectory

        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3) * gyro_cov if gyro_cov > 0 else np.zeros((3, 3))
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3) * accel_cov if accel_cov > 0 else np.zeros((3, 3))
        self.magnetometer_cov_mat: NDArray[np.float64] = np.eye(3) * magnetometer_cov if magnetometer_cov > 0 else np.zeros((3, 3))
        
        self.gyro_bias: NDArray[np.float64] = to_col_vector(gyro_bias, 3)
        self.accel_bias: NDArray[np.float64] = to_col_vector(accel_bias, 3)
        self.magnetometer_bias: NDArray[np.float64] = to_col_vector(magnetometer_bias, 3)

        self.g_i: NDArray[np.float64] = to_col_vector(gravity_inertial, 3)
        self.m_i: NDArray[np.float64] = to_col_vector(magnetometer_inertial, 3)

    def get_readings(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        p, v, q = self.traj.get_state(t)
        
        # Gyroscope measurement model: y = omega + bias
        gyro_reading = to_col_vector(self.traj.angular_vel_body, 3) + self.gyro_bias
        
        # Inertial to Body rotation requires the inverse (conjugate) quaternion
        q_inv = inverse_quaternion(q)
        
        # Accelerometer measures specific force: f = R_i_to_b * (a_inertial - g_inertial)
        # Note: If a_inertial is 0, reading is -g rotated into body frame
        accel_total_inertial = to_col_vector(self.traj.accel_body, 3) - self.g_i
        accel_reading = rotate_vector(accel_total_inertial, q_inv) + self.accel_bias

        # Magnetometer measurement model: m_body = R_i_to_b * m_inertial
        mag_reading = rotate_vector(self.m_i, q_inv)
        
        return gyro_reading, accel_reading, mag_reading
