import numpy as np
from numpy.typing import NDArray

from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL, to_col_vector
from pyekf.quaternions import rotate_vector, inverse_quaternion

class SensorSimulator:
    def __init__(
        self, 
        trajectory: object, 
        gyro_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        accel_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64)
    ):
        self.traj = trajectory
        
        self.gyro_bias = to_col_vector(gyro_bias, 3)
        self.accel_bias = to_col_vector(accel_bias, 3)

        self.g_i = GRAVITY_INERTIAL
        self.m_i = MAGNETOMETER_INERTIAL

    def get_readings(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        p, v, q = self.traj.get_state(t)
        
        # Gyroscope measurement model: y = omega + bias
        gyro_reading = to_col_vector(self.traj.w_b, 3) + self.gyro_bias
        
        # Inertial to Body rotation requires the inverse (conjugate) quaternion
        q_inv = inverse_quaternion(q)
        
        # Accelerometer measures specific force: f = R_i_to_b * (a_inertial - g_inertial)
        # Note: If a_inertial is 0, reading is -g rotated into body frame
        accel_total_inertial = to_col_vector(self.traj.a_b, 3) - self.g_i
        accel_reading = rotate_vector(accel_total_inertial, q_inv) + self.accel_bias

        # Magnetometer measurement model: m_body = R_i_to_b * m_inertial
        mag_reading = rotate_vector(self.m_i, q_inv)
        
        return gyro_reading, accel_reading, mag_reading
