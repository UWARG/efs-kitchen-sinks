import numpy as np
from numpy.typing import NDArray

from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL, to_col_vector
from pyekf.quaternions import rotate_vector, inverse_quaternion
from tests.sim.trajectory import Trajectory

class SensorSimulator:
    """
    Simulates IMU and Magnetometer readings by transforming inertial trajectory states 
    into the body frame and applying biases and additive Gaussian noise.
    """

    def __init__(
        self, 
        trajectory: Trajectory,

        # Sensor random noise variances (diagonal covariance matrices)
        gyro_cov: float = 0,               # (rad/s)^2
        accel_cov: float = 0,              # (m/s^2)^2
        magnetometer_cov: float = 0,       # unitless (normalized field)

        # Sensor biases (constant offsets)
        gyro_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        accel_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),
        magnetometer_bias: NDArray[np.float64] = np.zeros((3, 1), dtype=np.float64),

        # Inertial reference values
        gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
        magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,

        # Random seed for reproducibility of noise
        seed: int = None
    ):
        self.traj: Trajectory = trajectory
        
        # Initialize Random Number Generator for deterministic noise
        self.seed: int = seed if seed is not None else int(np.random.randint(0, np.iinfo(np.int32).max))
        self.rng = np.random.default_rng(self.seed)

        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3) * gyro_cov
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3) * accel_cov
        self.magnetometer_cov_mat: NDArray[np.float64] = np.eye(3) * magnetometer_cov
        
        self.gyro_bias: NDArray[np.float64] = to_col_vector(gyro_bias, 3)
        self.accel_bias: NDArray[np.float64] = to_col_vector(accel_bias, 3)
        self.magnetometer_bias: NDArray[np.float64] = to_col_vector(magnetometer_bias, 3)

        self.g_i: NDArray[np.float64] = to_col_vector(gravity_inertial, 3)
        self.m_i: NDArray[np.float64] = to_col_vector(magnetometer_inertial, 3)

    def get_readings(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Returns simulated sensor readings at time t. All sensor readings are in the body frame and include bias and noise.
        - Gyroscope (rad/s)
        - Accelerometer (m/s^2)
        - Magnetometer (unitless)
        """
        # Retrieve ground truth states from analytical trajectory, in iframe at time t.
        disp_i, vel_i, accel_i, quat_i, angular_vel_i = self.traj.get_state(t)
        quat_inv = inverse_quaternion(quat_i)

        # 1. Gyroscope: Rotate inertial angular velocity to body frame and add bias
        gyro_reading = rotate_vector(angular_vel_i, quat_inv) + self.gyro_bias
        
        # 2. Accelerometer: Specific force in body frame f = R(quat_inv) * (a_i - g_i)
        accel_total_inertial = accel_i - self.g_i
        accel_reading = rotate_vector(accel_total_inertial, quat_inv) + self.accel_bias

        # 3. Magnetometer: Rotate inertial field to body frame and add bias
        mag_reading = rotate_vector(self.m_i, quat_inv) + self.magnetometer_bias

        # Add Gaussian noise if covariance is non-zero
        if np.any(self.gyro_cov_mat):
            gyro_reading += to_col_vector(self.rng.multivariate_normal(np.zeros(3), self.gyro_cov_mat), 3)
        if np.any(self.accel_cov_mat):
            accel_reading += to_col_vector(self.rng.multivariate_normal(np.zeros(3), self.accel_cov_mat), 3)
        if np.any(self.magnetometer_cov_mat):
            mag_reading += to_col_vector(self.rng.multivariate_normal(np.zeros(3), self.magnetometer_cov_mat), 3)

        return gyro_reading, accel_reading, mag_reading
