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

        # Sensor bias variances (diagonal covariance matrices to pick constant offsets)
        gyro_bias_cov: float = 0,          # (rad/s)^2
        accel_bias_cov: float = 0,         # (m/s^2)^2
        magnetometer_bias_cov: float = 0,  # unitless (normalized field)

        # GPS random noise variances (diagonal covariance matrices), no bias
        gps_position_cov: float = 0,       # m^2
        gps_velocity_cov: float = 0,       # (m/s)^2

        # Inertial reference values
        gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
        magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,

        # Normalize the magnetometer reading to a unit vector (AHRS uses the field direction only, INS uses the raw field)
        normalize_magnetometer: bool = True,

        # Random seed for reproducibility of noise
        seed: int = None
    ):
        self.traj: Trajectory = trajectory
        self.normalize_magnetometer: bool = normalize_magnetometer
        
        # Initialize Random Number Generator for deterministic noise
        self.seed: int = seed if seed is not None else int(np.random.randint(0, np.iinfo(np.int32).max))
        self.rng = np.random.default_rng(self.seed)
        self.gps_rng = np.random.default_rng(self.seed + 1)

        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3) * gyro_cov
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3) * accel_cov
        self.magnetometer_cov_mat: NDArray[np.float64] = np.eye(3) * magnetometer_cov
        self.gps_position_cov_mat: NDArray[np.float64] = np.eye(3) * gps_position_cov
        self.gps_velocity_cov_mat: NDArray[np.float64] = np.eye(3) * gps_velocity_cov
        
        # Initialize bias covariance matrices and pick constant offsets
        gyro_bias_cov_mat = np.eye(3) * gyro_bias_cov
        accel_bias_cov_mat = np.eye(3) * accel_bias_cov
        mag_bias_cov_mat = np.eye(3) * magnetometer_bias_cov

        # TODO: no drift in biases, maybe add drift or guassian random noise
        self.gyro_bias: NDArray[np.float64] = to_col_vector(self.rng.multivariate_normal(np.zeros(3), gyro_bias_cov_mat), 3)
        self.accel_bias: NDArray[np.float64] = to_col_vector(self.rng.multivariate_normal(np.zeros(3), accel_bias_cov_mat), 3)
        self.magnetometer_bias: NDArray[np.float64] = to_col_vector(self.rng.multivariate_normal(np.zeros(3), mag_bias_cov_mat), 3)

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
            mag_norm = np.linalg.norm(mag_reading)
            if self.normalize_magnetometer and mag_norm > 1e-9:
                mag_reading /= mag_norm

        return gyro_reading, accel_reading, mag_reading

    def get_gps_readings(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """
        Returns simulated GPS readings at time t, in the inertial frame with additive Gaussian noise.
        Uses a separate random number generator so adding GPS to a test does not change the IMU/Mag noise.
        - Position (m)
        - Velocity (m/s)
        """
        disp_i, vel_i, _, _, _ = self.traj.get_state(t)
        gps_position = disp_i.copy()
        gps_velocity = vel_i.copy()

        if np.any(self.gps_position_cov_mat):
            gps_position += to_col_vector(self.gps_rng.multivariate_normal(np.zeros(3), self.gps_position_cov_mat), 3)
        if np.any(self.gps_velocity_cov_mat):
            gps_velocity += to_col_vector(self.gps_rng.multivariate_normal(np.zeros(3), self.gps_velocity_cov_mat), 3)

        return gps_position, gps_velocity
