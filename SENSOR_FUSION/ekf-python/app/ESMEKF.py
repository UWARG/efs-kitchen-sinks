import numpy as np
from numpy.typing import NDArray
from app.utils import (
    skew_symmetric,
    normalize_quaternion,
    IDENTITY_QUATERNION,
    GRAVITY_INERTIAL,
    b_to_i_frame_rot_matrix,
    MAGNETOMETER_INERTIAL,
    normalize_vector
)
from app.NominalState import NominalState

class ESMEKF:
    def __init__(
            self,
            displacement: NDArray[np.float64] = np.zeros((3, 1)),
            velocity: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion: NDArray[np.float64] = IDENTITY_QUATERNION,
            initial_gyro_measurement: NDArray[np.float64] = np.zeros((3, 1)),
            initial_accel_measurement: NDArray[np.float64] = np.zeros((3, 1)),
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
            magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,
            gyro_cov: np.float64 = np.float64(0.0),
            accel_cov: np.float64 = np.float64(0.0),
            gyro_bias_cov: np.float64 = np.float64(0.0),
            accel_bias_cov: np.float64 = np.float64(0.0),
            magnetometer_bias_cov: np.float64 = np.float64(0.0),
        ):

        # Nominal State
        self.nominal_state = NominalState(
            displacement=displacement,
            velocity=velocity,
            quaternion=quaternion,
            prev_gyro_measurement=initial_gyro_measurement,
            prev_accel_measurement=initial_accel_measurement,
            gravity_inertial=gravity_inertial
        )

        # magnetometer WMM inertial vector
        self.magnetometer_inertial = np.asarray(normalize_vector(magnetometer_inertial), dtype=float).reshape(3, 1)

        # 18 error states
        self.small_angle_error: NDArray[np.float64] = np.zeros((3, 1))
        self.velocity_error: NDArray[np.float64] = np.zeros((3, 1))
        self.displacement_error: NDArray[np.float64] = np.zeros((3, 1))
        self.gyro_bias: NDArray[np.float64] = np.zeros((3, 1))
        self.accelerometer_bias: NDArray[np.float64] = np.zeros((3, 1))
        self.magnetometer_bias: NDArray[np.float64] = np.zeros((3, 1))

        # Sensor covariance tuning params
        # Assuming assuming identical independent covariance across x,y,z axises
        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_cov)
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_cov)
        self.gyro_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_bias_cov)
        self.accel_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_bias_cov)
        self.magnetometer_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(magnetometer_bias_cov)


    def __str__(self):
        return (
            "ESMEKF Internal State:\n"
            f"  Small Angle Error:  {self.small_angle_error.flatten()}\n"
            f"  Velocity Error: {self.velocity_error.flatten()}\n"
            f"  Displacement Error: {self.displacement_error.flatten()}\n"
            f"  Gyro Bias:  {self.gyro_bias.flatten()}\n"
            f"  Accelerometer Bias: {self.accelerometer_bias.flatten()}\n"
            f"  Magnetometer Bias:  {self.magnetometer_bias.flatten()}\n\n"
            f"  WMM Inertial Magnetometer Vec:  {self.magnetometer_inertial.flatten()}\n\n"
            f"{self.nominal_state}"
        )
    
    def predict(
            self,
            gyro_measurement_new: NDArray[np.float64],
            accel_measurement_new: NDArray[np.float64],
            dt: np.float64
        ):

        gyro_measurement_new = np.asarray(gyro_measurement_new, dtype=float).reshape(3, 1)
        accel_measurement_new = np.asarray(accel_measurement_new, dtype=float).reshape(3, 1)

        gyro_measurement_prev: NDArray[np.float64] = self.nominal_state.prev_gyro_measurement
        accel_measurement_prev: NDArray[np.float64] = self.nominal_state.prev_accel_measurement

        self.nominal_state.update(
            gyro_measurement=gyro_measurement_new,
            accel_measurement=accel_measurement_new,
            dt=dt
        )

        # only computing non-zero terms
        gyro_bar: NDArray[np.float64] = (gyro_measurement_new + gyro_measurement_prev) / 2

    # make I + dt * F
    def _state_transition_matrix(self):
        pass

    def _process_noise_cov_matrix(self, dt):
        Q = np.zeros(shape=(18, 18), dtype=float)
        Q[0:3, 0:3] = self.gyro_cov_mat*dt + self.gyro_bias_cov_mat*(dt**3)/3.0
        Q[0:3, 9:12] = -self.gyro_bias_cov_mat*(dt**2)/2.0
        Q[3:6, 3:6] = self.accel_cov_mat*dt + self.accel_bias_cov_mat*(dt**3)/3.0
        Q[3:6, 6:9] = self.accel_bias_cov_mat*(dt**4)/8.0 + self.accel_cov_mat*(dt**2)/2.0
        Q[3:6, 12:15] = -self.accel_bias_cov_mat*(dt**2)/2.0
        Q[6:9, 3:6] = self.accel_cov_mat*(dt**2)/2.0 + self.accel_bias_cov_mat*(dt**4)/8.0
        Q[6:9, 6:9] = self.accel_cov_mat*(dt**3)/3.0 + self.accel_bias_cov_mat*(dt**5)/20.0
        Q[6:9, 12:15] = -self.accel_bias_cov_mat*(dt**3)/6.0
        Q[9:12, 0:3] = -self.gyro_bias_cov_mat*(dt**2)/2.0
        Q[9:12, 9:12] = self.gyro_bias_cov_mat*dt
        Q[12:15, 3:6] = -self.accel_bias_cov_mat*(dt**2)/2.0
        Q[12:15, 6:9] = -self.accel_bias_cov_mat*(dt**3)/6.0
        Q[12:15, 12:15] = self.accel_bias_cov_mat*dt
        Q[15:18, 15:18] = self.magnetometer_bias_cov_mat*dt

        return Q

    def correct(self):
        pass
