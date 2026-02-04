import numpy as np
from numpy.typing import NDArray

from pyekf.utils import (
    skew_symmetric,
    normalize_quaternion,
    IDENTITY_QUATERNION,
    GRAVITY_INERTIAL,
    b_to_i_frame_rot_matrix,
    MAGNETOMETER_INERTIAL,
    normalize_vector,
    average_quaternions,
)
from pyekf.NominalState import NominalState
from pyekf.RawMeasurements import RawMeasurements

class ESMEKF:
    def __init__(
            self,
            # Initial Measurements
            gyro_initial: NDArray[np.float64] = np.zeros((3, 1)),
            accel_initial: NDArray[np.float64] = np.zeros((3, 1)),
            mag_initial: NDArray[np.float64] = np.zeros((3, 1)),

            # Initial Nominal State
            displacement_initial: NDArray[np.float64] = np.zeros((3, 1)),
            velocity_initial: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,

            # Initial ESMEKF
            magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,
            gyro_cov: np.float64 = np.float64(0.0),
            accel_cov: np.float64 = np.float64(0.0),
            gyro_bias_cov: np.float64 = np.float64(0.0),
            accel_bias_cov: np.float64 = np.float64(0.0),
            magnetometer_bias_cov: np.float64 = np.float64(0.0),
        ):

        # Raw Measurements
        self.raw_measurements = RawMeasurements(
            gyro_initial=gyro_initial,
            accel_initial=accel_initial,
            mag_initial=mag_initial,
        )

        # Nominal State
        self.nominal_state = NominalState(
            displacement_initial=displacement_initial,
            velocity_initial=velocity_initial,
            quaternion_initial=quaternion_initial,
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
            gyro_new: NDArray[np.float64],
            accel_new: NDArray[np.float64],
            dt: np.float64
        ):

        self.raw_measurements.update_gyro(gyro_new)
        self.raw_measurements.update_accel(accel_new)

        self.nominal_state.update(
            gyro_new=self.raw_measurements.gyro_new,
            gyro_prev=self.raw_measurements.gyro_prev,
            accel_new=self.raw_measurements.accel_new,
            accel_prev=self.raw_measurements.accel_prev,
            dt=dt
        )

        state_transition_matrix = self._state_transition_matrix(dt)

    def _error_state_gradient_matrix_F(self):
        # non-zero submatrices of F
        omega_matrix = -skew_symmetric(self.raw_measurements.gyro_bar)
        accel_matrix = -0.5 * (
            np.dot(b_to_i_frame_rot_matrix(self.nominal_state.quaternion_new), self.raw_measurements.accel_new)
            +
            np.dot(b_to_i_frame_rot_matrix(self.nominal_state.quaternion_prev), self.raw_measurements.accel_prev)
        )
        change_of_basis_matrix = -b_to_i_frame_rot_matrix(average_quaternions(self.nominal_state.quaternion_new, self.nominal_state.quaternion_prev))


        F = np.zeros(shape=(18, 18), dtype=float)
        # TODO: fill in F with the correct submatrices

        return F


    # can make this I + F*dt + 1/2 F^2*dt^2 for better accuracy
    def _state_transition_matrix(self, dt: np.float64):
        return np.eye(18, dtype=float) + dt * self._error_state_gradient_matrix_F()

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
