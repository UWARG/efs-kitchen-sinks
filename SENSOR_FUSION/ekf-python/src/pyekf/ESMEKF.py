import numpy as np
from numpy.typing import NDArray

from pyekf.utils import (
    skew_symmetric,
    GRAVITY_INERTIAL,
    MAGNETOMETER_INERTIAL,
    normalize_vector,
)
from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    normalize_quaternion,
    average_quaternions,
    b_to_i_frame_rot_matrix,
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

        # Sensor covariance tuning params
        # Assuming assuming identical independent covariance across x,y,z axises
        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_cov)
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_cov)
        self.gyro_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_bias_cov)
        self.accel_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_bias_cov)
        self.magnetometer_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(magnetometer_bias_cov)

        # EKF
        # small_angle_error = self.error_state[0:3]
        # velocity_error = self.error_state[3:6]
        # displacement_error = self.error_state[6:9]
        # gyro_bias = self.error_state[9:12]
        # accelerometer_bias = self.error_state[12:15]
        # magnetometer_bias = self.error_state[15:18]
        self.error_state: NDArray[np.float64] = np.zeros((18, 1), dtype=float)
        self.error_state_cov_mat: NDArray[np.float64] = np.zeros((18, 18), dtype=float)
        self.kalman_gain: NDArray[np.float64] = np.zeros((18, 3), dtype=float)

        # magnetometer WMM inertial vector
        self.magnetometer_inertial = np.asarray(normalize_vector(magnetometer_inertial), dtype=float).reshape(3, 1)


    def __str__(self):
        return (
            "ESMEKF Internal State:\n"
            f"  Small Angle Error:  {self.error_state[0:3].flatten()}\n"
            f"  Velocity Error: {self.error_state[3:6].flatten()}\n"
            f"  Displacement Error: {self.error_state[6:9].flatten()}\n"
            f"  Gyro Bias:  {self.error_state[9:12].flatten()}\n"
            f"  Accelerometer Bias: {self.error_state[12:15].flatten()}\n"
            f"  Magnetometer Bias:  {self.error_state[15:18].flatten()}\n\n"
            f"  WMM Inertial Magnetometer Vec:  {self.magnetometer_inertial.flatten()}\n\n"
            f"{self.nominal_state}"
        )
    
    def state_extrapolation(
            self,
            gyro_new: NDArray[np.float64],
            accel_new: NDArray[np.float64],
            dt: np.float64
        ):

        self.raw_measurements.update_gyro(gyro_new)
        self.raw_measurements.update_accel(accel_new)

        self.nominal_state.state_extrapolation(
            gyro_new=self.raw_measurements.gyro_new,
            gyro_prev=self.raw_measurements.gyro_prev,
            accel_new=self.raw_measurements.accel_new,
            accel_prev=self.raw_measurements.accel_prev,
            dt=dt
        )

        state_transition_matrix = self._state_transition_matrix(dt)
        self.error_state = state_transition_matrix @ self.error_state
        self.error_state_cov_mat = state_transition_matrix @ self.error_state_cov_mat @ state_transition_matrix.T + self._process_noise_cov_matrix(dt)

    def _error_state_gradient_matrix_F(self):
        # non-zero submatrices of F
        omega_matrix = -skew_symmetric(self.raw_measurements.gyro_bar)
        accel_matrix = -0.5 * (
            np.dot(b_to_i_frame_rot_matrix(self.nominal_state.quaternion_new), skew_symmetric(self.raw_measurements.accel_new))
            +
            np.dot(b_to_i_frame_rot_matrix(self.nominal_state.quaternion_prev), skew_symmetric(self.raw_measurements.accel_prev))
        )
        change_of_basis_matrix = -b_to_i_frame_rot_matrix(average_quaternions(self.nominal_state.quaternion_new, self.nominal_state.quaternion_prev))


        F = np.zeros(shape=(18, 18), dtype=float)
        F[0:3, 0:3] = omega_matrix
        F[0:3, 9:12] = -np.eye(3, dtype=float)
        F[3:6, 0:3] = accel_matrix
        F[3:6, 12:15] = change_of_basis_matrix
        F[6:9, 3:6] = np.eye(3, dtype=float)

        return F


    # state transition matrix approximated by discritization of F
    # can make this I + F*dt + 1/2 F^2*dt^2 for better approximation of matrix exponential
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

    def correction_magnetometer(
            self,
            magnetometer_new: NDArray[np.float64],
        ):
        self.raw_measurements.update_mag(magnetometer_new)

        observation_matrix_H = self._observation_matrix_H_magnetometer()
        error_innovation = np.dot(observation_matrix_H, self.error_state)
        self.kalman_gain = np.dot(
            np.dot(self.error_state_cov_mat, observation_matrix_H.T),
            np.linalg.inv(
                np.dot(np.dot(observation_matrix_H, self.error_state_cov_mat), observation_matrix_H.T) + self.magnetometer_cov_mat
            )
        )

        self.error_state = self.error_state + np.dot(self.kalman_gain, error_innovation)
        self.error_state_cov_mat = np.dot(
            (np.eye(18, dtype=float) - np.dot(self.kalman_gain, observation_matrix_H)),
            self.error_state_cov_mat
        )

        # TODO: update nominal state with corrected error state


    def _observation_matrix_H_magnetometer(self):
        # non-zero submatrices of H
        small_angle_update_matrix = skew_symmetric(
            np.dot(
                b_to_i_frame_rot_matrix(average_quaternions(self.nominal_state.quaternion_new, self.nominal_state.quaternion_prev)),
                self.magnetometer_inertial
            )
        )

        H = np.zeros(shape=(3, 18), dtype=float)
        H[0:3, 0:3] = small_angle_update_matrix
        H[0:3, 15:18] = np.eye(3, dtype=float)

        return H
