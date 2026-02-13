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
    b_to_i_frame_rot_matrix,
)
from pyekf.NominalState import NominalState
from pyekf.Measurements import Measurements

class AHRS_ESMEKF:
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
            magnetometer_cov: np.float64 = np.float64(0.0),
            gyro_bias_cov: np.float64 = np.float64(0.0),
            accel_bias_cov: np.float64 = np.float64(0.0),
            magnetometer_bias_cov: np.float64 = np.float64(0.0),
            
            # Initial Covariance Estimates
            p_init_att: float = 0.1,
            p_init_bias: float = 0.01
        ):

        # Raw Measurements
        self.measurements = Measurements(
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
        
        # Store Environmental Constants
        self.gravity_inertial = gravity_inertial
        self.magnetometer_inertial = magnetometer_inertial

        # Initialize Error State Vector (9x1) - this is what makes it ES-MEKF
        self.error_state: NDArray[np.float64] = np.zeros((9, 1))
        
        # Initialize Covariance Matrix P (9x9)
        self.P: NDArray[np.float64] = np.eye(9)
        self.P[0:3, 0:3] *= p_init_att
        self.P[3:6, 3:6] *= p_init_bias # Gyro Bias Uncertainty
        self.P[6:9, 6:9] *= p_init_bias # Accel Bias Uncertainty

        # Initialize Process Noise Matrix Q (9x9)
        self.Q = np.zeros((9, 9))
        self.Q[0:3, 0:3] = np.eye(3) * gyro_cov
        self.Q[3:6, 3:6] = np.eye(3) * gyro_bias_cov
        self.Q[6:9, 6:9] = np.eye(3) * accel_bias_cov

        # Initialize Measurement Noise Matrices R
        self.R_accel = np.eye(3) * accel_cov
        self.R_mag = np.eye(3) * magnetometer_cov
        
        # Initialize Kalman Gain Matrix
        self.kalman_gain = np.zeros((9, 3))
        
        # Error state reset threshold (radians for attitude, m/s^2 for biases)
        self.reset_threshold_att = 0.1  # ~5.7 degrees
        self.reset_threshold_bias = 0.5

    def __str__(self):
        return (
            "====================================================\n"
            "AHRS ESMEKF Internal State (9-State AHRS):\n"
            "----------------------------------------------------\n"
            "  Error State Vector (9x1):\n"
            "    Small Angle Error [0:3]:  {att_err}\n"
            "    Gyro Bias Error   [3:6]:  {gb_err}\n"
            "    Accel Bias Error  [6:9]:  {ab_err}\n\n"
            
            "  Accumulated Biases:\n"
            "    Gyro Bias:   {gb}\n"
            "    Accel Bias:  {ab}\n\n"
            
            "  Environment:\n"
            "    Gravity Inertial:         {gi}\n"
            "    Magnetometer Inertial:    {mi}\n\n"
            
            "  Uncertainty (Covariance P Diagonals):\n"
            "    Attitude P: {p_att}\n"
            "    G-Bias P:   {p_gb}\n"
            "    A-Bias P:   {p_ab}\n\n"
            
            "  Latest Kalman Gain (9x3):\n"
            "{kg}\n\n"
            
            "{ns}" # NominalState __str__
            "{ms}" # Measurements __str__
            "===================================================="
        ).format(
            att_err=self.error_state[0:3].flatten(),
            gb_err=self.error_state[3:6].flatten(),
            ab_err=self.error_state[6:9].flatten(),
            gb=self.measurements.gyro_bias_accumulated.flatten(),
            ab=self.measurements.accel_bias_accumulated.flatten(),
            gi=self.gravity_inertial.flatten(),
            mi=self.magnetometer_inertial.flatten(),
            p_att=np.diag(self.P)[0:3],
            p_gb=np.diag(self.P)[3:6],
            p_ab=np.diag(self.P)[6:9],
            kg=self.kalman_gain,
            ns=str(self.nominal_state),
            ms=str(self.measurements)
        )

    def state_extrapolation(
            self,
            gyro_new: NDArray[np.float64],
            accel_new: NDArray[np.float64],
            dt: np.float64
        ):
        # Update measurements with new data
        self.measurements.update_gyro(gyro_new)
        self.measurements.update_accel(accel_new)

        # Propagate the nominal state using the new measurements
        self.nominal_state.state_extrapolation(
            self.measurements.gyro_new, self.measurements.gyro_prev,
            self.measurements.accel_new, self.measurements.accel_prev,
            dt
        )

        # Propagate error state covariance P
        gyro_body = self.measurements.gyro_new
        gyro_skew = skew_symmetric(gyro_body)

        # Create the state transition matrix F (9x9)
        F = np.eye(9)
        F[0:3, 0:3] = np.eye(3) - (gyro_skew * dt) 
        F[0:3, 3:6] = -np.eye(3) * dt

        # Propagate error state (ES-MEKF)
        self.error_state = F @ self.error_state

        # Update P
        self.P = F @ self.P @ F.T + self.Q * dt
        self.P = (self.P + self.P.T) / 2.0

    def correction_magnetometer(self, magnetometer_new: NDArray[np.float64], gate_threshold: float = 16.3):
        self.measurements.update_mag(magnetometer_new)

        # Predicted measurement in body frame
        R_matrix = b_to_i_frame_rot_matrix(self.nominal_state.quaternion_new)
        m_pred_body = R_matrix.T @ self.magnetometer_inertial

        # Normalize magnetometer prediction and measurement to avoid scale issues in innovation
        m_pred_body = normalize_vector(m_pred_body)
        m_meas_body = normalize_vector(self.measurements.mag_new)

        # Calculate innovation accounting for error state (ES-MEKF)
        m_skew = skew_symmetric(m_pred_body)
        innovation = m_meas_body - m_pred_body - m_skew @ self.error_state[0:3]

        # Jacobian H (3x9)
        H = np.zeros((3, 9))
        H[0:3, 0:3] = m_skew

        # Apply Measurement Update
        self._apply_update(innovation, H, self.R_mag, gate_threshold)

    def correction_accelerometer(self, accel_new: NDArray[np.float64], gate_threshold: float = 7.80):
        self.measurements.update_accel(accel_new)

        # TODO: Consider ignoring accelerometer updates when the device is accelerating significantly
        # linearly (using accel norm) to avoid bad corrections during dynamic motion. This would require 
        # a more complex gating strategy that considers both the innovation and the current acceleration 
        # level. For now, we will rely on the Mahalanobis distance gating to reject outliers during dynamic 
        # motion, but this is an area for future improvement.

        # Predicted measurement in body frame
        R_matrix = b_to_i_frame_rot_matrix(self.nominal_state.quaternion_new)
        g_reaction_inertial = -self.gravity_inertial # Reaction force

        # Normalize gravity prediction to avoid scale issues in innovation
        g_pred_body = R_matrix.T @ g_reaction_inertial
        g_pred_body = normalize_vector(g_pred_body)

        # Normalize the accelerometer measurement to avoid scale issues in innovation
        a_meas_body = normalize_vector(self.measurements.accel_new)

        # Calculate innovation accounting for error state (ES-MEKF)
        g_skew = skew_symmetric(g_pred_body)
        innovation = a_meas_body - g_pred_body - g_skew @ self.error_state[0:3] - self.error_state[6:9]

        # Jacobian H (3x9)
        H = np.zeros((3, 9))
        H[0:3, 0:3] = g_skew
        H[0:3, 6:9] = np.eye(3)

        # Apply measurement update
        self._apply_update(innovation, H, self.R_accel, gate_threshold)

    def _apply_update(self, y, H, R, gate_threshold):
        # S = H P H.T + R, where S is the innovation covariance
        S = H @ self.P @ H.T + R
        
        # Mahalanobis Distance Gating
        try:
            S_inv = np.linalg.inv(S)
            if y.T @ S_inv @ y > gate_threshold:
                return 
        except np.linalg.LinAlgError:
            return

        # Update the Kalman Gain K (9x3)
        K = self.P @ H.T @ S_inv
        self.kalman_gain = K

        # Update error state (ES-MEKF)
        self.error_state = self.error_state + K @ y

        # Update P
        I = np.eye(9)
        self.P = (I - K @ H) @ self.P
        self.P = (self.P + self.P.T) / 2.0

        # Check if error state exceeds threshold and reset if needed
        self._check_and_reset_error_state()

    def _check_and_reset_error_state(self):
        att_error = self.error_state[0:3]
        gyro_bias_error = self.error_state[3:6]
        accel_bias_error = self.error_state[6:9]
        
        # Check if any error component exceeds threshold
        if (np.linalg.norm(att_error) > self.reset_threshold_att or
            np.linalg.norm(gyro_bias_error) > self.reset_threshold_bias or
            np.linalg.norm(accel_bias_error) > self.reset_threshold_bias):
            
            # Apply error state to nominal state
            self.nominal_state.correct_state(att_error, np.zeros((3,1)), np.zeros((3,1)))
            
            # Apply bias errors to accumulated biases
            self.measurements.update_biases(gyro_bias_error, accel_bias_error, np.zeros((3,1)))
            
            # Reset error state to zero
            self.error_state = np.zeros((9, 1))
