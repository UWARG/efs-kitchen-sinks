import numpy as np
from numpy.typing import NDArray
from typing import Final

from pyekf.utils import (
    to_col_vector,
    skew_symmetric,
    GRAVITY_INERTIAL,
    MAGNETOMETER_INERTIAL,
    ensure_symmetric_matrix,
    normalize_vector,
)
from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    i_to_b_frame_rot_matrix,
)
from pyekf.ahrs_esmekf.NominalState import NominalState
from pyekf.Measurements import Measurements

class AHRS_ESMEKF:
    # Constants
    ERROR_STATE_SZ: Final[int] = 9

    def __init__(
            self,
            # Initial Measurements
            gyro_initial: NDArray[np.float64] = np.zeros((3, 1)),
            accel_initial: NDArray[np.float64] = np.zeros((3, 1)),
            mag_initial: NDArray[np.float64] = np.zeros((3, 1)),

            # Initialize Nominal State
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,

            # Initialize ESMEKF (tunable)
            gyro_cov: np.float64 = np.float64(0.0),
            accel_cov: np.float64 = np.float64(0.0),
            magnetometer_cov: np.float64 = np.float64(0.0),
            gyro_bias_cov: np.float64 = np.float64(0.0),
            accel_bias_cov: np.float64 = np.float64(0.0),

            # Mahalanobis Gating Threshold (tunable)
            accel_gate_threshold: np.float64 = np.float64(7.80), # corresponds to 3-sigma for 3 DOF, 95% confidence interval
            magnetometer_gate_threshold: np.float64 = np.float64(16.3), # corresponds to 3-sigma for 3 DOF, 99.9% confidence interval

            # Initial Covariance Estimates (tunable)
            p_init_att: float = 0.1,
            p_init_bias: float = 0.01,

            # Environmental Constants
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
            magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,
        ):

        # Raw Measurements
        self.measurements = Measurements(
            gyro_initial=gyro_initial,
            accel_initial=accel_initial,
            mag_initial=mag_initial,
        )

        # Nominal State
        self.nominal_state = NominalState(
            quaternion_initial=quaternion_initial,
        )

        # Sensor noise covariance tuning params
        # Assuming assuming identical independent covariance across x,y,z axises
        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_cov)
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_cov)
        self.magnetometer_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(magnetometer_cov)
        self.gyro_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_bias_cov)
        self.accel_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_bias_cov)

        # EKF
        # small_angle_error = self.error_state[0:3]
        # gyro_bias = self.error_state[3:6]
        # accelerometer_bias = self.error_state[6:9]
        self.error_state = np.zeros((self.ERROR_STATE_SZ, 1), dtype=np.float64) # strictly here for logging
        self.kalman_gain = np.zeros((self.ERROR_STATE_SZ, 3), dtype=np.float64) # strictly here for logging, 3 can change based on observation size
        self.P: NDArray[np.float64] = np.zeros((self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float) # Error State Covariance Matrix
        self.P[0:3, 0:3] = np.eye(3, dtype=np.float64) * p_init_att
        self.P[3:6, 3:6] = np.eye(3, dtype=np.float64) * p_init_bias # Gyro Bias Uncertainty
        self.P[6:9, 6:9] = np.eye(3, dtype=np.float64) * p_init_bias # Accel Bias Uncertainty

        # Malhanobis Gating Threshold (tunable)
        self.accel_gate_threshold = accel_gate_threshold
        self.magnetometer_gate_threshold = magnetometer_gate_threshold

        # environmental constants
        self.gravity_inertial = to_col_vector(gravity_inertial, 3)
        self.magnetometer_inertial = to_col_vector(magnetometer_inertial, 3)


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
            dt: np.float64
        ):

        self.measurements.update_gyro(gyro_new)

        self.nominal_state.state_extrapolation(
            gyro_new=self.measurements.gyro_new,
            gyro_prev=self.measurements.gyro_prev,
            dt=dt
        )

        state_transition_matrix = self._state_transition_matrix(dt)
        self.P = state_transition_matrix @ self.P @ state_transition_matrix.T + self._process_noise_cov_matrix(dt)
        self.P = ensure_symmetric_matrix(self.P) # ensure P stays symmetric for numerical stability

    # state transition matrix approximated by discritization of F
    # can make this I + F*dt + 1/2 F^2*dt^2 for better approximation of matrix exponential
    def _state_transition_matrix(self, dt: np.float64):
        F = self._error_state_gradient_matrix_F()
        return np.eye(self.ERROR_STATE_SZ, dtype=float) + dt * F + 0.5 * (dt**2) * F @ F
    
    def _error_state_gradient_matrix_F(self):
        # non-zero submatrices of F
        omega_matrix = -skew_symmetric(self.measurements.gyro_bar)

        F = np.zeros(shape=(self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float)
        F[0:3, 0:3] = omega_matrix
        F[0:3, 3:6] = -np.eye(3, dtype=float)

        return F

    def _process_noise_cov_matrix(self, dt):
        Q = np.zeros(shape=(self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float)
        Q[0:3, 0:3] = self.gyro_cov_mat*dt + self.gyro_bias_cov_mat*(dt**3)/3.0
        Q[0:3, 3:6] = -self.gyro_bias_cov_mat*(dt**2)/2.0
        Q[3:6, 0:3] = -self.gyro_bias_cov_mat*(dt**2)/2.0
        Q[3:6, 3:6] = self.gyro_bias_cov_mat*dt
        Q[6:9, 6:9] = self.accel_bias_cov_mat*dt

        return Q

    def correction_accelerometer(
            self,
            accelerometer_new: NDArray[np.float64],
        ):
        # ------------------------------------------------------------------
        # 1. Update Measurements
        # ------------------------------------------------------------------
        self.measurements.update_accel(accelerometer_new)

        # ------------------------------------------------------------------
        # 2. Innovation
        # ------------------------------------------------------------------
        accel_predicted = i_to_b_frame_rot_matrix(self.nominal_state.quaternion_new) @ -self.gravity_inertial
        innovation = self.measurements.accel_new - accel_predicted
    
        # ------------------------------------------------------------------
        # 3. Measurement Observation Matrix
        # ------------------------------------------------------------------
        H = np.zeros(shape=(3,self.ERROR_STATE_SZ), dtype=float)
        small_angle_update_matrix = skew_symmetric(accel_predicted)
        H[0:3, 0:3] = small_angle_update_matrix # TODO: check if negative or positive
        H[0:3, 6:9] = np.eye(3, dtype=float)

        # ------------------------------------------------------------------
        # 4. Kalman Update step
        # ------------------------------------------------------------------
        self._apply_update(
            y=innovation,
            H=H,
            R=self.accel_cov_mat,
            gate_threshold=self.accel_gate_threshold
        )

    def correction_magnetometer(
            self,
            magnetometer_new: NDArray[np.float64],
        ):
        # ------------------------------------------------------------------
        # 1. Update Measurements
        # ------------------------------------------------------------------
        self.measurements.update_mag(normalize_vector(magnetometer_new))

        # ------------------------------------------------------------------
        # 2. Innovation
        # ------------------------------------------------------------------
        mag_predicted = normalize_vector(i_to_b_frame_rot_matrix(self.nominal_state.quaternion_new) @ self.magnetometer_inertial)
        innovation = self.measurements.mag_new - mag_predicted # TODO: maybe use mag bar here, since longer time period and not integrating it?

        # ------------------------------------------------------------------
        # 3. Measurement Observation Matrix
        # ------------------------------------------------------------------
        H = np.zeros(shape=(3, self.ERROR_STATE_SZ), dtype=float)
        small_angle_update_matrix = skew_symmetric(mag_predicted)
        H[0:3, 0:3] = small_angle_update_matrix

        # ------------------------------------------------------------------
        # 4. Kalman Update step
        # ------------------------------------------------------------------
        self._apply_update(
            y=innovation,
            H=H,
            R=self.magnetometer_cov_mat,
            gate_threshold=self.magnetometer_gate_threshold
        )

    def _apply_update(
            self,
            y: NDArray[np.float64],
            H: NDArray[np.float64],
            R: NDArray[np.float64],
            gate_threshold: np.float64
        ):
        """
        y = measurement innovation (residual) (3x1)
        H = observation matrix (3x9)
        R = measurement noise covariance (3x3)
        K = Kalman Gain (9x3)
        error_state (9x1)
        gate_threshold = Mahalanobis distance gating threshold
        """
        # ------------------------------------------------------------------
        # 1. Mahalanobis Distance Gating
        # ------------------------------------------------------------------
        S = H @ self.P @ H.T + R

        try:
            S_inv = np.linalg.inv(S)
            # if y.T @ S_inv @ y > gate_threshold:
            #     return 
        except np.linalg.LinAlgError:
            return
        
        # ------------------------------------------------------------------
        # 2. Kalman Update step
        # ------------------------------------------------------------------
        K = self.P @ H.T @ S_inv
        error_state = K @ y
        I = np.eye(self.ERROR_STATE_SZ, dtype=float)
        self.P = (I - K @ H) @ self.P
        # self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = ensure_symmetric_matrix(self.P)

        # ------------------------------------------------------------------
        # 3. Update State and Bias Estimates
        # ------------------------------------------------------------------
        self.nominal_state.correct_state(
            small_angle_error=error_state[0:3, 0:1],
        )
        self.measurements.update_biases(
            gyro_bias_new=error_state[3:6, 0:1],
            accel_bias_new=error_state[6:9, 0:1],
            mag_bias_new=None,
        )

        # ------------------------------------------------------------------
        # 4. Reset Error State
        # ------------------------------------------------------------------
        J = np.eye(self.ERROR_STATE_SZ, dtype=float) # reset op jacobian
        J[0:3, 0:3] = np.eye(3, dtype=float) - 0.5 * skew_symmetric(error_state[0:3, 0:1])

        self.P = J @ self.P @ J.T
        self.P = ensure_symmetric_matrix(self.P)
