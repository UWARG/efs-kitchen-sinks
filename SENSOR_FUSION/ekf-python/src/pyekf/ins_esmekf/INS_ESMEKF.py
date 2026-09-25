import numpy as np
from numpy.typing import NDArray
from typing import Final

from pyekf.utils import (
    to_col_vector,
    skew_symmetric,
    GRAVITY_INERTIAL,
    MAGNETOMETER_INERTIAL,
    ensure_symmetric_matrix,
)
from pyekf.quaternions import (
    IDENTITY_QUATERNION,
    average_quaternions,
    quaternion_exponential,
    b_to_i_frame_rot_matrix,
    i_to_b_frame_rot_matrix,
)
from pyekf.ins_esmekf.NominalState import NominalState
from pyekf.Measurements import Measurements

class INS_ESMEKF:
    # Constants
    ERROR_STATE_SZ: Final[int] = 18

    def __init__(
            self,
            # Initial Measurements
            gyro_initial: NDArray[np.float64] = np.zeros((3, 1)),
            accel_initial: NDArray[np.float64] = np.zeros((3, 1)),
            mag_initial: NDArray[np.float64] = np.zeros((3, 1)),

            # Initialize Nominal State
            displacement_initial: NDArray[np.float64] = np.zeros((3, 1)),
            velocity_initial: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion_initial: NDArray[np.float64] = IDENTITY_QUATERNION,

            # Initialize ESMEKF (tunable)
            gyro_cov: np.float64 = np.float64(0.0),
            accel_cov: np.float64 = np.float64(0.0),
            magnetometer_cov: np.float64 = np.float64(0.0),
            gyro_bias_cov: np.float64 = np.float64(0.0),
            accel_bias_cov: np.float64 = np.float64(0.0),
            magnetometer_bias_cov: np.float64 = np.float64(0.0),
            gps_position_cov: np.float64 = np.float64(0.0),
            gps_velocity_cov: np.float64 = np.float64(0.0),

            # Mahalanobis Gating Threshold (tunable), chi-squared with 3 DOF at 99.9% confidence
            magnetometer_gate_threshold: np.float64 = np.float64(16.27),
            gps_position_gate_threshold: np.float64 = np.float64(16.27),
            gps_velocity_gate_threshold: np.float64 = np.float64(16.27),

            # Initial Covariance Estimates (tunable)
            p_init_att: float = 0.1,
            p_init_vel: float = 0.1,
            p_init_pos: float = 0.1,
            p_init_gyro_bias: float = 0.01,
            p_init_accel_bias: float = 0.01,
            p_init_mag_bias: float = 0.01,

            # Environmental Constants
            gravity_inertial: NDArray[np.float64] = GRAVITY_INERTIAL,
            magnetometer_inertial: NDArray[np.float64] = MAGNETOMETER_INERTIAL,
        ):

        # Raw Measurements
        self.measurements = Measurements(
            gyro_initial=gyro_initial,
            accel_initial=accel_initial,
            mag_initial=mag_initial,
            gps_position_initial=displacement_initial,
            gps_velocity_initial=velocity_initial,
        )

        # Nominal State
        self.nominal_state = NominalState(
            displacement_initial=displacement_initial,
            velocity_initial=velocity_initial,
            quaternion_initial=quaternion_initial,
            gravity_inertial=gravity_inertial
        )

        # Sensor noise covariance tuning params
        # Assuming identical independent covariance across x,y,z axises
        self.gyro_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_cov)
        self.accel_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_cov)
        self.magnetometer_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(magnetometer_cov)
        self.gyro_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gyro_bias_cov)
        self.accel_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(accel_bias_cov)
        self.magnetometer_bias_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(magnetometer_bias_cov)
        self.gps_position_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gps_position_cov)
        self.gps_velocity_cov_mat: NDArray[np.float64] = np.eye(3, dtype=np.float64) * np.float64(gps_velocity_cov)

        # EKF
        # small_angle_error = self.error_state[0:3]
        # velocity_error = self.error_state[3:6]
        # displacement_error = self.error_state[6:9]
        # gyro_bias = self.error_state[9:12]
        # accelerometer_bias = self.error_state[12:15]
        # magnetometer_bias = self.error_state[15:18]
        self.error_state = np.zeros((self.ERROR_STATE_SZ, 1), dtype=np.float64) # strictly here for logging, last error state before reset
        self.kalman_gain = np.zeros((self.ERROR_STATE_SZ, 3), dtype=np.float64) # strictly here for logging, last kalman gain used
        self.P: NDArray[np.float64] = np.zeros((self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float) # Error State Covariance Matrix
        self.P[0:3, 0:3] = np.eye(3, dtype=np.float64) * p_init_att
        self.P[3:6, 3:6] = np.eye(3, dtype=np.float64) * p_init_vel
        self.P[6:9, 6:9] = np.eye(3, dtype=np.float64) * p_init_pos
        self.P[9:12, 9:12] = np.eye(3, dtype=np.float64) * p_init_gyro_bias # Gyro Bias Uncertainty
        self.P[12:15, 12:15] = np.eye(3, dtype=np.float64) * p_init_accel_bias # Accel Bias Uncertainty
        self.P[15:18, 15:18] = np.eye(3, dtype=np.float64) * p_init_mag_bias # Mag Bias Uncertainty

        # Mahalanobis Gating Threshold (tunable)
        self.magnetometer_gate_threshold = magnetometer_gate_threshold
        self.gps_position_gate_threshold = gps_position_gate_threshold
        self.gps_velocity_gate_threshold = gps_velocity_gate_threshold

        # environmental constants, magnetometer field (e.g. from WMM) must be in the same units as the magnetometer readings
        self.gravity_inertial = to_col_vector(gravity_inertial, 3)
        self.magnetometer_inertial = to_col_vector(magnetometer_inertial, 3)

    def __str__(self):
        return (
            "====================================================\n"
            "INS ESMEKF Internal State (18-State INS):\n"
            "----------------------------------------------------\n"
            "  Error State Vector (18x1):\n"
            "    Small Angle Error [0:3]:    {att_err}\n"
            "    Velocity Error    [3:6]:    {vel_err}\n"
            "    Displacement Error [6:9]:   {pos_err}\n"
            "    Gyro Bias Error   [9:12]:   {gb_err}\n"
            "    Accel Bias Error  [12:15]:  {ab_err}\n"
            "    Mag Bias Error    [15:18]:  {mb_err}\n\n"

            "  Environment:\n"
            "    Gravity Inertial:         {gi}\n"
            "    Magnetometer Inertial:    {mi}\n\n"

            "  Uncertainty (Covariance P Diagonals):\n"
            "    Attitude P: {p_att}\n"
            "    Velocity P: {p_vel}\n"
            "    Position P: {p_pos}\n"
            "    G-Bias P:   {p_gb}\n"
            "    A-Bias P:   {p_ab}\n"
            "    M-Bias P:   {p_mb}\n\n"

            "  Latest Kalman Gain:\n"
            "{kg}\n\n"

            "{ns}" # NominalState __str__
            "{ms}" # Measurements __str__
            "===================================================="
        ).format(
            att_err=self.error_state[0:3].flatten(),
            vel_err=self.error_state[3:6].flatten(),
            pos_err=self.error_state[6:9].flatten(),
            gb_err=self.error_state[9:12].flatten(),
            ab_err=self.error_state[12:15].flatten(),
            mb_err=self.error_state[15:18].flatten(),
            gi=self.gravity_inertial.flatten(),
            mi=self.magnetometer_inertial.flatten(),
            p_att=np.diag(self.P)[0:3],
            p_vel=np.diag(self.P)[3:6],
            p_pos=np.diag(self.P)[6:9],
            p_gb=np.diag(self.P)[9:12],
            p_ab=np.diag(self.P)[12:15],
            p_mb=np.diag(self.P)[15:18],
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

        self.measurements.update_gyro(gyro_new)
        self.measurements.update_accel(accel_new)

        self.nominal_state.state_extrapolation(
            gyro_new=self.measurements.gyro_new,
            gyro_prev=self.measurements.gyro_prev,
            accel_new=self.measurements.accel_new,
            accel_prev=self.measurements.accel_prev,
            dt=dt
        )

        state_transition_matrix = self._state_transition_matrix(dt)
        self.P = state_transition_matrix @ self.P @ state_transition_matrix.T + self._process_noise_cov_matrix(dt)
        self.P = ensure_symmetric_matrix(self.P) # ensure P stays symmetric for numerical stability

    # state transition matrix approximated by discritization of F, Phi = exp(F*dt) ~= I + F*dt + 1/2 F^2*dt^2
    # The attitude block exp(-[w]x dt) is replaced by its exact closed form, a rotation by -w*dt. The truncated series is not
    # exactly a rotation, so the unobservable attitude direction in P slowly drifts away from the nominal state's frame,
    # which lets corrections leak into directions the sensors cannot see (and at first order makes P grow without bound).
    def _state_transition_matrix(self, dt: np.float64):
        F = self._error_state_gradient_matrix_F()
        Phi = np.eye(self.ERROR_STATE_SZ, dtype=float) + dt * F + 0.5 * (dt**2) * F @ F
        Phi[0:3, 0:3] = b_to_i_frame_rot_matrix(quaternion_exponential(self.measurements.gyro_bar * dt)).T
        return Phi

    def _error_state_gradient_matrix_F(self):
        # non-zero submatrices of F, rotation and specific force are averaged over the start and end of the time step
        omega_matrix = -skew_symmetric(self.measurements.gyro_bar)
        accel_matrix = -0.5 * (
            b_to_i_frame_rot_matrix(self.nominal_state.quaternion_new) @ skew_symmetric(self.measurements.accel_new)
            +
            b_to_i_frame_rot_matrix(self.nominal_state.quaternion_prev) @ skew_symmetric(self.measurements.accel_prev)
        )
        change_of_basis_matrix = -self._average_b_to_i_frame_rot_matrix()

        F = np.zeros(shape=(self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float)
        F[0:3, 0:3] = omega_matrix
        F[0:3, 9:12] = -np.eye(3, dtype=float)
        F[3:6, 0:3] = accel_matrix
        F[3:6, 12:15] = change_of_basis_matrix
        F[6:9, 3:6] = np.eye(3, dtype=float)

        return F

    def _process_noise_cov_matrix(self, dt):
        # Integrator chain approximation, see docs/ins_esmekf.md section 7.2
        # accel bias enters velocity rotated into the inertial frame, so its cross terms carry the rotation matrix
        R_bar = self._average_b_to_i_frame_rot_matrix()

        Q = np.zeros(shape=(self.ERROR_STATE_SZ, self.ERROR_STATE_SZ), dtype=float)
        Q[0:3, 0:3] = self.gyro_cov_mat*dt + self.gyro_bias_cov_mat*(dt**3)/3.0
        Q[0:3, 9:12] = -self.gyro_bias_cov_mat*(dt**2)/2.0
        Q[3:6, 3:6] = self.accel_cov_mat*dt + self.accel_bias_cov_mat*(dt**3)/3.0
        Q[3:6, 6:9] = self.accel_cov_mat*(dt**2)/2.0 + self.accel_bias_cov_mat*(dt**4)/8.0
        Q[3:6, 12:15] = -self.accel_bias_cov_mat @ R_bar * (dt**2)/2.0
        Q[6:9, 6:9] = self.accel_cov_mat*(dt**3)/3.0 + self.accel_bias_cov_mat*(dt**5)/20.0
        Q[6:9, 12:15] = -self.accel_bias_cov_mat @ R_bar * (dt**3)/6.0
        Q[9:12, 9:12] = self.gyro_bias_cov_mat*dt
        Q[12:15, 12:15] = self.accel_bias_cov_mat*dt
        Q[15:18, 15:18] = self.magnetometer_bias_cov_mat*dt

        # lower triangle blocks are the transposes of the upper triangle blocks
        Q[9:12, 0:3] = Q[0:3, 9:12].T
        Q[6:9, 3:6] = Q[3:6, 6:9].T
        Q[12:15, 3:6] = Q[3:6, 12:15].T
        Q[12:15, 6:9] = Q[6:9, 12:15].T

        return Q

    def _average_b_to_i_frame_rot_matrix(self):
        return b_to_i_frame_rot_matrix(average_quaternions(self.nominal_state.quaternion_new, self.nominal_state.quaternion_prev))

    def correction_magnetometer(
            self,
            magnetometer_new: NDArray[np.float64],
        ):
        # ------------------------------------------------------------------
        # 1. Update Measurements
        # ------------------------------------------------------------------
        # raw reading is used (not normalized), since a hard iron bias adds to the raw field. Normalizing a biased reading
        # removes the part of the bias along the field, which would make the bias look time varying
        self.measurements.update_mag(magnetometer_new)

        # ------------------------------------------------------------------
        # 2. Innovation
        # ------------------------------------------------------------------
        mag_predicted = i_to_b_frame_rot_matrix(self.nominal_state.quaternion_new) @ self.magnetometer_inertial
        innovation = self.measurements.mag_new - mag_predicted

        # ------------------------------------------------------------------
        # 3. Measurement Observation Matrix
        # ------------------------------------------------------------------
        H = np.zeros(shape=(3, self.ERROR_STATE_SZ), dtype=float)
        H[0:3, 0:3] = skew_symmetric(mag_predicted)
        H[0:3, 15:18] = np.eye(3, dtype=float)

        # ------------------------------------------------------------------
        # 4. Kalman Update step
        # ------------------------------------------------------------------
        self._apply_update(
            y=innovation,
            H=H,
            R=self.magnetometer_cov_mat,
            gate_threshold=self.magnetometer_gate_threshold
        )

    def correction_gps_position(
            self,
            gps_position_new: NDArray[np.float64],
        ):
        # ------------------------------------------------------------------
        # 1. Update Measurements
        # ------------------------------------------------------------------
        # GPS position is assumed already converted to the same local inertial frame as the displacement (e.g. NED)
        self.measurements.update_gps_position(gps_position_new)

        # ------------------------------------------------------------------
        # 2. Innovation
        # ------------------------------------------------------------------
        innovation = self.measurements.gps_position_new - self.nominal_state.displacement_new

        # ------------------------------------------------------------------
        # 3. Measurement Observation Matrix
        # ------------------------------------------------------------------
        H = np.zeros(shape=(3, self.ERROR_STATE_SZ), dtype=float)
        H[0:3, 6:9] = np.eye(3, dtype=float)

        # ------------------------------------------------------------------
        # 4. Kalman Update step
        # ------------------------------------------------------------------
        self._apply_update(
            y=innovation,
            H=H,
            R=self.gps_position_cov_mat,
            gate_threshold=self.gps_position_gate_threshold
        )

    def correction_gps_velocity(
            self,
            gps_velocity_new: NDArray[np.float64],
        ):
        # ------------------------------------------------------------------
        # 1. Update Measurements
        # ------------------------------------------------------------------
        self.measurements.update_gps_velocity(gps_velocity_new)

        # ------------------------------------------------------------------
        # 2. Innovation
        # ------------------------------------------------------------------
        innovation = self.measurements.gps_velocity_new - self.nominal_state.velocity_new

        # ------------------------------------------------------------------
        # 3. Measurement Observation Matrix
        # ------------------------------------------------------------------
        H = np.zeros(shape=(3, self.ERROR_STATE_SZ), dtype=float)
        H[0:3, 3:6] = np.eye(3, dtype=float)

        # ------------------------------------------------------------------
        # 4. Kalman Update step
        # ------------------------------------------------------------------
        self._apply_update(
            y=innovation,
            H=H,
            R=self.gps_velocity_cov_mat,
            gate_threshold=self.gps_velocity_gate_threshold
        )

    def _apply_update(
            self,
            y: NDArray[np.float64],
            H: NDArray[np.float64],
            R: NDArray[np.float64],
            gate_threshold: np.float64
        ):
        """
        y = measurement innovation (residual) (mx1)
        H = observation matrix (mx18)
        R = measurement noise covariance (mxm)
        K = Kalman Gain (18xm)
        error_state (18x1)
        gate_threshold = Mahalanobis distance gating threshold
        """
        # ------------------------------------------------------------------
        # 1. Mahalanobis Distance Gating
        # ------------------------------------------------------------------
        S = H @ self.P @ H.T + R

        try:
            S_inv = np.linalg.inv(S)
            if y.T @ S_inv @ y > gate_threshold:
                return
        except np.linalg.LinAlgError:
            return

        # ------------------------------------------------------------------
        # 2. Kalman Update step
        # ------------------------------------------------------------------
        # Joseph form keeps P symmetric positive semi-definite under round off
        K = self.P @ H.T @ S_inv
        error_state = K @ y
        I_KH = np.eye(self.ERROR_STATE_SZ, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = ensure_symmetric_matrix(self.P)

        self.kalman_gain = K
        self.error_state = error_state

        # ------------------------------------------------------------------
        # 3. Update State and Bias Estimates
        # ------------------------------------------------------------------
        self.nominal_state.correct_state(
            small_angle_error=error_state[0:3, 0:1],
            velocity_error=error_state[3:6, 0:1],
            displacement_error=error_state[6:9, 0:1],
        )
        self.measurements.update_biases(
            gyro_bias_new=error_state[9:12, 0:1],
            accel_bias_new=error_state[12:15, 0:1],
            mag_bias_new=error_state[15:18, 0:1],
        )

        # ------------------------------------------------------------------
        # 4. Reset Error State
        # ------------------------------------------------------------------
        J = np.eye(self.ERROR_STATE_SZ, dtype=float) # reset op jacobian
        J[0:3, 0:3] = np.eye(3, dtype=float) - 0.5 * skew_symmetric(error_state[0:3, 0:1])

        self.P = J @ self.P @ J.T
        self.P = ensure_symmetric_matrix(self.P)
