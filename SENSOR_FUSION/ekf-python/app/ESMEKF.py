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
    
    def predict(self):
        pass

    # make I + delta_t * F
    def _state_transition_matrix(self):
        pass

    def _process_noise_cov_matrix(self):
        pass

    def correct(self):
        pass
