from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

@dataclass
class SensorParams:
    """Defines noise characteristics and bias stability for the sensor suite."""
    # Sensor random noise variances (Diagonal of Covariance Matrices)
    gyro_cov: float                # (rad/s)^2
    accel_cov: float               # (m/s^2)^2
    magnetometer_cov: float        # unitless (normalized field)

    # Sensor bias variances (Used to sample the constant offsets in Simulator)
    gyro_bias_cov: float           # (rad/s)^2
    accel_bias_cov: float          # (m/s^2)^2
    magnetometer_bias_cov: float   # unitless (normalized field)

@dataclass
class ConstantSimParams:
    """Defines the flight scenario and environment for a constant motion test."""
    # Timing
    delta_t: float                 # s
    duration: float                # s

    # Initial State (Inertial Frame, 3x1 and 4x1)
    displacement_initial_iframe: NDArray[np.float64]  # m
    velocity_initial_iframe: NDArray[np.float64]      # m/s
    quaternion_initial_iframe: NDArray[np.float64]    # unitless [w, x, y, z]

    # Constant Motion Truth (Inertial Frame, 3x1)
    accel_constant_iframe: NDArray[np.float64]        # m/s^2
    angular_vel_constant_iframe: NDArray[np.float64]  # rad/s

    # Reference Fields
    gravity_iframe: NDArray[np.float64]      # m/s^2
    mag_field_iframe: NDArray[np.float64]    # unitless
