import numpy as np
from numpy.typing import NDArray

from pyekf.utils import to_col_vector
from pyekf.quaternions import quaternion_exponential, multiply_quaternions

class ConstantMotionTrajectory:
    """
    Simulates a deterministic trajectory of object with constant linear acceleration and constant angular velocity, both defined in the inertial frame (iframe).
    """

    def __init__(
        self, 
        displacement_initial_iframe: NDArray[np.float64],   # m
        velocity_initial_iframe: NDArray[np.float64],       # m/s
        quaternion_initial_iframe: NDArray[np.float64],     # unitless [w, x, y, z]
        accel_constant_iframe: NDArray[np.float64],         # m/s^2
        angular_vel_constant_iframe: NDArray[np.float64]    # rad/s
    ):
        self.displacement_initial_iframe: NDArray[np.float64] = to_col_vector(displacement_initial_iframe, 3)
        self.velocity_initial_iframe: NDArray[np.float64] = to_col_vector(velocity_initial_iframe, 3)
        self.quaternion_initial_iframe: NDArray[np.float64] = to_col_vector(quaternion_initial_iframe, 4)
        self.accel_constant_iframe: NDArray[np.float64] = to_col_vector(accel_constant_iframe, 3)
        self.angular_vel_constant_iframe: NDArray[np.float64] = to_col_vector(angular_vel_constant_iframe, 3)

    def get_state(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Returns the state of the trajectory at time t in the inertial frame.
        - Displacement (m)
        - Velocity (m/s)
        - Quaternion (unitless)
        - Angular velocity (rad/s)
        """
        # Closed-form solution for quaternion in iframe at time t with constant angular velocity
        angle_axis_iframe = self.angular_vel_constant_iframe * t
        q_delta = quaternion_exponential(angle_axis_iframe)
        quaternion_iframe = multiply_quaternions(q_delta, self.quaternion_initial_iframe)

        # Closed-form solution for velocity and displacement in iframe at time t with constant acceleration
        velocity_iframe = self.velocity_initial_iframe + self.accel_constant_iframe * t
        displacement_iframe = self.displacement_initial_iframe + self.velocity_initial_iframe * t + 0.5 * self.accel_constant_iframe * (t**2)

        return displacement_iframe, velocity_iframe, quaternion_iframe, self.angular_vel_constant_iframe
