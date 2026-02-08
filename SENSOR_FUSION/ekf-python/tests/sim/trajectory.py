import numpy as np
from numpy.typing import NDArray

from pyekf.utils import to_col_vector
from pyekf.quaternions import quaternion_exponential, multiply_quaternions

class ConstantMotionTrajectory:
    def __init__(
        self, 
        displacement_initial_iframe: NDArray[np.float64], 
        velocity_initial_iframe: NDArray[np.float64], 
        quaternion_initial_iframe: NDArray[np.float64], 
        accel_constant_iframe: NDArray[np.float64], 
        angular_vel_constant_iframe: NDArray[np.float64]
    ):
        self.displacement_initial_iframe: NDArray[np.float64] = to_col_vector(displacement_initial_iframe, 3)
        self.velocity_initial_iframe: NDArray[np.float64] = to_col_vector(velocity_initial_iframe, 3)
        self.quaternion_initial_iframe: NDArray[np.float64] = to_col_vector(quaternion_initial_iframe, 4)
        self.accel_constant_iframe: NDArray[np.float64] = to_col_vector(accel_constant_iframe, 3)
        self.angular_vel_constant_iframe: NDArray[np.float64] = to_col_vector(angular_vel_constant_iframe, 3)

    def get_state(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Returns the state of the trajectory at time t in the inertial frame. The state includes:
        - Displacement (3x1 vector)
        - Velocity (3x1 vector)
        - Quaternion (4x1 vector)
        - Angular velocity (3x1 vector)
        """
        # Integration of orientation in inertial frame
        angle_axis_iframe = self.angular_vel_constant_iframe * t
        q_delta = quaternion_exponential(angle_axis_iframe)
        quaternion_iframe = multiply_quaternions(q_delta, self.quaternion_initial_iframe)

        # Integration of constant acceleration in inertial frame
        velocity_iframe = self.velocity_initial_iframe + self.accel_constant_iframe * t
        displacement_iframe = self.displacement_initial_iframe + self.velocity_initial_iframe * t + 0.5 * self.accel_constant_iframe * (t**2)

        return displacement_iframe, velocity_iframe, quaternion_iframe, self.angular_vel_constant_iframe
