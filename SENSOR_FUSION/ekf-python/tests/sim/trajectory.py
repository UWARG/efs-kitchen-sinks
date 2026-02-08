import numpy as np
from numpy.typing import NDArray

from pyekf.utils import to_col_vector
from pyekf.quaternions import quaternion_exponential, multiply_quaternions

class ConstantMotionTrajectory:
    def __init__(self, displacement_initial, velocity_initial, quaternion_initial, accel_body, omega_body):
        """
        displacement_initial, velocity_initial: (3, 1) column vectors
        quaternion_initial: (4, 1) column vector [w, x, y, z]^T
        accel_body: (3, 1) linear acceleration
        omega_body: (3, 1) angular velocity
        """
        self.displacement_initial = to_col_vector(displacement_initial, 3)
        self.velocity_initial = to_col_vector(velocity_initial, 3)
        self.quaternion_initial = to_col_vector(quaternion_initial, 4)
        self.accel_body = to_col_vector(accel_body, 3)
        self.angular_vel_body = to_col_vector(omega_body, 3)

    def get_state(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        # 1. Orientation
        angle_axis = self.angular_vel_body * t
        q_delta = quaternion_exponential(angle_axis)
        q_t = multiply_quaternions(self.quaternion_initial, q_delta)

        # 2. Displacement + Velocity
        v_t = self.velocity_initial + self.accel_body * t
        p_t = self.displacement_initial + self.velocity_initial * t + 0.5 * self.accel_body * (t**2)

        return p_t, v_t, q_t
