from pyekf.quaternions import quaternion_exponential, multiply_quaternions

class ConstantMotionTrajectory:
    def __init__(self, p0, v0, q0, accel_body, omega_body):
        """
        p0, v0: (3, 1) column vectors
        q0: (4, 1) column vector [w, x, y, z]^T
        accel_body: (3, 1) linear acceleration
        omega_body: (3, 1) angular velocity
        """
        self.p0 = p0.reshape(3, 1)
        self.v0 = v0.reshape(3, 1)
        self.q0 = q0.reshape(4, 1)
        self.a_b = accel_body.reshape(3, 1)
        self.w_b = omega_body.reshape(3, 1)

    def get_state(self, t):
        # 1. Orientation Integration
        angle_axis = self.w_b * t
        q_delta = quaternion_exponential(angle_axis)
        q_t = multiply_quaternions(self.q0, q_delta)

        # 2. Kinematics (Assuming accel is constant in the Frame of Reference)
        v_t = self.v0 + self.a_b * t
        p_t = self.p0 + self.v0 * t + 0.5 * self.a_b * (t**2)

        return p_t, v_t, q_t
