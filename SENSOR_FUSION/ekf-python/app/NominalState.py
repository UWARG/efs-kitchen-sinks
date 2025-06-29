import numpy as np
from utils import skew_symmetric, normalize_quaternion

class NominalState:
    def __init__(self, position, velocity, quaternion, prev_gyro_measurement, prev_accel_measurement):
        self.displacement = np.asarray(position, dtype=float).reshape(3, 1)
        self.velocity = np.asarray(velocity, dtype=float).reshape(3, 1)
        self.quaternion = normalize_quaternion(np.asarray(quaternion, dtype=float)).reshape(4, 1)
        self.prev_gyro_measurement = np.asarray(prev_gyro_measurement, dtype=float).reshape(3, 1)
        self.prev_accel_measurement = np.asarray(prev_accel_measurement, dtype=float).reshape(3, 1)

    def __str__(self):
        return (f"Nominal State:\n"
                f"  Displacement (p):   {self.displacement.flatten()} m\n"
                f"  Velocity (v):   {self.velocity.flatten()} m/s\n"
                f"  Quaternion (q): {self.quaternion.flatten()} (w,x,y,z)")

    def update_nominal_state(self, gyro_body, accel_body, dt, g_inertial):
        """
        Updates the nominal state (position, velocity, quaternion) using IMU measurements.

        Args:
            omega_body (np.array): Angular velocity vector [wx, wy, wz] in body frame (rad/s).
            accel_body (np.array): Specific force vector [ax, ay, az] in body frame (m/s^2).
            dt (float): Time step in seconds.
            g_inertial (np.array): Gravity vector [gx, gy, gz] in inertial frame (m/s^2).
        """
        # Ensure input dimensions are consistent (column vectors where appropriate)
        omega_body = np.asarray(omega_body, dtype=float).reshape(3, 1)
        accel_body = np.asarray(accel_body, dtype=float).reshape(3, 1)
        g_inertial = np.asarray(g_inertial, dtype=float).reshape(3, 1)

        # 1. Update Quaternion (q_k+1 = exp(0.5 * Omega(omega*dt)) * q_k)
        # The calculate_exp_omega_correct function expects a flat gyro_vec and returns a 4x4 matrix
        exp_term_matrix = calculate_exp_omega_correct(omega_body.flatten(), dt)
        
        # Perform the matrix-vector multiplication to update the quaternion
        # q_prev is reshaped to 4x1 for dot product, then flattened back to 1D array for storage
        q_new_unnormalized = np.dot(exp_term_matrix, self.q.reshape(4, 1)).flatten()
        self.q = normalize_quaternion(q_new_unnormalized)

        # 2. Update Velocity (v_k+1 = v_k + (C_b_i * f_b + g_i) * dt)
        # Rotate body-frame specific force to inertial frame using the current quaternion
        # Note: For nominal propagation, typically the 'old' quaternion is used for the
        # acceleration calculation, representing the state *at the beginning* of the interval.
        R_b_to_i = quat_to_rot_matrix(self.q) 
        
        # Calculate acceleration in inertial frame (specific force rotated + gravity)
        accel_inertial = np.dot(R_b_to_i, accel_body) + g_inertial

        self.v = self.v + accel_inertial * dt

        # 3. Update Position (p_k+1 = p_k + v_k * dt + 0.5 * accel_inertial * dt^2)
        # Using the velocity at the start of the interval (v_k) for simple Euler integration
        self.p = self.p + self.v * dt + 0.5 * accel_inertial * dt**2