import numpy as np

def skew_symmetric_cross_matrix(vec):
    """
    Constructs the 3x3 skew-symmetric matrix for cross product: [vec x].
    vec = [vx, vy, vz]
    """
    vx, vy, vz = vec[0], vec[1], vec[2]
    return np.array([
        [0, -vz, vy],
        [vz, 0, -vx],
        [-vy, vx, 0]
    ])

def calculate_exp_omega_sigma_half(sigma_vec):
    """
    Calculates the 4x4 matrix exponential exp(0.5 * Omega(sigma_vec)) using the
    closed-form formula derived from the Rodrigues' formula / quaternion exponential map.

    Args:
        sigma_vec (np.array): The rotation vector (omega * dt) = [sigma_x, sigma_y, sigma_z].

    Returns:
        np.array: The 4x4 matrix for quaternion propagation.
    """
    norm_sigma = np.linalg.norm(sigma_vec)

    # Handle the case of very small rotation to avoid division by zero and numerical instability
    if norm_sigma < 1e-9:
        # For very small angles, sin(x)/x approaches 1, and cos(x) approaches 1.
        # The matrix becomes identity for zero rotation.
        return np.eye(4)

    half_norm_sigma = norm_sigma / 2.0
    sin_half_norm_sigma = np.sin(half_norm_sigma)
    cos_half_norm_sigma = np.cos(half_norm_sigma)

    # Common factor for the vector parts
    sinc_half_norm_sigma = sin_half_norm_sigma / norm_sigma # This is sin(theta/2) / (theta/2) * (1/2) in some notations
                                                            # but here it's sin(angle)/angle
    # sinc_half_norm_sigma = sin_half_norm_sigma / 2 # This is sin(theta/2) / (theta/2) * (1/2) in some notations
                                                            # but here it's sin(angle)/angle

    # Initialize the 4x4 matrix
    exp_matrix = np.zeros((4, 4))

    # Top-left scalar part
    exp_matrix[0, 0] = cos_half_norm_sigma

    # Top-right 1x3 part (- (sin(||sigma||/2) / ||sigma||) * sigma_vec^T )
    exp_matrix[0, 1:] = -sinc_half_norm_sigma * sigma_vec

    # Bottom-left 3x1 part ( (sin(||sigma||/2) / ||sigma||) * sigma_vec )
    exp_matrix[1:, 0] = sinc_half_norm_sigma * sigma_vec

    # Bottom-right 3x3 matrix
    # cos(||sigma||/2)I_3x3 - (sin(||sigma||/2) / ||sigma||) * [sigma x]
    exp_matrix[1:, 1:] = cos_half_norm_sigma * np.eye(3) - \
                         sinc_half_norm_sigma * skew_symmetric_cross_matrix(sigma_vec)

    return exp_matrix

def calculate_exp_omega(gyro_vec, delta_t):

    norm_gyro = np.linalg.norm(gyro_vec)
    norm_sigma = 0.5 * delta_t * norm_gyro

    # Handle the case of very small rotation to avoid division by zero and numerical instability
    if norm_gyro < 1e-9:
        # For very small angles, sin(x)/x approaches 1, and cos(x) approaches 1.
        # The matrix becomes identity for zero rotation.
        return np.eye(4)
    
    gx, gy, gz = gyro_vec[0], gyro_vec[1], gyro_vec[2]
    gyro_mult_matrix = np.array((
        [0, -gx, -gy, -gz],
        [gx, 0, gz, -gy],
        [gy, -gz, 0, gx],
        [gz, gy, -gx, 0]
    )) 

    return (np.cos(norm_sigma) * np.eye(4) + np.sin(norm_sigma)/norm_gyro*gyro_mult_matrix)




def quaternion_update_explicit_exp_matrix(q_prev, omega_body, dt):
    """
    Updates a quaternion using the explicit 4x4 matrix exponential derivation.

    Args:
        q_prev (np.array): Previous quaternion [w, x, y, z].
        omega_body (np.array): Angular velocity vector [wx, wy, wz] in body frame (rad/s).
        dt (float): Time step in seconds.

    Returns:
        np.array: Updated quaternion [w, x, y, z].
    """
    # Calculate the rotation vector sigma = omega * dt
    # sigma_vec = omega_body * dt

    # Get the 4x4 exponential matrix
    # exp_term_matrix = calculate_exp_omega_sigma_half(sigma_vec)
    exp_term_matrix = calculate_exp_omega(omega_body, dt)

    # Perform the matrix-vector multiplication
    # Ensure q_prev is treated as a column vector for multiplication
    q_new = np.dot(exp_term_matrix, q_prev.reshape(4, 1)).flatten()

    # Normalize the new quaternion (good practice for numerical stability over many steps)
    return q_new / np.linalg.norm(q_new)

# --- Example Usage ---
if __name__ == "__main__":
    # Initial quaternion (no rotation, [1, 0, 0, 0] for scalar-first)
    q_initial = np.array([1.0, 0.0, 0.0, 0.0])

    # Angular velocity (e.g., rotating around Z-axis at 1 rad/s)
    omega_gyro = np.array([0.0, 0.0, 1.0]) # rad/s

    # Time step
    delta_t = 0.01 # seconds

    print(f"Initial Quaternion: {q_initial}")
    print(f"Angular Velocity: {omega_gyro} rad/s")
    print(f"Time Step: {delta_t} s\n")

    # Update the quaternion using the explicit matrix method
    q_updated_explicit = quaternion_update_explicit_exp_matrix(q_initial, omega_gyro, delta_t)

    print(f"Updated Quaternion (Explicit Exp Matrix): {q_updated_explicit}")

    # For comparison, let's use the simple axis-angle method from before.
    # They should yield identical results.
    def normalize_quaternion(q):
        return q / np.linalg.norm(q)

    def quaternion_from_axis_angle(axis, angle):
        if np.linalg.norm(axis) < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])
        half_angle = angle / 2.0
        s_half_angle = np.sin(half_angle)
        c_half_angle = np.cos(half_angle)
        axis = axis / np.linalg.norm(axis)
        return np.array([c_half_angle, axis[0] * s_half_angle, axis[1] * s_half_angle, axis[2] * s_half_angle])

    def quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return normalize_quaternion(np.array([w, x, y, z]))

    def quaternion_update_exp_map_comparison(q_prev, omega_body, dt):
        delta_theta = np.linalg.norm(omega_body) * dt
        if delta_theta < 1e-9:
            return q_prev
        rotation_axis = omega_body / np.linalg.norm(omega_body)
        delta_q = quaternion_from_axis_angle(rotation_axis, delta_theta)
        q_new = quaternion_multiply(q_prev, delta_q)
        return normalize_quaternion(q_new)

    q_updated_comparison = quaternion_update_exp_map_comparison(q_initial, omega_gyro, delta_t)
    print(f"Updated Quaternion (Axis-Angle comparison): {q_updated_comparison}")

    # Verify they are very close
    print(f"Difference (Explicit - Axis-Angle): {np.linalg.norm(q_updated_explicit - q_updated_comparison)}")

    # Multi-step example for the explicit method
    q_current_explicit = np.array([1.0, 0.0, 0.0, 0.0])
    print("\n--- Multi-step update (Explicit Exp Matrix) ---")
    num_steps = 100
    total_time = num_steps * delta_t
    print(f"Simulating {num_steps} steps over {total_time} seconds with 1 rad/s around Z-axis.")
    for i in range(num_steps):
        q_current_explicit = quaternion_update_explicit_exp_matrix(q_current_explicit, omega_gyro, delta_t)
    print(f"Final Quaternion (Explicit Exp Matrix, after {num_steps} steps): {q_current_explicit}")
    expected_q_z = np.array([np.cos(0.5 * total_time), 0, 0, np.sin(0.5 * total_time)])
    print(f"Expected final Z rotation by {total_time} rad: {expected_q_z}")
