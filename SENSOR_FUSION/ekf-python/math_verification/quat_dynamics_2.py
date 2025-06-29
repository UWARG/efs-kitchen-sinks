import numpy as np

def normalize_quaternion(q):
    """Normalizes a quaternion."""
    return q / np.linalg.norm(q)

def quaternion_from_axis_angle(axis, angle):
    """
    Creates a quaternion from an axis and an angle (in radians).
    The axis must be a unit vector.
    """
    if np.linalg.norm(axis) < 1e-9: # Handle zero rotation gracefully
        return np.array([1.0, 0.0, 0.0, 0.0]) # Identity quaternion

    half_angle = angle / 2.0
    s_half_angle = np.sin(half_angle)
    c_half_angle = np.cos(half_angle)
    
    # Ensure axis is normalized
    axis = axis / np.linalg.norm(axis)

    return np.array([
        c_half_angle,
        axis[0] * s_half_angle,
        axis[1] * s_half_angle,
        axis[2] * s_half_angle
    ])

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions (q1 * q2).
    Assumes scalar-first format [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return normalize_quaternion(np.array([w, x, y, z]))

def quaternion_update_exp_map(q_prev, omega_body, dt):
    """
    Updates a quaternion using the exponential map (axis-angle to quaternion) method.

    Args:
        q_prev (np.array): Previous quaternion [w, x, y, z].
        omega_body (np.array): Angular velocity vector [wx, wy, wz] in body frame (rad/s).
        dt (float): Time step in seconds.

    Returns:
        np.array: Updated quaternion [w, x, y, z].
    """
    # Calculate the magnitude of the rotation vector (angle)
    delta_theta = np.linalg.norm(omega_body) * dt

    # Handle the case of zero angular velocity to avoid division by zero
    if delta_theta < 1e-9: # Very small rotation
        return q_prev # No significant change

    # Calculate the rotation axis
    rotation_axis = omega_body / np.linalg.norm(omega_body)

    # Create the incremental rotation quaternion (delta_q)
    delta_q = quaternion_from_axis_angle(rotation_axis, delta_theta)

    # Update the main quaternion by multiplying current q with the incremental rotation
    q_new = quaternion_multiply(q_prev, delta_q)

    # Normalization after multiplication is good practice, though delta_q is already unit
    return normalize_quaternion(q_new)

# --- Example Usage ---
if __name__ == "__main__":
    # Initial quaternion (no rotation, [1, 0, 0, 0] for scalar-first)
    q_initial = np.array([1.0, 0.0, 0.0, 0.0])

    # Angular velocity (e.g., rotating around Z-axis at 1 rad/s)
    omega_gyro = np.array([0.0, 0.0, 1.0]) # rad/s

    # Time step
    delta_t = 0.1 # seconds

    print(f"Initial Quaternion: {q_initial}")
    print(f"Angular Velocity: {omega_gyro} rad/s")
    print(f"Time Step: {delta_t} s\n")

    # Update the quaternion
    q_updated_exp_map = quaternion_update_exp_map(q_initial, omega_gyro, delta_t)

    print(f"Updated Quaternion (Method 2 - Exp Map): {q_updated_exp_map}")

    # Example 2: Multiple steps
    q_current_exp_map = np.array([1.0, 0.0, 0.0, 0.0])
    print("\n--- Multi-step update (Exp Map) ---")
    num_steps = 10
    total_time = num_steps * delta_t
    print(f"Simulating {num_steps} steps over {total_time} seconds with 1 rad/s around Z-axis.")
    for i in range(num_steps):
        q_current_exp_map = quaternion_update_exp_map(q_current_exp_map, omega_gyro, delta_t)
        print(f"Step {i+1} Q: {q_current_exp_map} | Norm: {np.linalg.norm(q_current_exp_map)}")
    print(f"Final Quaternion (Method 2 - Exp Map, after {num_steps} steps): {q_current_exp_map}")
    # Expected final rotation around Z by 1 rad: cos(0.5), 0, 0, sin(0.5)
    expected_q_z = np.array([np.cos(0.5 * total_time), 0, 0, np.sin(0.5 * total_time)])
    print(f"Expected final Z rotation by {total_time} rad: {expected_q_z}")