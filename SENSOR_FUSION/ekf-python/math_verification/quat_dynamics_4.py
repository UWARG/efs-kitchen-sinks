import numpy as np

def normalize_quaternion(q):
    """Normalizes a quaternion."""
    norm = np.linalg.norm(q)
    if norm < 1e-9: # Handle very small quaternions (e.g., zero vector)
        return np.array([1.0, 0.0, 0.0, 0.0]) # Return identity quaternion
    return q / norm

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
    return np.array([w, x, y, z]) # Normalization is done in the update function

def quaternion_update_euler(q_prev, omega_body, dt):
    """
    Updates a quaternion using the simple Euler integration method.

    Args:
        q_prev (np.array): Previous quaternion [w, x, y, z]. Must be normalized.
        omega_body (np.array): Angular velocity vector [wx, wy, wz] in body frame (rad/s).
        dt (float): Time step in seconds.

    Returns:
        np.array: Updated quaternion [w, x, y, z].
    """
    # 1. Represent angular velocity as a pure quaternion [0, wx, wy, wz]
    omega_q = np.array([0.0, omega_body[0], omega_body[1], omega_body[2]])

    # 2. Calculate q_dot * dt
    # q_dot = 0.5 * q_prev * omega_q (quaternion multiplication)
    q_dot_dt_term = 0.5 * quaternion_multiply(q_prev, omega_q) * dt

    # 3. Euler integration step: q_new = q_prev + q_dot_dt_term
    q_new_unnormalized = q_prev + q_dot_dt_term

    # 4. Normalize the new quaternion to keep it a unit quaternion
    return normalize_quaternion(q_new_unnormalized)

# --- Example Usage ---
if __name__ == "__main__":
    # Initial quaternion (no rotation, [1, 0, 0, 0] for scalar-first)
    q_initial = np.array([1.0, 0.0, 0.0, 0.0])

    # Angular velocity (e.g., rotating around Z-axis at 1 rad/s)
    omega_gyro = np.array([0.0, 0.0, 1.0]) # rad/s

    # Time step
    delta_t = 0.01 # seconds (smaller delta_t improves Euler accuracy)

    print(f"Initial Quaternion: {q_initial}")
    print(f"Angular Velocity: {omega_gyro} rad/s")
    print(f"Time Step: {delta_t} s\n")

    # Single step update
    q_updated_euler_single_step = quaternion_update_euler(q_initial, omega_gyro, delta_t)
    print(f"Updated Quaternion (Euler, single step): {q_updated_euler_single_step}")
    print(f"Norm: {np.linalg.norm(q_updated_euler_single_step)}\n")

    # Multi-step example for the Euler method
    q_current_euler = np.array([1.0, 0.0, 0.0, 0.0])
    num_steps = 100 # More steps to observe drift/error
    total_time = num_steps * delta_t # Should be 1.0 seconds for this example

    print(f"--- Multi-step update (Simple Euler Integration) ---")
    print(f"Simulating {num_steps} steps over {total_time} seconds with 1 rad/s around Z-axis.")

    for i in range(num_steps):
        q_current_euler = quaternion_update_euler(q_current_euler, omega_gyro, delta_t)
        # print(f"Step {i+1} Q: {q_current_euler} | Norm: {np.linalg.norm(q_current_euler)}")

    print(f"Final Quaternion (Euler, after {num_steps} steps): {q_current_euler}")
    print(f"Final Norm: {np.linalg.norm(q_current_euler)}")

    # --- Compare with the (more accurate) closed-form solution ---
    # The true rotation over `total_time` would be a single rotation by `total_time` radians
    # around the Z-axis.
    expected_q_z_angle = np.linalg.norm(omega_gyro) * total_time # 1 rad
    half_expected_angle = expected_q_z_angle / 2.0
    expected_q_z = np.array([np.cos(half_expected_angle), 0, 0, np.sin(half_expected_angle)])

    print(f"\nExpected final Quaternion (Closed-form): {expected_q_z}")
    print(f"Difference (Euler vs. Closed-form): {np.linalg.norm(q_current_euler - expected_q_z)}")
    print("Notice the difference, which indicates the error accumulated by Euler integration.")

    # --- Demonstrate how error changes with larger dt ---
    print("\n--- Euler with Larger Time Step (Demonstrates more error) ---")
    large_dt = 0.1 # seconds
    q_large_dt_euler = np.array([1.0, 0.0, 0.0, 0.0])
    num_steps_large_dt = 10 # total_time will still be 1.0s
    for i in range(num_steps_large_dt):
        q_large_dt_euler = quaternion_update_euler(q_large_dt_euler, omega_gyro, large_dt)
    print(f"Final Quaternion (Euler, dt={large_dt}s): {q_large_dt_euler}")
    print(f"Difference (Euler vs. Closed-form, large dt): {np.linalg.norm(q_large_dt_euler - expected_q_z)}")
    print("The error is larger with a larger time step.")
