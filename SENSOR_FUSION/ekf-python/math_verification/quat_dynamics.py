import numpy as np
from pyquaternion import Quaternion
from numpy.typing import NDArray
from scipy.linalg import expm

def skew_symmetric_matrix(omega):
    """
    Constructs the 4x4 skew-symmetric matrix Omega(omega) for quaternion multiplication.
    omega = [omega_x, omega_y, omega_z]
    """
    omega_x, omega_y, omega_z = omega[0], omega[1], omega[2]
    return np.array([
        [0, -omega_x, -omega_y, -omega_z],
        [omega_x, 0, omega_z, -omega_y],
        [omega_y, -omega_z, 0, omega_x],
        [omega_z, omega_y, -omega_x, 0]
    ])

def quaternion_update_expm(q_prev, omega_body, dt):
    """
    Updates a quaternion using the matrix exponential method.

    Args:
        q_prev (np.array): Previous quaternion [w, x, y, z].
        omega_body (np.array): Angular velocity vector [wx, wy, wz] in body frame (rad/s).
        dt (float): Time step in seconds.

    Returns:
        np.array: Updated quaternion [w, x, y, z].
    """
    # Ensure q_prev is a column vector for matrix multiplication
    q_prev = np.asarray(q_prev).reshape(4, 1)

    # Construct the Omega matrix
    Omega = skew_symmetric_matrix(omega_body)

    # Calculate the matrix exponential term
    exp_term = expm(0.5 * Omega * dt)

    # Update the quaternion
    q_new = np.dot(exp_term, q_prev)

    # Normalize the new quaternion (important due to floating point errors, even if theoretically not needed)
    return q_new / np.linalg.norm(q_new)


def main():
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
    q_updated = quaternion_update_expm(q_initial, omega_gyro, delta_t)

    print(f"Updated Quaternion (Method 1 - expm): {q_updated.flatten()}")

    # Example 2: Multiple steps
    q_current = np.array([1.0, 0.0, 0.0, 0.0])
    print("\n--- Multi-step update ---")
    num_steps = 10
    total_time = num_steps * delta_t
    print(f"Simulating {num_steps} steps over {total_time} seconds with 1 rad/s around Z-axis.")
    for i in range(num_steps):
        q_current = quaternion_update_expm(q_current, omega_gyro, delta_t)
        print(f"Step {i+1} Q: {q_current.flatten()} | Norm: {np.linalg.norm(q_current)}")
    print(f"Final Quaternion (Method 1 - expm, after {num_steps} steps): {q_current.flatten()}")
    # Expected final rotation around Z by 1 rad: cos(0.5), 0, 0, sin(0.5)
    expected_q_z = np.array([np.cos(0.5 * total_time), 0, 0, np.sin(0.5 * total_time)])
    print(f"Expected final Z rotation by {total_time} rad: {expected_q_z}")


if __name__ == "__main__":
    main()
