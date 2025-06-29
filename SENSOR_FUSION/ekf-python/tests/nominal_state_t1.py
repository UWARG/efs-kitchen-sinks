import numpy as np
from app.NominalState import NominalState
from app.utils import GRAVITY_INERTIAL

def test1():
    # Initial State
    initial_position = [0.0, 0.0, 0.0]  # meters
    initial_velocity = [0.0, 0.0, 0.0]  # meters/second
    initial_quaternion = [1.0, 0.0, 0.0, 0.0]  # identity quaternion (no rotation)

    # Simulation parameters
    dt = 0.1  # time step in seconds
    num_simulation_steps = 10 # Simulate for 1 second (100 * 0.01s)
    total_time = (num_simulation_steps) * dt

    # Example IMU measurements (constant rotation around Z and constant upward acceleration)
    # Rotating around Z-axis at 1 rad/s
    example_omega_body = np.array([0.0, 0.0, 1.0]) # rad/s
    # Accelerating upwards in (NED) body frame (e.g., a thrust)
    example_accel_body = np.array([0.0, 0.0, -10.0]) # m/s^2

    print(f"Simulating {num_simulation_steps} steps with dt = {dt} s...")
    print(f"Angular velocity: {example_omega_body} rad/s")
    print(f"Specific force: {example_accel_body} m/s^2")
    print("-" * 30)

    # Create the nominal state object
    nominal_state = NominalState(
        displacement=initial_position,
        velocity=initial_velocity,
        quaternion=initial_quaternion,
        prev_gyro_measurement=example_omega_body,
        prev_accel_measurement=example_accel_body
    )
    print("Initial Nominal State:")
    print(nominal_state)
    print("-" * 30)

    for i in range(num_simulation_steps):
        # Update the nominal state
        nominal_state.update(
            example_omega_body,
            example_accel_body,
            dt
        )
    
    print(f"State after {num_simulation_steps} steps (t = {(num_simulation_steps) * dt:.2f} s):")
    print(nominal_state)

    expected_p = initial_position + 1/2 * total_time**2 * (example_accel_body + GRAVITY_INERTIAL.flatten())
    expected_v = initial_velocity + total_time * (example_accel_body + GRAVITY_INERTIAL.flatten())
    expected_q = np.array([np.cos(0.5 * total_time), 0, 0, np.sin(0.5 * total_time)])
    print(f"Expected final position: {expected_p}")
    print(f"Expected final velocity: {expected_v}")
    print(f"Expected final Z rotation by {total_time} rad: {expected_q}")
    print("-" * 30)

    print("\nSimulation Complete.")