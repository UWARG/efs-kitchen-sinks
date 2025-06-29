import numpy as np
from NominalState import NominalState

def main():
    # Simulation parameters
    dt = 0.01  # time step in seconds
    num_simulation_steps = 100 # Simulate for 1 second (100 * 0.01s)

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
    print("-" * 30)

    print("\nSimulation Complete.")


if __name__ == "__main__":
    main()
