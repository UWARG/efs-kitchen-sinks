import numpy as np
import pytest
from numpy.testing import assert_allclose

from tests.utils.utils import assert_quaternion_close
from pyekf.ins_esmekf.NominalState import NominalState
from pyekf.Measurements import Measurements
from pyekf.utils import GRAVITY_INERTIAL


def test_nominal_state_constant_rotation_and_acceleration():
    # Initial State
    initial_position = np.array([0.0, 0.0, 0.0])
    initial_velocity = np.array([0.0, 0.0, 0.0])
    initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)

    # Simulation parameters
    dt = 0.1
    num_simulation_steps = 10
    total_time = num_simulation_steps * dt

    # IMU measurements are constant
    example_omega_body = np.array([0.0, 0.0, 1.0])      # rad/s
    example_accel_body = np.array([0.0, 0.0, -10.0])   # m/s^2

    print("\n--- Nominal State Propagation Test ---")
    print(f"dt = {dt}, steps = {num_simulation_steps}, total_time = {total_time}")
    print(f"Gyro (body): {example_omega_body}")
    print(f"Accel (body): {example_accel_body}")
    print("-" * 50)

    # Create state
    measurements = Measurements(
        gyro_initial=example_omega_body,
        accel_initial=example_accel_body,
    )

    nominal_state = NominalState(
        displacement_initial=initial_position,
        velocity_initial=initial_velocity,
        quaternion_initial=initial_quaternion,
    )

    print("Initial state:")
    print(nominal_state)
    print("-" * 50)

    # Propagate state
    for i in range(num_simulation_steps):
        measurements.update_gyro(example_omega_body)
        measurements.update_accel(example_accel_body)
        nominal_state.state_extrapolation(
            gyro_new=measurements.gyro_new,
            gyro_prev=measurements.gyro_prev,
            accel_new=measurements.accel_new,
            accel_prev=measurements.accel_prev,
            dt=dt
        )
        print(f"After step {i + 1}:")
        print(nominal_state)
        print("-" * 30)

    # ---- Expected analytical results ----

    accel_total = example_accel_body + GRAVITY_INERTIAL.flatten()

    expected_p = (
        initial_position
        + 0.5 * total_time**2 * accel_total
    )

    expected_v = (
        initial_velocity
        + total_time * accel_total
    )

    expected_q = np.array([
        np.cos(0.5 * total_time),
        0.0,
        0.0,
        np.sin(0.5 * total_time),
    ])

    print("\nExpected results:")
    print(f"Expected displacement: {expected_p}")
    print(f"Expected velocity:     {expected_v}")
    print(f"Expected quaternion:   {expected_q}")
    print("-" * 50)

    print("Final estimated state:")
    print(f"Displacement: {nominal_state.displacement_new.flatten()}")
    print(f"Velocity:     {nominal_state.velocity_new.flatten()}")
    print(f"Quaternion:   {nominal_state.quaternion_new.flatten()}")
    print("-" * 50)

    # ---- Assertions ----

    assert_allclose(
        nominal_state.displacement_new.flatten(),
        expected_p,
        rtol=1e-6,
        atol=1e-8,
    )

    assert_allclose(
        nominal_state.velocity_new.flatten(),
        expected_v,
        rtol=1e-6,
        atol=1e-8,
    )

    assert_quaternion_close(
        estimate=nominal_state.quaternion_new.flatten(),
        actual=expected_q,
        rtol=1e-6,
        atol=1e-8
    )

    print("Test passed: Nominal state matches analytical solution.")
