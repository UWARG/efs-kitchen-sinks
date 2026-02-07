import numpy as np
import pytest
from numpy.testing import assert_allclose

from pyekf.ESMEKF import ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL


def test_esmekf_constant_imu_no_drift_no_mag():
    # Initial state
    initial_position = np.array([0.0, 0.0, 0.0])
    initial_velocity = np.array([0.0, 0.0, 0.0])
    initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

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

    # Create filter
    ekf = ESMEKF(
        gyro_initial=example_omega_body,
        accel_initial=example_accel_body,
        displacement_initial=initial_position,
        velocity_initial=initial_velocity,
        quaternion_initial=initial_quaternion,
        gyro_cov=1e-6,
        accel_cov=1e-6,
        gyro_bias_cov=1e-8,
        accel_bias_cov=1e-8,
    )

    print("Initial state:")
    print(ekf)
    print("-" * 50)

    # Run filter
    for _ in range(num_simulation_steps):
        ekf.state_extrapolation(
            gyro_new=example_omega_body,
            accel_new=example_accel_body,
            dt=dt,
        )

    # ---- Expected analytical results ----

    accel_total = example_accel_body + GRAVITY_INERTIAL.flatten()

    expected_position = (
        initial_position
        + 0.5 * total_time**2 * accel_total
    )

    expected_velocity = (
        initial_velocity
        + total_time * accel_total
    )

    expected_quaternion = np.array([
        np.cos(0.5 * total_time),
        0.0,
        0.0,
        np.sin(0.5 * total_time),
    ])

    print("\nExpected results:")
    print(f"Expected displacement: {expected_position}")
    print(f"Expected velocity:     {expected_velocity}")
    print(f"Expected quaternion:   {expected_quaternion}")
    print("-" * 50)

    print("Final estimated state:")
    print(f"Displacement: {ekf.nominal_state.displacement_new.flatten()}")
    print(f"Velocity:     {ekf.nominal_state.velocity_new.flatten()}")
    print(f"Quaternion:   {ekf.nominal_state.quaternion_new.flatten()}")
    print("-" * 50)

    # Assertions: nominal state
    assert_allclose(
        ekf.nominal_state.displacement_new.flatten(),
        expected_position,
        atol=1e-6,
    )

    assert_allclose(
        ekf.nominal_state.velocity_new.flatten(),
        expected_velocity,
        atol=1e-6,
    )

    # Quaternion sign ambiguity
    q_est = ekf.nominal_state.quaternion_new.flatten()
    if np.dot(q_est, expected_quaternion) < 0:
        q_est = -q_est

    assert_allclose(
        q_est,
        expected_quaternion,
        atol=1e-5,
    )

    # Assertions: error state
    assert_allclose(
        ekf.error_state,
        np.zeros((18, 1)),
        atol=1e-6,
    )

    # Assertions: covariance sanity
    P = ekf.error_state_cov_mat

    # symmetric
    assert_allclose(P, P.T, atol=1e-10)

    # positive semi-definite (numerically)
    eigvals = np.linalg.eigvalsh(P)
    assert np.all(eigvals > -1e-10)

    # Assertions: bias estimates
    assert_allclose(
        ekf.measurements.gyro_bias_accumulated,
        np.zeros((3, 1)),
        atol=1e-6,
    )

    assert_allclose(
        ekf.measurements.accel_bias_accumulated,
        np.zeros((3, 1)),
        atol=1e-6,
    )

    assert_allclose(
        ekf.measurements.mag_bias_accumulated,
        np.zeros((3, 1)),
        atol=1e-6,
    )
