import numpy as np
import pytest
from numpy.testing import assert_allclose

from pyekf.NominalState import NominalState
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

    # IMU measurements
    example_omega_body = np.array([0.0, 0.0, 1.0])      # rad/s
    example_accel_body = np.array([0.0, 0.0, -10.0])   # m/s^2

    # Create nominal state
    nominal_state = NominalState(
        displacement=initial_position,
        velocity=initial_velocity,
        quaternion=initial_quaternion,
        prev_gyro_measurement=example_omega_body,
        prev_accel_measurement=example_accel_body,
    )

    # Propagate state
    for _ in range(num_simulation_steps):
        nominal_state.update(
            example_omega_body,
            example_accel_body,
            dt
        )

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

    # ---- Assertions ----

    assert_allclose(
        nominal_state.prev_displacement.flatten(),
        expected_p,
        rtol=1e-6,
        atol=1e-8,
    )

    assert_allclose(
        nominal_state.prev_velocity.flatten(),
        expected_v,
        rtol=1e-6,
        atol=1e-8,
    )

    # Quaternion sign ambiguity: q and -q represent same rotation
    q_est = nominal_state.prev_quaternion.flatten()
    if np.dot(q_est, expected_q) < 0:
        q_est = -q_est

    assert_allclose(
        q_est,
        expected_q,
        rtol=1e-6,
        atol=1e-8,
    )
