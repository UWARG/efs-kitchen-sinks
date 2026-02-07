import numpy as np
import pytest
from numpy.testing import assert_allclose

from pyekf.ESMEKF import ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL


def test_esmekf_constant_sensors_no_drift():
    # ----------------------------
    # Initial state
    # ----------------------------
    initial_position = np.array([0.0, 0.0, 0.0])
    initial_velocity = np.array([0.0, 0.0, 0.0])
    initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    # ----------------------------
    # Simulation parameters
    # ----------------------------
    dt = 0.01
    num_steps = 200
    total_time = num_steps * dt

    # Constant IMU measurements
    gyro_body = np.array([0.0, 0.0, 1.0])        # rad/s (yaw)
    accel_body = np.array([0.0, 0.0, -9.81])     # cancel gravity
    mag_body = MAGNETOMETER_INERTIAL.flatten()   # perfectly aligned

    # ----------------------------
    # Create filter
    # ----------------------------
    ekf = ESMEKF(
        gyro_initial=gyro_body,
        accel_initial=accel_body,
        mag_initial=mag_body,
        displacement_initial=initial_position,
        velocity_initial=initial_velocity,
        quaternion_initial=initial_quaternion,
        gyro_cov=1e-6,
        accel_cov=1e-6,
        magnetometer_cov=1e-6,
        gyro_bias_cov=1e-8,
        accel_bias_cov=1e-8,
        magnetometer_bias_cov=1e-8,
    )

    # ----------------------------
    # Run filter
    # ----------------------------
    for _ in range(num_steps):
        ekf.state_extrapolation(
            gyro_new=gyro_body,
            accel_new=accel_body,
            dt=dt,
        )

        ekf.correction_magnetometer(mag_body)

    # ----------------------------
    # Expected analytical solution
    # ----------------------------
    expected_position = initial_position
    expected_velocity = initial_velocity

    expected_quaternion = np.array([
        np.cos(0.5 * total_time),
        0.0,
        0.0,
        np.sin(0.5 * total_time),
    ])

    # ----------------------------
    # Assertions: nominal state
    # ----------------------------
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

    # ----------------------------
    # Assertions: error state
    # ----------------------------
    assert_allclose(
        ekf.error_state,
        np.zeros((18, 1)),
        atol=1e-6,
    )

    # ----------------------------
    # Assertions: covariance sanity
    # ----------------------------
    P = ekf.error_state_cov_mat

    # symmetric
    assert_allclose(P, P.T, atol=1e-10)

    # positive semi-definite (numerically)
    eigvals = np.linalg.eigvalsh(P)
    assert np.all(eigvals > -1e-10)

    # ----------------------------
    # Assertions: bias estimates
    # ----------------------------
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
