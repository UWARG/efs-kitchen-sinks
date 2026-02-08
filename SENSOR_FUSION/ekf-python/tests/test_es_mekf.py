import numpy as np
import pytest

from pyekf.ESMEKF import ESMEKF
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator

def test_y_axis_rotation_90_degrees():
    # 1. Setup Ground Truth (Constant rotation in I-frame)
    # 90 degrees in 2 seconds -> pi/4 rad/s around Y-axis
    omega_y = np.pi / 4
    angular_vel_i = np.array([[0.0], [omega_y], [0.0]])
    
    traj = ConstantMotionTrajectory(
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=angular_vel_i
    )
    
    # Initialize simulator with zero noise for deterministic verification
    sim = SensorSimulator(traj, gyro_cov=0.0, accel_cov=0.0, magnetometer_cov=0.0)

    # 2. Initialize Filter
    g0, a0, m0 = sim.get_readings(0.0)
    ekf = ESMEKF(
        gyro_initial=g0, 
        accel_initial=a0,
        displacement_initial=np.zeros((3, 1)),
        velocity_initial=np.zeros((3, 1)),
        quaternion_initial=np.array([[1.0], [0.0], [0.0], [0.0]])
    )

    # 3. Run Simulation Loop
    dt = 0.01
    duration = 2.0
    for t in np.arange(dt, duration + dt, dt):
        gyro, accel, mag = sim.get_readings(t)
        ekf.state_extrapolation(gyro, accel, dt)

    # 4. Verification
    # Expected state at t=2.0 (Rotation: pi/2 around Y)
    expected_p = np.zeros((3, 1))
    expected_v = np.zeros((3, 1))
    expected_q = np.array([[np.cos(np.pi/4)], [0.0], [np.sin(np.pi/4)], [0.0]])
    
    # Retrieve current nominal state
    actual_p = ekf.nominal_state.displacement_new
    actual_v = ekf.nominal_state.velocity_new
    actual_q = ekf.nominal_state.quaternion_new

    # Assertions using column vector shapes
    np.testing.assert_allclose(actual_q, expected_q, atol=1e-5)
    np.testing.assert_allclose(actual_p, expected_p, atol=1e-5)
    np.testing.assert_allclose(actual_v, expected_v, atol=1e-5)
