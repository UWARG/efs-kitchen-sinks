import numpy as np
import pytest

from pyekf.ESMEKF import ESMEKF
from tests.sim.trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator

def test_y_axis_rotation_90_degrees():
    # 1. Setup Ground Truth
    # Spin at pi/4 rad/s around Y (90 degrees in 2 seconds)
    omega_y = np.pi / 4
    traj = ConstantMotionTrajectory(
        p0=np.zeros(3),
        v0=np.zeros(3),
        q0=np.array([1.0, 0.0, 0.0, 0.0]),
        accel_body=np.zeros(3),
        omega_body=np.array([0.0, omega_y, 0.0])
    )
    sim = SensorSimulator(traj)

    # 2. Initialize Filter
    # Get initial readings at t=0
    g0, a0, m0 = sim.get_readings(0)
    ekf = ESMEKF(
        gyro_initial=g0, accel_initial=a0,
        displacement_initial=np.zeros(3),
        velocity_initial=np.zeros(3),
        quaternion_initial=np.array([1.0, 0.0, 0.0, 0.0])
    )

    # 3. Run Simulation
    dt = 0.01
    duration = 2.0
    for t in np.arange(dt, duration + dt, dt):
        gyro, accel, mag = sim.get_readings(t)
        ekf.state_extrapolation(gyro, accel, dt)
        # Optional: ekf.measurement_update(mag) if you've implemented it

    # 4. Deterministic Verification
    final_p, final_v, final_q = traj.get_state(duration)
    
    # At t=2, angle = (pi/4) * 2 = pi/2 (90 degrees)
    # Expected Quat: [cos(pi/4), 0, sin(pi/4), 0] = [0.707, 0, 0.707, 0]
    expected_q = np.array([np.cos(np.pi/4), 0.0, np.sin(np.pi/4), 0.0])

    # Assertions
    np.testing.assert_allclose(ekf.nominal_state.quaternion_new.flatten(), expected_q, atol=1e-5)
    np.testing.assert_allclose(ekf.nominal_state.displacement_new.flatten(), [0, 0, 0], atol=1e-5)
