import numpy as np
import pytest

from pyekf.ESMEKF import ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator
from tests.sim.params import ConstantSimParams, SensorParams

seed: int = 42

def test_constant_rotation_no_noise_no_correction():
    # 1. Setup Parameter Objects
    sensor_params = SensorParams(
        gyro_cov=0.0,
        accel_cov=0.0,
        magnetometer_cov=0.0,
        gyro_bias=np.zeros((3, 1)),
        accel_bias=np.zeros((3, 1)),
        magnetometer_bias=np.zeros((3, 1))
    )

    sim_params = ConstantSimParams(
        delta_t=0.01,
        duration=2.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=np.array([[0.0], [np.pi / 4], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL,
        mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    # 2. Setup Ground Truth and Simulator
    trajectory = ConstantMotionTrajectory(
        displacement_initial_iframe=sim_params.displacement_initial_iframe,
        velocity_initial_iframe=sim_params.velocity_initial_iframe,
        quaternion_initial_iframe=sim_params.quaternion_initial_iframe,
        accel_constant_iframe=sim_params.accel_constant_iframe,
        angular_vel_constant_iframe=sim_params.angular_vel_constant_iframe
    )
    
    simulator = SensorSimulator(
        trajectory=trajectory, 
        gyro_cov=sensor_params.gyro_cov, 
        accel_cov=sensor_params.accel_cov, 
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias=sensor_params.gyro_bias,
        accel_bias=sensor_params.accel_bias,
        magnetometer_bias=sensor_params.magnetometer_bias,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        seed=seed
    )

    # 3. Initialize Filter
    gyro_initial, accel_initial, mag_initial = simulator.get_readings(0.0)
    ekf = ESMEKF(
        gyro_initial=gyro_initial, 
        accel_initial=accel_initial,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe
    )

    # 4. Run Simulation Loop
    delta_t = sim_params.delta_t
    duration = sim_params.duration
    
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for time_step in np.arange(delta_t, duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(time_step)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        
        gt_displacement, gt_velocity, gt_accel, gt_quaternion, gt_angular_vel = trajectory.get_state(time_step)
        
        est_displacement = ekf.nominal_state.displacement_new
        est_velocity = ekf.nominal_state.velocity_new
        est_quaternion = ekf.nominal_state.quaternion_new

        print(f"{time_step:5.2f} | EKF   | {est_quaternion.flatten()} | {est_displacement.flatten()} | {est_velocity.flatten()}")
        print(f"{' ':>5} | GT    | {gt_quaternion.flatten()} | {gt_displacement.flatten()} | {gt_velocity.flatten()}")
        print("-" * 110)

    # 5. Verification
    expected_displacement, expected_velocity, expected_accel, expected_quaternion, expected_angular_vel = trajectory.get_state(duration)
    
    actual_displacement = ekf.nominal_state.displacement_new
    actual_velocity = ekf.nominal_state.velocity_new
    actual_quaternion = ekf.nominal_state.quaternion_new

    np.testing.assert_allclose(actual_quaternion, expected_quaternion, atol=1e-5)
    np.testing.assert_allclose(actual_displacement, expected_displacement, atol=1e-5)
    np.testing.assert_allclose(actual_velocity, expected_velocity, atol=1e-5)
