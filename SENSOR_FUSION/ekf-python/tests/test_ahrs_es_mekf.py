import numpy as np
import pytest

from pyekf.AHRS_ESMEKF import AHRS_ESMEKF 
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from tests.utils import assert_quaternion_close
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator
from tests.sim.params import ConstantSimParams, SensorParams

seed: int = 42

def test_constant_rotation_and_accel_no_noise_no_correction():
    """
    BASELINE: Zero noise, zero bias, and NO correction updates.
    Tests if the kinematic integration (state extrapolation) alone 
    can perfectly track a constant rotation.
    """
    sim_params = ConstantSimParams(
        delta_t=0.01,
        duration=2.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[5.0], [3.2], [1.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi / 4], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL,
        mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory = ConstantMotionTrajectory(
        displacement_initial_iframe=sim_params.displacement_initial_iframe,
        velocity_initial_iframe=sim_params.velocity_initial_iframe,
        quaternion_initial_iframe=sim_params.quaternion_initial_iframe,
        accel_constant_iframe=sim_params.accel_constant_iframe,
        angular_vel_constant_iframe=sim_params.angular_vel_constant_iframe
    )
    
    simulator = SensorSimulator(
        trajectory=trajectory, 
        gyro_cov=0.0, accel_cov=0.0, magnetometer_cov=0.0,
        gyro_bias_cov=0.0, accel_bias_cov=0.0, magnetometer_bias_cov=0.0,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        seed=seed
    )

    gyro_init, accel_init, mag_init = simulator.get_readings(0.0)
    ekf = AHRS_ESMEKF(
        gyro_initial=gyro_init, accel_initial=accel_init, mag_initial=mag_init,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe
    )

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, _ = simulator.get_readings(curr_time)
        # ONLY extrapolation
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)

    gt_disp, gt_vel, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    
    # Should be precise to 4 decimal places
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=1e-4)
    np.testing.assert_allclose(ekf.nominal_state.velocity_new, gt_vel, atol=1e-4)

def test_constant_rotation_and_accel_no_bias_no_correction():
    """
    STOCHASTIC BASELINE: Added measurement noise but still NO correction updates.
    Expect the orientation to drift away from Ground Truth over time.
    """
    sensor_params = SensorParams(
        gyro_cov=0.01, accel_cov=0.01, magnetometer_cov=0,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=2.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[5.0], [3.2], [1.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi / 4], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory = ConstantMotionTrajectory(
        displacement_initial_iframe=sim_params.displacement_initial_iframe,
        velocity_initial_iframe=sim_params.velocity_initial_iframe,
        quaternion_initial_iframe=sim_params.quaternion_initial_iframe,
        accel_constant_iframe=sim_params.accel_constant_iframe,
        angular_vel_constant_iframe=sim_params.angular_vel_constant_iframe
    )
    
    simulator = SensorSimulator(
        trajectory=trajectory, 
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov, 
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe, seed=seed
    )

    gyro_init, accel_init, mag_init = simulator.get_readings(0.0)
    ekf = AHRS_ESMEKF(
        gyro_initial=gyro_init, accel_initial=accel_init, mag_initial=mag_init,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov
    )

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, _ = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)

    _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)

def test_constant_rotation_and_accel_no_bias():
    """
    FULL AHRS (No Bias): Noise is present, and we use corrections to fix drift.
    Tests if Mag/Accel updates successfully pull the noisy estimate back to Ground Truth.
    """
    sensor_params = SensorParams(
        gyro_cov=0.1, accel_cov=0.1, magnetometer_cov=0.0001,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.1, duration=10.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=np.array([[0.0], [np.pi], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory = ConstantMotionTrajectory(
        displacement_initial_iframe=sim_params.displacement_initial_iframe,
        velocity_initial_iframe=sim_params.velocity_initial_iframe,
        quaternion_initial_iframe=sim_params.quaternion_initial_iframe,
        accel_constant_iframe=sim_params.accel_constant_iframe,
        angular_vel_constant_iframe=sim_params.angular_vel_constant_iframe
    )
    
    simulator = SensorSimulator(
        trajectory=trajectory, 
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov, 
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe, seed=seed
    )

    gyro_init, accel_init, mag_init = simulator.get_readings(0.0)
    ekf = AHRS_ESMEKF(
        gyro_initial=gyro_init, accel_initial=accel_init, mag_initial=mag_init,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov,
        magnetometer_cov=sensor_params.magnetometer_cov
    )

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        ekf.correction_accelerometer(accel_new=accel_reading)

    _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    
    # With corrections, we expect to be within 0.2 even with high noise over 10s
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)

def test_ahrs_full_correction_constant_rotation():
    """
    Tests the 9-state AHRS_ESMEKF with Gyro, Magnetometer, AND Accelerometer updates.
    Includes telemetry logging to verify K-Gain and convergence over 10 seconds.
    """
    # 1. Define Simulation Parameters
    sensor_params = SensorParams(
        gyro_cov=0.01,
        accel_cov=0.1,
        magnetometer_cov=0.01,
        gyro_bias_cov=1e-5,
        accel_bias_cov=1e-4,
        magnetometer_bias_cov=0.0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01,
        duration=10.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)), 
        angular_vel_constant_iframe=np.array([[0.0], [0.1], [0.5]]), 
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
        gyro_bias_cov=sensor_params.gyro_bias_cov,
        accel_bias_cov=sensor_params.accel_bias_cov,
        magnetometer_bias_cov=sensor_params.magnetometer_bias_cov,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        seed=seed
    )

    # 3. Initialize Filter
    gyro_initial, accel_initial, mag_initial = simulator.get_readings(0.0)
    
    ekf = AHRS_ESMEKF(
        gyro_initial=gyro_initial, 
        accel_initial=accel_initial,
        mag_initial=mag_initial,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov,
        accel_cov=sensor_params.accel_cov,
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=sensor_params.gyro_bias_cov,
        accel_bias_cov=sensor_params.accel_bias_cov,
        magnetometer_bias_cov=sensor_params.magnetometer_bias_cov,
        p_init_att=0.1,
        p_init_bias=0.01
    )

    # 4. Run Simulation Loop
    delta_t = sim_params.delta_t
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        ekf.correction_accelerometer(accel_new=accel_reading)

        if curr_time % 0.5 < delta_t:
            gt_disp, gt_vel, _, gt_quat, _ = trajectory.get_state(curr_time)
            est_disp = ekf.nominal_state.displacement_new
            est_vel = ekf.nominal_state.velocity_new
            est_quat = ekf.nominal_state.quaternion_new
            
            kalman_gain_norm = np.linalg.norm(ekf.kalman_gain, ord='fro')
            
            print(f"Time: {curr_time:5.2f} | K-Gain Norm: {kalman_gain_norm:8.4f}")
            print(f"{curr_time:5.2f} | EKF   | {fmt_vec(est_quat)} | {fmt_vec(est_disp)} | {fmt_vec(est_vel)}")
            print(f"{' ':>5} | GT    | {fmt_vec(gt_quat)} | {fmt_vec(gt_disp)} | {fmt_vec(gt_vel)}")
            print("-" * 130)

    # 5. Verification
    _, _, _, expected_quat, _ = trajectory.get_state(sim_params.duration)
    assert_quaternion_close(actual=expected_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)
