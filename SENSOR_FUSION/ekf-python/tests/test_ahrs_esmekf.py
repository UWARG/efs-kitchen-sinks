import numpy as np
import pytest

from pyekf.ahrs_esmekf.AHRS_ESMEKF import AHRS_ESMEKF 
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from tests.utils.utils import assert_quaternion_close
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator
from tests.utils.grapher import ResultsCollector
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
        duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[0.0], [0.0], [0.0]]), # must be zero for all ahrs esmekf tests cause doesn't predict velocity or position
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
        gyro_initial=gyro_init,
        accel_initial=accel_init,
        mag_initial=mag_init,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe
    )

    delta_t = sim_params.delta_t
    print(f"\n{'Time':>5} | {'Gyro [x y z]':^30} | {'Accel [x y z]':^30}")
    print("-" * 70)
    print(f"Time: {0.00:5.2f} | Gyro: {gyro_init.flatten()} | Accel: {accel_init.flatten()}")
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, _ = simulator.get_readings(curr_time)
        print(f"Time: {curr_time:.2f} | Gyro: {gyro_reading.flatten()} | Accel: {accel_reading.flatten()}")
        ekf.state_extrapolation(gyro_reading, delta_t)


    _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    
    # Should be precise to 4 decimal places
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=1e-4)


def test_constant_rotation_and_accel_no_bias_no_correction(plots_dir):
    """
    STOCHASTIC BASELINE: Added measurement noise but still NO correction updates.
    Expect the orientation to drift away from Ground Truth over time.
    """
    sensor_params = SensorParams(
        gyro_cov=0.05, accel_cov=0.001, magnetometer_cov=0.01,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[5.0], [3.2], [1.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi], [0.02]]),
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
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov
    )

    grapher = ResultsCollector(
        title="AHRS_ESMEKF: (2)test_constant_rotation_and_accel_no_bias_no_correction",
        metadata={
            "dt": sim_params.delta_t,
            "T_total": sim_params.duration,
            "gyro_var": sensor_params.gyro_cov,
            "accel_var": sensor_params.accel_cov,
            "mag_var": sensor_params.magnetometer_cov,
            "gyro_bias_cov": sensor_params.gyro_bias_cov,
            "accel_bias_cov": sensor_params.accel_bias_cov,
            "mag_bias_cov": sensor_params.magnetometer_bias_cov,
        },
    )
    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, _, _ = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, delta_t)
        grapher.collect(curr_time, trajectory.get_state(curr_time), ekf.nominal_state)

    grapher.save_and_show(output_dir=plots_dir, filename="2_ahrs_no_bias_no_correction.png")

    # _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    # assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)

def test_constant_rotation_and_accel_no_bias(plots_dir):
    """
    FULL AHRS (No Bias): Noise is present, and we use corrections to fix drift.
    Tests if Mag/Accel updates successfully pull the noisy estimate back to Ground Truth.
    """
    sensor_params = SensorParams(
        gyro_cov=0.05, accel_cov=0.001, magnetometer_cov=0.01,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=400.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[0.0], [0.0], [0.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi], [0.02]]),
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
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov, accel_cov=sensor_params.accel_cov,
        magnetometer_cov=sensor_params.magnetometer_cov
    )

    grapher = ResultsCollector(
        title="AHRS_ESMEKF: (3)test_constant_rotation_and_accel_no_bias",
        metadata={
            "dt": sim_params.delta_t,
            "T_total": sim_params.duration,
            "gyro_var": sensor_params.gyro_cov,
            "accel_var": sensor_params.accel_cov,
            "mag_var": sensor_params.magnetometer_cov,
            "gyro_bias_cov": sensor_params.gyro_bias_cov,
            "accel_bias_cov": sensor_params.accel_bias_cov,
            "mag_bias_cov": sensor_params.magnetometer_bias_cov,
        },
    )
    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        
        ekf.state_extrapolation(gyro_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        ekf.correction_accelerometer(accelerometer_new=accel_reading)
        grapher.collect(curr_time, trajectory.get_state(curr_time), ekf.nominal_state)
    
    grapher.save_and_show(output_dir=plots_dir, filename="3_ahrs_no_bias.png")

    _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    
    # With corrections, we expect to be within 0.2 even with high noise over 10s
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)

def test_ahrs_full_correction_constant_rotation(plots_dir):
    """
    Tests the 9-state AHRS_ESMEKF with Gyro, Magnetometer, AND Accelerometer updates.
    Includes telemetry logging to verify K-Gain and convergence over 10 seconds.
    """
    # 1. Define Simulation Parameters
    sensor_params = SensorParams(
        gyro_cov=0.05, accel_cov=0.001, magnetometer_cov=0.01,
        gyro_bias_cov=0.01, accel_bias_cov=0.01, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=300.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[0.0], [0.0], [0.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi], [0.02]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
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
    print(f"Initial Readings -> Gyro: {gyro_initial.flatten()}, Accel: {accel_initial.flatten()}, Mag: {mag_initial.flatten()}")
    ekf = AHRS_ESMEKF(
        gyro_initial=gyro_initial, 
        accel_initial=accel_initial,
        mag_initial=mag_initial,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov,
        accel_cov=sensor_params.accel_cov,
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=sensor_params.gyro_bias_cov,
        accel_bias_cov=sensor_params.accel_bias_cov,
        # p_init_att=0.1,
        # p_init_bias=0.01
    )

    # 4. Run Simulation Loop
    grapher = ResultsCollector(
        title="AHRS_ESMEKF: (4)test_ahrs_full_correction_constant_rotation",
        metadata={
            "dt": sim_params.delta_t,
            "T_total": sim_params.duration,
            "gyro_var": sensor_params.gyro_cov,
            "accel_var": sensor_params.accel_cov,
            "mag_var": sensor_params.magnetometer_cov,
            "gyro_bias_cov": sensor_params.gyro_bias_cov,
            "accel_bias_cov": sensor_params.accel_bias_cov,
            "mag_bias_cov": sensor_params.magnetometer_bias_cov,
        },
    )
    delta_t = sim_params.delta_t
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        
        ekf.state_extrapolation(gyro_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        ekf.correction_accelerometer(accelerometer_new=accel_reading)
        
        grapher.collect(curr_time, trajectory.get_state(curr_time), ekf.nominal_state)

        if curr_time % 0.5 < delta_t:
            _, _, _, gt_quat, _ = trajectory.get_state(curr_time)
            est_quat = ekf.nominal_state.quaternion_new
            
            kalman_gain_norm = np.linalg.norm(ekf.kalman_gain, ord='fro')
            
            print(f"Time: {curr_time:5.2f} | K-Gain Norm: {kalman_gain_norm:8.4f}")
            print(f"{curr_time:5.2f} | EKF   | {fmt_vec(est_quat)}")
            print(f"{' ':>5} | GT    | {fmt_vec(gt_quat)}")
            print("-" * 130)

    grapher.save_and_show(output_dir=plots_dir, filename="4_ahrs_full_correction_constant_rotation.png")
    
    print(simulator.gyro_bias)
    print(ekf.measurements.gyro_bias_accumulated)

    print(simulator.accel_bias)
    print(ekf.measurements.accel_bias_accumulated)

    # 5. Verification
    _, _, _, expected_quat, _ = trajectory.get_state(sim_params.duration)
    assert_quaternion_close(actual=expected_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)
