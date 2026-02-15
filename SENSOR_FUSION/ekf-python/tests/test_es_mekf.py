import numpy as np
import pytest

from pyekf.ESMEKF import ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from tests.utils.utils import assert_quaternion_close
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator
from tests.sim.params import ConstantSimParams, SensorParams

seed: int = 42

def test_constant_rotation_and_accel_no_noise_no_correction():
    sensor_params = SensorParams(
        gyro_cov=0.0,
        accel_cov=0.0,
        magnetometer_cov=0.0,
        gyro_bias_cov=0,
        accel_bias_cov=0,
        magnetometer_bias_cov=0
    )

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
    ekf = ESMEKF(
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
    )

    # 4. Run Simulation Loop
    delta_t = sim_params.delta_t
    duration = sim_params.duration
    
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for curr_time in np.arange(delta_t, duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        
        gt_displacement, gt_velocity, gt_accel, gt_quaternion, gt_angular_vel = trajectory.get_state(curr_time)
        
        est_displacement = ekf.nominal_state.displacement_new
        est_velocity = ekf.nominal_state.velocity_new
        est_quaternion = ekf.nominal_state.quaternion_new

        print(f"{curr_time:5.2f} | EKF   | {fmt_vec(est_quaternion)} | {fmt_vec(est_displacement)} | {fmt_vec(est_velocity)}")
        print(f"{' ':>5} | GT    | {fmt_vec(gt_quaternion)} | {fmt_vec(gt_displacement)} | {fmt_vec(gt_velocity)}")
        print("-" * 130)

    # 5. Verification
    expected_displacement, expected_velocity, expected_accel, expected_quaternion, expected_angular_vel = trajectory.get_state(duration)
    
    estimated_displacement = ekf.nominal_state.displacement_new
    estimated_velocity = ekf.nominal_state.velocity_new
    estimated_quaternion = ekf.nominal_state.quaternion_new

    # no noise test should be precise
    assert_quaternion_close(actual=expected_quaternion, estimate=estimated_quaternion, atol=1e-4)
    np.testing.assert_allclose(estimated_displacement, expected_displacement, atol=1e-4)
    np.testing.assert_allclose(estimated_velocity, expected_velocity, atol=1e-4)


def test_constant_rotation_and_accel_no_bias_no_correction():
    sensor_params = SensorParams(
        gyro_cov=0.01,
        accel_cov=0.01,
        magnetometer_cov=0,
        gyro_bias_cov=0,
        accel_bias_cov=0,
        magnetometer_bias_cov=0
    )

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
    ekf = ESMEKF(
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
    )

    # 4. Run Simulation Loop
    delta_t = sim_params.delta_t
    duration = sim_params.duration
    
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for curr_time in np.arange(delta_t, duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        
        gt_displacement, gt_velocity, gt_accel, gt_quaternion, gt_angular_vel = trajectory.get_state(curr_time)
        
        est_displacement = ekf.nominal_state.displacement_new
        est_velocity = ekf.nominal_state.velocity_new
        est_quaternion = ekf.nominal_state.quaternion_new

        print(f"{curr_time:5.2f} | EKF   | {fmt_vec(est_quaternion)} | {fmt_vec(est_displacement)} | {fmt_vec(est_velocity)}")
        print(f"{' ':>5} | GT    | {fmt_vec(gt_quaternion)} | {fmt_vec(gt_displacement)} | {fmt_vec(gt_velocity)}")
        print("-" * 130)

    # 5. Verification
    expected_displacement, expected_velocity, expected_accel, expected_quaternion, expected_angular_vel = trajectory.get_state(duration)
    
    estimated_displacement = ekf.nominal_state.displacement_new
    estimated_velocity = ekf.nominal_state.velocity_new
    estimated_quaternion = ekf.nominal_state.quaternion_new

    # no noise test should be precise
    assert_quaternion_close(actual=expected_quaternion, estimate=estimated_quaternion, atol=2e-1, rtol=1e-1)
    np.testing.assert_allclose(estimated_displacement, expected_displacement, atol=2e-1, rtol=1e-1)
    np.testing.assert_allclose(estimated_velocity, expected_velocity, atol=2e-1, rtol=1e-1)


def test_constant_rotation_and_accel_no_bias():
    sensor_params = SensorParams(
        gyro_cov=0.000001,
        accel_cov=0.00005,
        magnetometer_cov=0.0001,
        gyro_bias_cov=0,
        accel_bias_cov=0,
        magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01,
        duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=np.array([[0.0], [np.pi], [0.0]]),
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
    ekf = ESMEKF(
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
    )

    # 4. Run Simulation Loop
    delta_t = sim_params.delta_t
    duration = sim_params.duration
    
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    print(f"\n{'Time':>5} | {'Type':>5} | {'Quaternion [w x y z]':^30} | {'Displacement':^18} | {'Velocity':^18}")
    print("-" * 110)

    for curr_time in np.arange(delta_t, duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)

        # print("gyro bias acc", ekf.measurements.gyro_bias_accumulated.flatten())
        # print("accel bias acc", ekf.measurements.accel_bias_accumulated.flatten())
        # print("mag bias acc", ekf.measurements.mag_bias_accumulated.flatten())
        # print("mag bar measured", self.measurements.mag_bar.flatten())
        # print("mag predicted", mag_predicted.flatten())
        # print("mag new measured", self.measurements.mag_new.flatten())
        # print("predicted average quaternion", average_quaternions(self.nominal_state.quaternion_new, self.nominal_state.quaternion_prev).flatten())
        # print("predicted new quaternion", self.nominal_state.quaternion_new.flatten())
        # The gain matrix is 18x3. The norm tells us the total 'strength' of the update.
        kalman_gain_norm = np.linalg.norm(ekf.kalman_gain, ord='fro')
        
        # Optional: Print specifically the orientation part of the gain (first 3 rows)
        orientation_gain_norm = np.linalg.norm(ekf.kalman_gain[0:3, :], ord='fro')

        print(f"Time: {curr_time:5.2f} | K-Gain Norm: {kalman_gain_norm:10.6f} | Ori-Gain Norm: {orientation_gain_norm:10.6f}")
        # print(ekf.kalman_gain)
        # print(ekf.measurements.gyro_bias_accumulated.flatten(), simulator.gyro_bias.flatten())
        
        gt_displacement, gt_velocity, gt_accel, gt_quaternion, gt_angular_vel = trajectory.get_state(curr_time)
        
        est_displacement = ekf.nominal_state.displacement_new
        est_velocity = ekf.nominal_state.velocity_new
        est_quaternion = ekf.nominal_state.quaternion_new

        print(f"{curr_time:5.2f} | EKF   | {fmt_vec(est_quaternion)} | {fmt_vec(est_displacement)} | {fmt_vec(est_velocity)}")
        print(f"{' ':>5} | GT    | {fmt_vec(gt_quaternion)} | {fmt_vec(gt_displacement)} | {fmt_vec(gt_velocity)}")
        print("-" * 130)

    # 5. Verification
    expected_displacement, expected_velocity, expected_accel, expected_quaternion, expected_angular_vel = trajectory.get_state(duration)
    
    estimated_displacement = ekf.nominal_state.displacement_new
    estimated_velocity = ekf.nominal_state.velocity_new
    estimated_quaternion = ekf.nominal_state.quaternion_new

    # no noise test should be precise
    assert_quaternion_close(estimated_quaternion, expected_quaternion, atol=2e-1, rtol=1e-1)
    # np.testing.assert_allclose(estimated_displacement, expected_displacement, atol=2e-1, rtol=1e-1)
    # np.testing.assert_allclose(estimated_velocity, expected_velocity, atol=2e-1, rtol=1e-1)
