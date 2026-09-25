import numpy as np
import pytest

from pyekf.ins_esmekf.INS_ESMEKF import INS_ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from pyekf.quaternions import b_to_i_frame_rot_matrix
from tests.utils.utils import (
    assert_quaternion_close,
    attitude_error_vector,
    assert_covariance_valid,
    assert_error_within_sigma,
)
from tests.sim.constant_trajectory import ConstantMotionTrajectory
from tests.sim.sensors import SensorSimulator
from tests.utils.grapher import INSResultsCollector
from tests.sim.params import ConstantSimParams, SensorParams

seed: int = 42

# NOTE: tests (3) and (4) only use magnetometer corrections. Without GPS, displacement, velocity, and accelerometer
# bias are unobservable and drift whenever there is noise, so those tests only assert on attitude accuracy and use the
# covariance consistency check for velocity and displacement (the filter must at least know how uncertain it is).
# Tests (5) to (7) add GPS position and velocity corrections, and assert on attitude, velocity, and displacement.

# Bias random walk density given to the filter. The simulated biases are constant, so this is close to zero.
BIAS_RANDOM_WALK_COV: float = 1e-10

def _build_sim_and_ekf(sensor_params: SensorParams, sim_params: ConstantSimParams):
    """
    Creates the ground truth trajectory, sensor simulator, and INS filter initialized from the readings at t=0.

    The simulator and filter use the noise parameters differently, so they are converted before tuning the filter:
    - Gyro and accel noise are a per-sample variance in the simulator, but a continuous noise density in the filter's Q,
      so the filter gets variance * dt.
    - Biases are constant offsets drawn from N(0, bias_cov) in the simulator, so bias_cov is the filter's initial bias
      uncertainty, and the bias random walk density is set close to zero. With bias_cov = 0 the bias state is frozen at 0.
    - Magnetometer and GPS noise are per-sample in both, so they are passed through unchanged. The INS uses the raw
      (not normalized) magnetometer reading, so the simulator's normalization is turned off.
    """
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
        gps_position_cov=sensor_params.gps_position_cov,
        gps_velocity_cov=sensor_params.gps_velocity_cov,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        normalize_magnetometer=False,
        seed=seed
    )

    gyro_initial, accel_initial, mag_initial = simulator.get_readings(0.0)
    ekf = INS_ESMEKF(
        gyro_initial=gyro_initial,
        accel_initial=accel_initial,
        mag_initial=mag_initial,
        displacement_initial=sim_params.displacement_initial_iframe,
        velocity_initial=sim_params.velocity_initial_iframe,
        quaternion_initial=sim_params.quaternion_initial_iframe,
        gravity_inertial=sim_params.gravity_iframe,
        magnetometer_inertial=sim_params.mag_field_iframe,
        gyro_cov=sensor_params.gyro_cov * sim_params.delta_t,
        accel_cov=sensor_params.accel_cov * sim_params.delta_t,
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=BIAS_RANDOM_WALK_COV if sensor_params.gyro_bias_cov > 0 else 0.0,
        accel_bias_cov=BIAS_RANDOM_WALK_COV if sensor_params.accel_bias_cov > 0 else 0.0,
        magnetometer_bias_cov=BIAS_RANDOM_WALK_COV if sensor_params.magnetometer_bias_cov > 0 else 0.0,
        gps_position_cov=sensor_params.gps_position_cov,
        gps_velocity_cov=sensor_params.gps_velocity_cov,
        magnetometer_gate_threshold=1000.0,
        gps_position_gate_threshold=1000.0,
        gps_velocity_gate_threshold=1000.0,
        p_init_att=0.1,
        p_init_vel=0.1,
        p_init_pos=0.1,
        p_init_gyro_bias=sensor_params.gyro_bias_cov,
        p_init_accel_bias=sensor_params.accel_bias_cov,
        p_init_mag_bias=sensor_params.magnetometer_bias_cov,
    )

    return trajectory, simulator, ekf

def _build_grapher(title: str, sensor_params: SensorParams, sim_params: ConstantSimParams):
    return INSResultsCollector(
        title=title,
        metadata={
            "dt": sim_params.delta_t,
            "T_total": sim_params.duration,
            "gyro_var": sensor_params.gyro_cov,
            "accel_var": sensor_params.accel_cov,
            "mag_var": sensor_params.magnetometer_cov,
            "gyro_bias_cov": sensor_params.gyro_bias_cov,
            "accel_bias_cov": sensor_params.accel_bias_cov,
            "mag_bias_cov": sensor_params.magnetometer_bias_cov,
            "gps_pos_var": sensor_params.gps_position_cov,
            "gps_vel_var": sensor_params.gps_velocity_cov,
        },
    )

def _collect(grapher: INSResultsCollector, trajectory: ConstantMotionTrajectory, ekf: INS_ESMEKF, curr_time: float):
    gt_displacement, gt_velocity, _, gt_quaternion, _ = trajectory.get_state(curr_time)
    grapher.collect_ins(
        time=curr_time,
        gt_quaternion=gt_quaternion,
        est_quaternion=ekf.nominal_state.quaternion_new,
        gt_velocity=gt_velocity,
        est_velocity=ekf.nominal_state.velocity_new,
        gt_displacement=gt_displacement,
        est_displacement=ekf.nominal_state.displacement_new,
    )

def _rms_errors(grapher: INSResultsCollector, t_start: float):
    """
    Returns the RMS attitude error (deg), velocity error norm (m/s), and displacement error norm (m) over all samples
    collected after t_start. Using the RMS over a window instead of only the final sample makes the asserts less
    sensitive to a single noisy step.
    """
    times = np.array(grapher.times)
    window = times >= t_start

    attitude_errors_deg = np.array([
        np.degrees(np.linalg.norm(attitude_error_vector(q_true, q_est)))
        for q_true, q_est in zip(grapher.gt_quaternions, grapher.est_quaternions)
    ])
    velocity_errors = np.linalg.norm(np.array(grapher.gt_velocities) - np.array(grapher.est_velocities), axis=1)
    displacement_errors = np.linalg.norm(np.array(grapher.gt_displacements) - np.array(grapher.est_displacements), axis=1)

    rms = lambda e: float(np.sqrt(np.mean(e[window]**2)))
    return rms(attitude_errors_deg), rms(velocity_errors), rms(displacement_errors)

def _assert_filter_consistent(trajectory: ConstantMotionTrajectory, ekf: INS_ESMEKF, t: float):
    """
    Checks that the error state covariance is a valid covariance, and that the true attitude, velocity, and
    displacement errors lie within 3-sigma of what the filter's covariance predicts.
    Error state layout: [small angle (0:3), velocity (3:6), displacement (6:9), gyro bias (9:12), accel bias (12:15), mag bias (15:18)]
    """
    gt_displacement, gt_velocity, _, gt_quaternion, _ = trajectory.get_state(t)
    P = ekf.P

    assert_covariance_valid(P)
    assert_error_within_sigma(attitude_error_vector(gt_quaternion, ekf.nominal_state.quaternion_new), P[0:3, 0:3], name="attitude")
    assert_error_within_sigma(gt_velocity - ekf.nominal_state.velocity_new, P[3:6, 3:6], name="velocity")
    assert_error_within_sigma(gt_displacement - ekf.nominal_state.displacement_new, P[6:9, 6:9], name="displacement")


def test_constant_rotation_and_accel_no_noise_no_correction(plots_dir):
    """
    BASELINE: Zero noise, zero bias, and NO correction updates.
    Tests if the kinematic integration (state extrapolation) alone can perfectly track a constant rotation
    AND constant linear acceleration. Unlike the AHRS, the INS integrates the accelerometer, so acceleration can be non-zero.
    """
    sensor_params = SensorParams(
        gyro_cov=0.0, accel_cov=0.0, magnetometer_cov=0.0,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[5.0], [3.2], [1.0]]),
        angular_vel_constant_iframe=np.array([[10.0], [np.pi / 4], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (1)test_constant_rotation_and_accel_no_noise_no_correction", sensor_params, sim_params)

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, _ = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        _collect(grapher, trajectory, ekf, curr_time)

    grapher.save_and_show(output_dir=plots_dir, filename="1_ins_no_noise_no_correction.png")

    # With no noise, trapezoidal integration is exact for constant acceleration and constant angular velocity
    expected_displacement, expected_velocity, _, expected_quaternion, _ = trajectory.get_state(sim_params.duration)
    assert_quaternion_close(actual=expected_quaternion, estimate=ekf.nominal_state.quaternion_new, atol=1e-4)
    np.testing.assert_allclose(ekf.nominal_state.velocity_new, expected_velocity, atol=1e-4)
    np.testing.assert_allclose(ekf.nominal_state.displacement_new, expected_displacement, atol=1e-4, rtol=1e-7)


def test_constant_rotation_and_accel_no_bias_no_correction(plots_dir):
    """
    STOCHASTIC BASELINE: Added measurement noise but still NO correction updates.
    Expect the attitude, velocity, and displacement to drift away from Ground Truth over time, so accuracy is not asserted.
    Instead, tests that the covariance propagation (F, Phi, Q) is sane: P must stay a valid covariance, and the drift
    must stay within the 3-sigma bounds the filter predicts.
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

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (2)test_constant_rotation_and_accel_no_bias_no_correction", sensor_params, sim_params)

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, _ = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        _collect(grapher, trajectory, ekf, curr_time)

    grapher.save_and_show(output_dir=plots_dir, filename="2_ins_no_bias_no_correction.png")

    # EXPECT DRIFT: only check that the filter's uncertainty grows consistently with the actual error
    _assert_filter_consistent(trajectory, ekf, sim_params.duration)


def test_constant_rotation_no_bias(plots_dir):
    """
    INS with magnetometer correction (No Bias): Noise is present, and magnetometer updates are used to fix attitude drift.
    Body is stationary (zero acceleration) and rotating about the inertial y-axis, so the body sees the magnetic field sweep
    through the x-z plane. Note the magnetometer can never observe rotation about the magnetic field vector itself,
    so low gyro noise is used to keep that drift small.
    Displacement and velocity are unobservable (no GPS) so only their covariance consistency is checked.
    """
    sensor_params = SensorParams(
        gyro_cov=1e-6, accel_cov=5e-5, magnetometer_cov=1e-4,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=np.array([[0.0], [np.pi], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (3)test_constant_rotation_no_bias", sensor_params, sim_params)

    delta_t = sim_params.delta_t
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        _collect(grapher, trajectory, ekf, curr_time)

    grapher.save_and_show(output_dir=plots_dir, filename="3_ins_no_bias.png")

    # With corrections, we expect attitude to stay close to Ground Truth
    _, _, _, gt_quat, _ = trajectory.get_state(sim_params.duration)
    assert_quaternion_close(actual=gt_quat, estimate=ekf.nominal_state.quaternion_new, atol=2e-1, rtol=1e-1)

    _assert_filter_consistent(trajectory, ekf, sim_params.duration)


def test_ins_full_correction_constant_rotation(plots_dir):
    """
    Tests the 18-state INS_ESMEKF with gyro bias, using magnetometer corrections.
    Same motion as test (3): stationary, rotating about the inertial y-axis.

    Observability notes (why this test only asserts on some things):
    - Rotation about the magnetic field vector is unobservable from the magnetometer, so while the gyro bias is
      still being learned, attitude error about the field builds up and is never corrected (the AHRS shows the same
      behaviour with only magnetometer corrections). Only the attitude error perpendicular to the field is asserted.
    - Magnetometer bias is left at zero: with a single fixed rotation axis, mag bias along that axis is indistinguishable
      from an attitude tilt. Needs a multi-axis rotation trajectory to test.
    - Accelerometer bias is left at zero since it is unobservable without a position/velocity aiding sensor.
    """
    sensor_params = SensorParams(
        gyro_cov=1e-6, accel_cov=5e-5, magnetometer_cov=1e-4,
        gyro_bias_cov=0.01, accel_bias_cov=0, magnetometer_bias_cov=0
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.zeros((3, 1)),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.zeros((3, 1)),
        angular_vel_constant_iframe=np.array([[0.0], [np.pi], [0.0]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (4)test_ins_full_correction_constant_rotation", sensor_params, sim_params)

    delta_t = sim_params.delta_t
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"
    for curr_time in np.arange(delta_t, sim_params.duration + delta_t, delta_t):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)
        _collect(grapher, trajectory, ekf, curr_time)

        if curr_time % 10.0 < delta_t:
            _, _, _, gt_quat, _ = trajectory.get_state(curr_time)
            print(f"Time: {curr_time:6.2f} | K-Gain Norm: {np.linalg.norm(ekf.kalman_gain, ord='fro'):8.4f}")
            print(f"{' ':>6} | EKF | {fmt_vec(ekf.nominal_state.quaternion_new)}")
            print(f"{' ':>6} | GT  | {fmt_vec(gt_quat)}")

    grapher.save_and_show(output_dir=plots_dir, filename="4_ins_full_correction_constant_rotation.png")

    print(f"Gyro bias  true: {simulator.gyro_bias.flatten()} | est: {ekf.measurements.gyro_bias_accumulated.flatten()}")

    # Rotate the (body frame) attitude error into the inertial frame, then remove the unobservable component along the
    # magnetic field. What is left must be small since the magnetometer directly observes it.
    _, _, _, expected_quat, _ = trajectory.get_state(sim_params.duration)
    attitude_error_inertial = b_to_i_frame_rot_matrix(ekf.nominal_state.quaternion_new) @ attitude_error_vector(expected_quat, ekf.nominal_state.quaternion_new)
    mag_direction = sim_params.mag_field_iframe / np.linalg.norm(sim_params.mag_field_iframe)
    attitude_error_observable = attitude_error_inertial - (mag_direction.T @ attitude_error_inertial) * mag_direction
    print(f"Attitude error inertial: {attitude_error_inertial.flatten()} | observable part: {attitude_error_observable.flatten()}")

    assert np.linalg.norm(attitude_error_observable) < np.deg2rad(2.0)
    np.testing.assert_allclose(ekf.measurements.gyro_bias_accumulated, simulator.gyro_bias, atol=1e-2)
    assert_covariance_valid(ekf.P)


def _run_gps_mag_loop(trajectory, simulator, ekf, grapher, sim_params: ConstantSimParams, gps_period: float):
    """
    IMU propagation and magnetometer correction every step, GPS position and velocity correction every gps_period seconds.
    """
    delta_t = sim_params.delta_t
    steps_per_gps = int(round(gps_period / delta_t))
    fmt_vec = lambda v: "[" + " ".join(f"{x:8.4f}" for x in v.flatten()) + "]"

    for step, curr_time in enumerate(np.arange(delta_t, sim_params.duration + delta_t, delta_t), start=1):
        gyro_reading, accel_reading, mag_reading = simulator.get_readings(curr_time)
        ekf.state_extrapolation(gyro_reading, accel_reading, delta_t)
        ekf.correction_magnetometer(magnetometer_new=mag_reading)

        if step % steps_per_gps == 0:
            gps_position, gps_velocity = simulator.get_gps_readings(curr_time)
            ekf.correction_gps_position(gps_position_new=gps_position)
            ekf.correction_gps_velocity(gps_velocity_new=gps_velocity)

        _collect(grapher, trajectory, ekf, curr_time)

        if curr_time % 10.0 < delta_t:
            gt_displacement, gt_velocity, _, gt_quat, _ = trajectory.get_state(curr_time)
            print(f"Time: {curr_time:6.2f} | K-Gain Norm: {np.linalg.norm(ekf.kalman_gain, ord='fro'):8.4f}")
            print(f"{' ':>6} | EKF | {fmt_vec(ekf.nominal_state.quaternion_new)} | {fmt_vec(ekf.nominal_state.displacement_new)} | {fmt_vec(ekf.nominal_state.velocity_new)}")
            print(f"{' ':>6} | GT  | {fmt_vec(gt_quat)} | {fmt_vec(gt_displacement)} | {fmt_vec(gt_velocity)}")


def test_ins_gps_mag_no_bias(plots_dir):
    """
    FULL INS (No Bias): Noise is present, with magnetometer corrections every step and GPS position + velocity at 10 Hz.
    Body is accelerating and rotating mostly about the vertical axis. With GPS, velocity and displacement become
    observable, and so does the attitude about the magnetic field (a tilt error causes a velocity error GPS can see).
    Attitude, velocity, and displacement must all track Ground Truth.
    """
    sensor_params = SensorParams(
        gyro_cov=1e-4, accel_cov=1e-3, magnetometer_cov=1e-4,
        gyro_bias_cov=0, accel_bias_cov=0, magnetometer_bias_cov=0,
        gps_position_cov=0.25, gps_velocity_cov=0.01,
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=100.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.array([[1.0], [0.0], [0.0]]),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[0.2], [0.1], [-0.05]]),
        angular_vel_constant_iframe=np.array([[0.1], [0.1], [np.pi / 4]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (5)test_ins_gps_mag_no_bias", sensor_params, sim_params)

    _run_gps_mag_loop(trajectory, simulator, ekf, grapher, sim_params, gps_period=0.1)
    grapher.save_and_show(output_dir=plots_dir, filename="5_ins_gps_mag_no_bias.png")

    # Errors over the second half of the run, once the filter has converged
    attitude_rms_deg, velocity_rms, displacement_rms = _rms_errors(grapher, t_start=sim_params.duration / 2)
    print(f"RMS errors -> attitude: {attitude_rms_deg:.4f} deg | velocity: {velocity_rms:.4f} m/s | displacement: {displacement_rms:.4f} m")

    # GPS noise is 0.5 m and 0.1 m/s, the filter must do better than GPS alone
    assert attitude_rms_deg < 0.5
    assert velocity_rms < 0.075
    assert displacement_rms < 0.25
    _assert_filter_consistent(trajectory, ekf, sim_params.duration)


def test_ins_gps_mag_with_bias(plots_dir):
    """
    FULL INS with gyro, accelerometer, and magnetometer biases. Same motion and sensors as test (5).
    On top of attitude, velocity, and displacement accuracy, the bias estimates must agree with the true biases
    within the filter's own uncertainty.
    """
    sensor_params = SensorParams(
        gyro_cov=1e-4, accel_cov=1e-3, magnetometer_cov=1e-4,
        gyro_bias_cov=1e-3, accel_bias_cov=1e-2, magnetometer_bias_cov=1e-4,
        gps_position_cov=0.25, gps_velocity_cov=0.01,
    )

    sim_params = ConstantSimParams(
        delta_t=0.01, duration=200.0,
        displacement_initial_iframe=np.zeros((3, 1)),
        velocity_initial_iframe=np.array([[1.0], [0.0], [0.0]]),
        quaternion_initial_iframe=np.array([[1.0], [0.0], [0.0], [0.0]]),
        accel_constant_iframe=np.array([[0.2], [0.1], [-0.05]]),
        angular_vel_constant_iframe=np.array([[0.1], [0.1], [np.pi / 4]]),
        gravity_iframe=GRAVITY_INERTIAL, mag_field_iframe=MAGNETOMETER_INERTIAL
    )

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (6)test_ins_gps_mag_with_bias", sensor_params, sim_params)

    _run_gps_mag_loop(trajectory, simulator, ekf, grapher, sim_params, gps_period=0.1)
    grapher.save_and_show(output_dir=plots_dir, filename="6_ins_gps_mag_with_bias.png")

    print(f"Gyro bias  true: {simulator.gyro_bias.flatten()} | est: {ekf.measurements.gyro_bias_accumulated.flatten()}")
    print(f"Accel bias true: {simulator.accel_bias.flatten()} | est: {ekf.measurements.accel_bias_accumulated.flatten()}")
    print(f"Mag bias   true: {simulator.magnetometer_bias.flatten()} | est: {ekf.measurements.mag_bias_accumulated.flatten()}")

    attitude_rms_deg, velocity_rms, displacement_rms = _rms_errors(grapher, t_start=sim_params.duration / 2)
    print(f"RMS errors -> attitude: {attitude_rms_deg:.4f} deg | velocity: {velocity_rms:.4f} m/s | displacement: {displacement_rms:.4f} m")

    # GPS noise is 0.5 m and 0.1 m/s, the filter must do better than GPS alone
    assert attitude_rms_deg < 0.5
    assert velocity_rms < 0.075
    assert displacement_rms < 0.25
    _assert_filter_consistent(trajectory, ekf, sim_params.duration)

    # Bias estimates must be within 3-sigma of the filter's own bias uncertainty
    P = ekf.P
    assert_error_within_sigma(simulator.gyro_bias - ekf.measurements.gyro_bias_accumulated, P[9:12, 9:12], name="gyro bias")
    assert_error_within_sigma(simulator.accel_bias - ekf.measurements.accel_bias_accumulated, P[12:15, 12:15], name="accel bias")
    assert_error_within_sigma(simulator.magnetometer_bias - ekf.measurements.mag_bias_accumulated, P[15:18, 15:18], name="mag bias")


def test_ins_gps_mag_high_rate_rotation(plots_dir):
    """
    STRESS TEST: same fast rotation and strong acceleration as tests (1) and (2) (~10 rad/s), with noise, gyro and
    accelerometer biases, magnetometer corrections every step and GPS at 10 Hz. Checks the filter stays stable and
    accurate when the body spins fast, where discretization errors in Phi and Q are largest.
    """
    sensor_params = SensorParams(
        gyro_cov=1e-4, accel_cov=1e-3, magnetometer_cov=1e-4,
        gyro_bias_cov=1e-3, accel_bias_cov=1e-2, magnetometer_bias_cov=0,
        gps_position_cov=0.25, gps_velocity_cov=0.01,
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

    trajectory, simulator, ekf = _build_sim_and_ekf(sensor_params, sim_params)
    grapher = _build_grapher("INS_ESMEKF: (7)test_ins_gps_mag_high_rate_rotation", sensor_params, sim_params)

    _run_gps_mag_loop(trajectory, simulator, ekf, grapher, sim_params, gps_period=0.1)
    grapher.save_and_show(output_dir=plots_dir, filename="7_ins_gps_mag_high_rate_rotation.png")

    attitude_rms_deg, velocity_rms, displacement_rms = _rms_errors(grapher, t_start=sim_params.duration / 2)
    print(f"RMS errors -> attitude: {attitude_rms_deg:.4f} deg | velocity: {velocity_rms:.4f} m/s | displacement: {displacement_rms:.4f} m")

    # GPS noise is 0.5 m and 0.1 m/s, the filter must do better than GPS alone
    assert attitude_rms_deg < 0.5
    assert velocity_rms < 0.075
    assert displacement_rms < 0.25
    _assert_filter_consistent(trajectory, ekf, sim_params.duration)
