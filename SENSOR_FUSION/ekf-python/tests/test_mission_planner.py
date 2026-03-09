import pytest
import os
import time
from dotenv import load_dotenv
import numpy as np
from pymavlink import mavutil

from pyekf.ahrs_esmekf.AHRS_ESMEKF import AHRS_ESMEKF
from pyekf.utils import GRAVITY_INERTIAL, MAGNETOMETER_INERTIAL
from tests.utils.utils import assert_quaternion_close
from tests.mission_planner.drone import Drone
from tests.utils.grapher import ResultsCollector
from tests.sim.params import SensorParams

load_dotenv()
seed: int = 42

@pytest.mark.mission_planner
def test_ahrs_with_mission_planner(plots_dir):
    """
    Test AHRS_ESMEKF using live/simulated data from Mission Planner.
    Pulls SCALED_IMU and ATTITUDE_QUATERNION messages.
    """
    CONNECTION_STRING = os.getenv("CONNECTION_STRING")
    assert CONNECTION_STRING is not None, "CONNECTION_STRING env var must be set."

    # 1. Connect to Drone
    drone = Drone(connection_string=CONNECTION_STRING)
    rate_hz = 50
    drone.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, rate_hz=rate_hz)
    drone.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate_hz=rate_hz)
    drone.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, rate_hz=rate_hz)

    # 2. Get initial readings to initialize filter
    imu = None
    while imu is None:
        imu = drone.get_scaled_imu_data(timeout=1.0)
    initial_gyro = np.array([[imu['xgyro']], [imu['ygyro']], [imu['zgyro']]])
    initial_accel = np.array([[imu['xacc']], [imu['yacc']], [imu['zacc']]])
    initial_mag = np.array([[imu['xmag']], [imu['ymag']], [imu['zmag']]])

    # 3. Initialize AHRS_ESMEKF filter
    sensor_params = SensorParams(
        gyro_cov=1e-5, accel_cov=0.001, magnetometer_cov=0.001,
        gyro_bias_cov=0.01, accel_bias_cov=0.01, magnetometer_bias_cov=0
    )

    ekf = AHRS_ESMEKF(
        gyro_initial=initial_gyro,
        accel_initial=initial_accel,
        mag_initial=initial_mag,
        quaternion_initial=np.array([[1.0], [0.0], [0.0], [0.0]]),
        gravity_inertial=GRAVITY_INERTIAL,
        magnetometer_inertial=initial_mag,
        gyro_cov=sensor_params.gyro_cov,
        accel_cov=sensor_params.accel_cov,
        magnetometer_cov=sensor_params.magnetometer_cov,
        gyro_bias_cov=sensor_params.gyro_bias_cov,
        accel_bias_cov=sensor_params.accel_bias_cov,
        accel_gate_threshold=1000.0,
        magnetometer_gate_threshold=1000.0,
        p_init_att=0.1,
        p_init_bias=0.01,
    )

    # 4. Initialize grapher
    total_time_secs = 120.0
    grapher = ResultsCollector(
        title="AHRS_ESMEKF: (5)test_ahrs_with_mission_planner",
        metadata={
            "dt": "N/A (real-time data)",
            "T_total": total_time_secs,
            "gyro_var": sensor_params.gyro_cov,
            "accel_var": sensor_params.accel_cov,
            "mag_var": sensor_params.magnetometer_cov,
            "gyro_bias_cov": sensor_params.gyro_bias_cov,
            "accel_bias_cov": sensor_params.accel_bias_cov,
            "mag_bias_cov": sensor_params.magnetometer_bias_cov,
        },
    )

    # 5. Run real-time loop
    start_time = time.time()
    last_time = start_time
    while (time.time() - start_time) < total_time_secs:
        current_time = time.time()
        delta_t = current_time - last_time
        last_time = current_time
        print(f"Time: {current_time - start_time:.2f}s | Δt: {delta_t:.3f}s")

        imu = drone.get_scaled_imu_data(timeout=1.0)
        attitude_quaternion = drone.get_attitude_quaternion_data(timeout=1.0)
        if imu is None or attitude_quaternion is None:
            continue

        gyro_reading = np.array([[imu['xgyro']], [imu['ygyro']], [imu['zgyro']]])
        accel_reading = np.array([[imu['xacc']], [imu['yacc']], [imu['zacc']]])
        mag_reading = np.array([[imu['xmag']], [imu['ymag']], [imu['zmag']]])

        print(f"Gyro: {gyro_reading.flatten()} | Accel: {accel_reading.flatten()} | Mag: {mag_reading.flatten()}")

        ekf.state_extrapolation(gyro_reading, delta_t)
        ekf.correction_accelerometer(accel_reading)
        ekf.correction_magnetometer(mag_reading)

        gt_quat = np.array([
            attitude_quaternion['q1'],
            attitude_quaternion['q2'],
            attitude_quaternion['q3'],
            attitude_quaternion['q4']
        ])

        grapher.collect(current_time - start_time, gt_quat, ekf.nominal_state.quaternion_new)

        print(f"Estimated Quaternion: {ekf.nominal_state.quaternion_new.flatten()} | Ground Truth Quaternion: {gt_quat.flatten()}")
        
        time.sleep(1/rate_hz)

    # 6. Simple verification: quaternion norm should be ~1
    grapher.save_and_show(output_dir=plots_dir, filename="5_ahrs_with_mission_planner.png")
    assert_quaternion_close(actual=gt_quat.flatten(), estimate=ekf.nominal_state.quaternion_new.flatten(), atol=2e-1, rtol=1e-1)
