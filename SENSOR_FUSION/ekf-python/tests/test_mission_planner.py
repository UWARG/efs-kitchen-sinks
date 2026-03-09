import pytest
import os
from dotenv import load_dotenv
from pymavlink import mavutil

load_dotenv()

from tests.mission_planner.drone import Drone

def test_mission_planner(plots_dir):
    CONNECTION_STRING = os.getenv("CONNECTION_STRING")
    assert CONNECTION_STRING is not None, "CONNECTION_STRING environment variable must be set for testing."

    print("Starting mission planner test with connection string:", CONNECTION_STRING)

    drone_connection = Drone(connection_string=CONNECTION_STRING)
    drone_connection.request_message_interval(message_id=mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, rate_hz=1)
    drone_connection.request_message_interval(message_id=mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate_hz=1)
    drone_connection.request_message_interval(message_id=mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, rate_hz=1)

    while (True):
        imu = drone_connection.get_scaled_imu_data()
        attitude = drone_connection.get_attitude_data()
        attitude_quaternion = drone_connection.get_attitude_quaternion_data()
        if (imu):
            print("IMU:", imu)
        if (attitude):
            print("Attitude: ", attitude)
        if (attitude_quaternion):
            print("Attitude Quaternion: ", attitude_quaternion)
