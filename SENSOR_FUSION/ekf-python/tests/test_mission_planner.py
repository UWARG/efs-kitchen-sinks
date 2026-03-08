import pytest
import os
from dotenv import load_dotenv

load_dotenv()

from tests.mission_planner.drone import Drone

def test_mission_planner(plots_dir):
    CONNECTION_STRING = os.getenv("CONNECTION_STRING")
    assert CONNECTION_STRING is not None, "CONNECTION_STRING environment variable must be set for testing."

    print("Starting mission planner test with connection string:", CONNECTION_STRING)

    drone_connection = Drone(connection_string=CONNECTION_STRING)
    drone_connection.request_raw_sensor_stream(rate_hz=10)

    while (True):
        print("Waiting for sensor data...")
        imu = drone_connection.get_scaled_imu_data()
        gps = drone_connection.get_gps_data()
        if (imu):
            print("IMU:", imu)
        if (gps):
            print("GPS: ", gps)

