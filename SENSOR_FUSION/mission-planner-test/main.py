from modules import (
    drone,
)
from dotenv import load_dotenv
import os

load_dotenv()

CONNECTION_STRING = os.getenv("CONNECTION_STRING")

def main():
    drone_connection = drone.Drone(connection_string=CONNECTION_STRING)
    drone_connection.request_raw_sensor_stream(rate_hz=10)

    while (True):
        imu = drone_connection.get_imu_data()
        gps = drone_connection.get_gps_data()
        if (imu):
            print("IMU:", imu)
        if (gps):
            print("GPS: ", gps)


if __name__ == "__main__":
    main()
