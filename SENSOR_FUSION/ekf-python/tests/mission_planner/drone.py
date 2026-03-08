from pymavlink import mavutil

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.mavlink_connection = mavutil.mavlink_connection(connection_string)
        self.mavlink_connection.wait_heartbeat()
        print("Connected to drone and received heartbeat.")

    def request_message_interval(self, message_id, rate_hz):
        """
        Request a specific MAVLink message at a given rate.
        """
        interval_us = int(1e6 / rate_hz)

        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    def request_sensor_streams(self, imu_rate=50, gps_rate=10):
        """
        Request all telemetry streams needed.
        """

        # RAW_IMU
        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            imu_rate
        )

        # SCALED_IMU
        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
            imu_rate
        )

        # GPS
        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            gps_rate
        )

        print(f"Requested RAW_IMU & SCALED_IMU @ {imu_rate} Hz")
        print(f"Requested GPS_RAW_INT @ {gps_rate} Hz")


    def get_raw_imu_data(self, timeout=5):
        """
        Waits for a RAW_IMU MAVLink message and returns the IMU data.
        :param timeout: Maximum time in seconds to wait for the message.
        :return: Dictionary with IMU values or None if timeout.
        """
        msg = self.mavlink_connection.recv_match(type='RAW_IMU', blocking=True, timeout=timeout)
        if msg is None:
            print("Timeout waiting for RAW_IMU message.")
            return None
        return msg.to_dict()
        
    def get_scaled_imu_data(self, timeout=5):
        """
        Waits for a SCALED_IMU MAVLink message and returns the IMU data in standard units.
        :param timeout: Maximum time in seconds to wait for the message.
        :return: Dictionary with IMU values or None if timeout.
        """
        msg = self.mavlink_connection.recv_match(type='SCALED_IMU', blocking=True, timeout=timeout)
        if msg is None:
            print("Timeout waiting for SCALED_IMU message.")
            return None

        # Convert accelerometer from mG to m/s^2 (1 g ≈ 9.80665 m/s^2)
        xacc = msg.xacc * 9.80665 / 1000.0
        yacc = msg.yacc * 9.80665 / 1000.0
        zacc = msg.zacc * 9.80665 / 1000.0

        # Convert gyroscope from mrad/s to rad/s
        xgyro = msg.xgyro / 1000.0
        ygyro = msg.ygyro / 1000.0
        zgyro = msg.zgyro / 1000.0

        # Normalize magnetometer vector (unitless)
        import math
        mag_norm = math.sqrt(msg.xmag**2 + msg.ymag**2 + msg.zmag**2)
        if mag_norm > 0:
            xmag = msg.xmag / mag_norm
            ymag = msg.ymag / mag_norm
            zmag = msg.zmag / mag_norm
        else:
            xmag = ymag = zmag = 0.0

        scaled_imu_data = {
            'time_boot_ms': msg.time_boot_ms,
            'xacc': xacc,
            'yacc': yacc,
            'zacc': zacc,
            'xgyro': xgyro,
            'ygyro': ygyro,
            'zgyro': zgyro,
            'xmag': xmag,
            'ymag': ymag,
            'zmag': zmag,
            'temperature': msg.temperature / 100.0  # cdegC to °C
        }
        return scaled_imu_data
    
    def get_gps_data(self, timeout=5):
        """
        Waits for a GPS_RAW_INT MAVLink message and returns GPS data.
        :param timeout: Max time to wait in seconds.
        :return: Dictionary with GPS values or None.
        """
        msg = self.mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout)
        if msg is None:
            print("Timeout waiting for GPS_RAW_INT message.")
            return None

        gps_data = {
            'time_usec': msg.time_usec,        # [us]
            'fix_type': msg.fix_type,          # 0-1: No fix, 2: 2D, 3: 3D fix
            'lat': msg.lat / 1e7,              # [deg]
            'lon': msg.lon / 1e7,              # [deg]
            'alt': msg.alt / 1e3,              # [m]
            'eph': msg.eph / 100.0,            # [m] GPS HDOP
            'epv': msg.epv / 100.0,            # [m] GPS VDOP
            'vel': msg.vel / 100.0,            # [m/s] ground speed
            'cog': msg.cog / 100.0,            # [deg] course over ground
            'satellites_visible': msg.satellites_visible
        }
        return gps_data

    # def request_raw_sensor_stream(self, rate_hz = 10):
    #     """
    #     Requests raw sensor data (e.g., IMU, GPS) at the given rate (Hz).
    #     """
    #     self.mavlink_connection.mav.request_data_stream_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    #         rate_hz,
    #         1
    #     )
    #     print(f"Requested HIGH_REZ_RAW_SENSOR stream: {rate_hz} hz.")

    # def arm(self):
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #         0,  # confirmation
    #         1,  # arm
    #         0, 0, 0, 0, 0, 0
    #     )
        
    # def takeoff(self, altitude):
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    #         0,  # confirmation
    #         0, 0, 0, 0, 0, 0, altitude
    #     )
        
    # def land(self):
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_CMD_NAV_LAND,
    #         0,
    #         0, 0, 0, 0, 0, 0, 0
    #     )
        
    # def rtl(self):
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    #         0,
    #         0, 0, 0, 0, 0, 0, 0
    #     )
    
    # def set_speed(self, vx, vy, vz):
    #     self.mavlink_connection.mav.set_position_target_local_ned_send(
    #         int(self.mavlink_connection.time_since("SYSTEM_TIME")),
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    #         0b110111000111,
    #         0, 0, 0,
    #         vx, vy, vz,
    #         0, 0, 0,
    #         0, 0)
        
    # def set_yaw_rate(self, yaw_rate):
    #     self.mavlink_connection.mav.set_position_target_local_ned_send(
    #         int(self.mavlink_connection.time_since("SYSTEM_TIME")),
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    #         0b010111000111,
    #         0, 0, 0,
    #         0, 0, 0,
    #         0, 0, 0,
    #         0, yaw_rate,
    #     )
        
    # def set_speed_and_yaw_rate(self, vx, vy, vz, yaw_rate):
    #     self.mavlink_connection.mav.set_position_target_local_ned_send(
    #         int(self.mavlink_connection.time_since("SYSTEM_TIME")),
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    #         0b010111000111,
    #         0, 0, 0,
    #         vx, vy, vz,
    #         0, 0, 0,
    #         0, yaw_rate
    #     )