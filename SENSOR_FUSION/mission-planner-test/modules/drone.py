import pymavlink
from pymavlink import mavutil   

class Drone:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.mavlink_connection = pymavlink.mavutil.mavlink_connection(connection_string)
        self.mavlink_connection.wait_heartbeat()
        print("Connected to drone and received heartbeat.")
        
    def arm(self):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        
    def takeoff(self, altitude):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, altitude
        )
        
    def land(self):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
    def rtl(self):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    
    def set_speed(self, vx, vy, vz):
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            int(self.mavlink_connection.time_since("SYSTEM_TIME")),
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        
    def set_yaw_rate(self, yaw_rate):
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            int(self.mavlink_connection.time_since("SYSTEM_TIME")),
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b010111000111,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, yaw_rate,
        )
        
    def set_speed_and_yaw_rate(self, vx, vy, vz, yaw_rate):
        self.mavlink_connection.mav.set_position_target_local_ned_send(
            int(self.mavlink_connection.time_since("SYSTEM_TIME")),
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b010111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate
        )

    def request_raw_sensor_stream(self, rate_hz = 10):
        """
        Requests raw sensor data (e.g., IMU, GPS) at the given rate (Hz).
        """
        self.mavlink_connection.mav.request_data_stream_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            rate_hz,
            1
        )

        print(f"Requested HIGH_REZ_RAW_SENSOR stream: {rate_hz} hz.")

    def get_imu_data(self, timeout=5):
        """
        Waits for a RAW_IMU MAVLink message and returns the IMU data.
        :param timeout: Maximum time in seconds to wait for the message.
        :return: Dictionary with IMU values or None if timeout.
        """
        msg = self.mavlink_connection.recv_match(type='RAW_IMU', blocking=True, timeout=timeout)
        if msg is None:
            print("Timeout waiting for RAW_IMU message.")
            return None
        
        imu_data = {
            'time_usec': msg.time_usec,
            'xacc': msg.xacc,
            'yacc': msg.yacc,
            'zacc': msg.zacc,
            'xgyro': msg.xgyro,
            'ygyro': msg.ygyro,
            'zgyro': msg.zgyro,
            'xmag': msg.xmag,
            'ymag': msg.ymag,
            'zmag': msg.zmag,
            # 'abs_pressure': msg.abs_pressure,
            # 'diff_pressure': msg.diff_pressure,
            # 'pressure_alt': msg.pressure_alt,
            # 'temperature': msg.temperature
        }
        return imu_data
    
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
                