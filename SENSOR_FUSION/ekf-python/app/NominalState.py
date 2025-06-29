import numpy as np
from numpy.typing import NDArray
from utils import skew_symmetric, normalize_quaternion, IDENTITY_QUATERNION

class NominalState:
    def __init__(
            self,
            position: NDArray[np.float64] = np.zeros((3, 1)),
            velocity: NDArray[np.float64] = np.zeros((3, 1)),
            quaternion: NDArray[np.float64] = IDENTITY_QUATERNION,
            prev_gyro_measurement: NDArray[np.float64] = np.zeros((3, 1)),
            prev_accel_measurement: NDArray[np.float64] = np.zeros((3, 1))
        ):

        self.displacement = np.asarray(position, dtype=float).reshape(3, 1)
        self.velocity = np.asarray(velocity, dtype=float).reshape(3, 1)
        self.quaternion = normalize_quaternion(np.asarray(quaternion, dtype=float)).reshape(4, 1)
        self.prev_gyro_measurement = np.asarray(prev_gyro_measurement, dtype=float).reshape(3, 1)
        self.prev_accel_measurement = np.asarray(prev_accel_measurement, dtype=float).reshape(3, 1)

    def __str__(self):
        return (f"Nominal State:\n"
                f"  Displacement (p):   {self.displacement.flatten()} m\n"
                f"  Velocity (v):   {self.velocity.flatten()} m/s\n"
                f"  Quaternion (q): {self.quaternion.flatten()} (w,x,y,z)\n"
                f"  Prev Gyro (w_prev): {self.prev_gyro_measurement.flatten()} rad/s\n"
                f"  Prev Accel (a_prev): {self.prev_accel_measurement.flatten()} m/s^2")
