import numpy as np
from numpy.typing import NDArray

from pyekf.utils import to_col_vector

class RawMeasurements:
    """
    Class needed to store current and previous timestep measurements.
    """

    def __init__(
        self,
        gyro_initial: NDArray[np.float64] = np.zeros((3, 1)),
        accel_initial: NDArray[np.float64] = np.zeros((3, 1)),
        mag_initial: NDArray[np.float64] = np.zeros((3, 1)),
    ):
        self.gyro_prev: NDArray[np.float64] = to_col_vector(gyro_initial, 3)
        self.gyro_new: NDArray[np.float64] = to_col_vector(gyro_initial, 3)
        self.accel_prev: NDArray[np.float64] = to_col_vector(accel_initial, 3)
        self.accel_new: NDArray[np.float64] = to_col_vector(accel_initial, 3)
        self.mag_prev: NDArray[np.float64] = to_col_vector(mag_initial, 3)
        self.mag_new: NDArray[np.float64] = to_col_vector(mag_initial, 3)

    def update_gyro(self, gyro_new: NDArray[np.float64]):
        self.gyro_prev = self.gyro_new
        self.gyro_new = to_col_vector(gyro_new, 3)
    
    def update_accel(self, accel_new: NDArray[np.float64]):
        self.accel_prev = self.accel_new
        self.accel_new = to_col_vector(accel_new, 3)
    
    def update_mag(self, mag_new: NDArray[np.float64]):
        self.mag_prev = self.mag_new
        self.mag_new = to_col_vector(mag_new, 3)

    @property
    def gyro_bar(self) -> NDArray[np.float64]:
        return np.average([self.gyro_prev, self.gyro_new], axis=0)

    @property
    def accel_bar(self) -> NDArray[np.float64]:
        return np.average([self.accel_prev, self.accel_new], axis=0)

    @property
    def mag_bar(self) -> NDArray[np.float64]:
        return np.average([self.mag_prev, self.mag_new], axis=0)

    def __str__(self):
        return (
            f"RawMeasurements()\n"
            f"  Gyro prev: {self.gyro_prev.flatten()}\n"
            f"  Gyro new: {self.gyro_new.flatten()}\n"
            f"  Gyro bar:  {self.gyro_bar.flatten()}\n"
            f"  Accel prev: {self.accel_prev.flatten()}\n"
            f"  Accel new: {self.accel_new.flatten()}\n"
            f"  Accel bar:  {self.accel_bar.flatten()}\n"
            f"  Mag prev: {self.mag_prev.flatten()}\n"
            f"  Mag new: {self.mag_new.flatten()}\n"
            f"  Mag bar:  {self.mag_bar.flatten()}\n"
        )
