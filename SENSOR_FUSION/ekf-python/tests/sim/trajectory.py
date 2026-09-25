from abc import ABC, abstractmethod
import numpy as np
from numpy.typing import NDArray

class Trajectory(ABC):
    """
    Abstract Base Class for all trajectory models.
    Ensures any simulator can interact with any trajectory via a common API.
    """

    @abstractmethod
    def get_state(self, t: float) -> tuple[
        NDArray[np.float64], # Displacement (3x1)
        NDArray[np.float64], # Velocity (3x1)
        NDArray[np.float64], # Acceleration (3x1)
        NDArray[np.float64], # Quaternion (4x1)
        NDArray[np.float64]  # Angular Velocity (3x1)
    ]:
        """Returns the inertial frame state at time t."""
        pass
