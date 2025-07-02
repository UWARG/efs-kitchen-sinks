import numpy as np
from numpy.typing import NDArray
from app.utils import skew_symmetric, normalize_quaternion, IDENTITY_QUATERNION, GRAVITY_INERTIAL, b_to_i_frame_rot_matrix
from app.NominalState import NominalState

class ESMEKF:
    def __init__(
            self
        ):

        pass

    def __str__(self):
        return (f"ESMEKF Internal State:\n")
