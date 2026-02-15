import numpy as np
from numpy.typing import NDArray

# Convention: every vector is a column vector
IDENTITY_QUATERNION = np.array([[1.0], [0.0], [0.0], [0.0]], dtype=np.float64)

def multiply_quaternions(q1: NDArray[np.float64], q2: NDArray[np.float64]) -> NDArray[np.float64]:
    q1 = q1.reshape(4, 1)
    q2 = q2.reshape(4, 1)
    w1, x1, y1, z1 = q1[:, 0]
    w2, x2, y2, z2 = q2[:, 0]
    return np.array([
        [w1*w2 - x1*x2 - y1*y2 - z1*z2],
        [w1*x2 + x1*w2 + y1*z2 - z1*y2],
        [w1*y2 - x1*z2 + y1*w2 + z1*x2],
        [w1*z2 + x1*y2 - y1*x2 + z1*w2],
    ], dtype=np.float64)

def inverse_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    q = q.reshape(4, 1)
    w, x, y, z = q[:, 0]
    q_norm_squared: float = (q.T @ q).item()
    return np.array([[w], [-x], [-y], [-z]], dtype=np.float64) / q_norm_squared

def normalize_quaternion(q: NDArray[np.float64]) -> NDArray[np.float64]:
    q = q.reshape(4, 1)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        return IDENTITY_QUATERNION.copy()
    return q / norm

def average_quaternions(q1: NDArray[np.float64], q2: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Computes the mid-point (geodesic average) between two unit quaternions.
    
    The algorithm maps the relative rotation between q1 and q2 into the Lie Algebra 
    (tangent space) using the logarithmic map, halves the resulting rotation vector, 
    and maps it back to the SO(3) manifold via the exponential map. This is 
    mathematically equivalent to Slerp with t=0.5.
    """
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)

    # Relative rotation: r = q1^{-1} x q2
    r = multiply_quaternions(inverse_quaternion(q1), q2)

    # Ensure shortest path (important for averaging)
    if r[0, 0] < 0.0:
        r = -r

    r0 = np.clip(r[0, 0], -1.0, 1.0)
    rv = r[1:4, 0:1]

    # ||mu|| = 2 * acos(rtheta)
    mu_norm = 2.0 * np.arccos(r0)

    # Small-angle handling
    if mu_norm < 1e-12:
        return q1.copy()

    # mu = r_vec * (||mu|| / sin(||mu||/2))
    sin_half = np.sin(mu_norm / 2.0)
    mu = rv * (mu_norm / sin_half)

    # Half rotation: ||mu|| / 2
    half_norm = mu_norm / 2.0

    # r_n = [cos(||mu||/4), (mu/||mu||) sin(||mu||/4)]
    axis = mu / mu_norm
    r_n = np.zeros((4, 1), dtype=np.float64)
    r_n[0, 0] = np.cos(half_norm / 2.0)
    r_n[1:4, 0:1] = axis * np.sin(half_norm / 2.0)

    # Average quaternion = q1 x r_n
    q_avg = multiply_quaternions(q1, r_n)
    return q_avg / np.linalg.norm(q_avg)

def b_to_i_frame_rot_matrix(q: NDArray[np.float64]) -> NDArray[np.float64]:
    q = normalize_quaternion(q)
    w, x, y, z = q[:, 0]

    # Rotation matrix from body frame to inertial frame (C_b^i)
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)

# TODO: also could just return transpose of b_to_i_frame_rot_matrix
def i_to_b_frame_rot_matrix(q: NDArray[np.float64]) -> NDArray[np.float64]:
    return b_to_i_frame_rot_matrix(inverse_quaternion(q))

def quaternion_exponential(rotation_vector: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Converts a rotation vector (3x1) into a unit quaternion (4x1).
    Formula: q = [cos(theta/2), sin(theta/2) * v/theta]^T
    """
    theta = np.linalg.norm(rotation_vector)
    
    if theta < 1e-12:
        return np.array([[1.0], [0.0], [0.0], [0.0]])
    
    unit_axis = rotation_vector / theta
    
    q_w = np.cos(theta / 2.0)
    q_xyz = unit_axis * np.sin(theta / 2.0)
    
    return np.vstack((np.array([[q_w]]), q_xyz))

def rotate_vector(v: NDArray[np.float64], q: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Rotates a 3x1 vector v using quaternion q (4x1).
    This computes v' = R(q) @ v.
    """
    v = v.reshape(3, 1)
    q = normalize_quaternion(q)
    R = b_to_i_frame_rot_matrix(q)
    return R @ v

def angular_distance_degrees(q_true: NDArray[np.float64], q_est: NDArray[np.float64]) -> float:
        # Relative rotation: q_err = q_est^{-1} x q_true
        q_err = multiply_quaternions(
            inverse_quaternion(q_est),
            q_true,
        )

        q_err = normalize_quaternion(q_err)

        # Enforce shortest rotation (q and -q represent same orientation)
        if q_err[0, 0] < 0.0:
            q_err = -q_err

        w = np.clip(q_err[0, 0], -1.0, 1.0)

        # Geodesic distance on SO(3)
        angle_rad = 2.0 * np.arccos(w)

        return np.degrees(angle_rad)
