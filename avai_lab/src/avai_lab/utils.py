import numpy as np

def to_unit_vec(v: np.ndarray):
    """Return the normalized unit vector (length = 1) of a given vector"""
    if v.sum() == 0:
        return v
    return v / np.linalg.norm(v)

def angle_between(v1: np.ndarray, v2: np.ndarray):
    """Return the angle between two vectors in radian"""
    return np.arccos(np.clip(np.dot(to_unit_vec(v1), to_unit_vec(v2)), -1.0, 1.0))

def quat_to_rot_vec(z: float, w: float) -> float:
    """Calculate the rotation in radians on the z-axis given the (z, w) component of a quaternion"""
    return 2 * np.arctan2(z, w)

def get_direction_vec(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Calculate the vector between a start and an end point"""
    return end - start

def rot_from_vec(v: np.ndarray):
    """Calculate the rotation in radian from a vector"""
    return np.arctan2(*v[::-1])
