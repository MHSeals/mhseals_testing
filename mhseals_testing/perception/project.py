"""Static transform into the camera optical frame (ROS-free, numpy only).

`to_optical` is built from a /tf_static entry (parent = the box's frame, child =
the optical frame, as published). Corners then go through ui.overlay.rect_3d.
"""

import numpy as np


def rotation(qx, qy, qz, qw):
    """3x3 rotation matrix of a unit quaternion."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_optical(translation, quaternion):
    """(R, t) with p_optical = R p + t, inverting a parent->child /tf_static entry."""
    rot = rotation(*quaternion).T
    return rot, -rot @ np.asarray(translation, float)


def to_frame(points, transform):
    """Points (Nx3) mapped by an (R, t) from `to_optical`."""
    rot, trans = transform
    return np.asarray(points, float).reshape(-1, 3) @ rot.T + trans
