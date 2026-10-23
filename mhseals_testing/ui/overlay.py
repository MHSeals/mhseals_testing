"""Project detections into camera pixels for the camera view (perception.md section 8).

Pure geometry, no ROS: callers pass a camera_info K and boxes already expressed
in the camera optical frame (x right, y down, z forward; the /tf_static cache
supplies the transform, as in the minimap). 2D sources use `rect_2d` directly.
"""

import numpy as np

NEAR = 0.05  # m; corners nearer than this are behind or on the camera plane

_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)


def box_corners(center, size):
    """The 8 corners (8x3) of an axis-aligned box given its center and (dx, dy, dz) size."""
    return np.asarray(center, float) + _SIGNS * (np.asarray(size, float) / 2)


def project(points, k):
    """Pixels (Nx2) for camera-frame points (Nx3) with K as the 9-element row-major list or 3x3."""
    k = np.asarray(k, float).reshape(3, 3)
    p = np.asarray(points, float).reshape(-1, 3)
    return np.stack([k[0, 0] * p[:, 0] / p[:, 2] + k[0, 2], k[1, 1] * p[:, 1] / p[:, 2] + k[1, 2]], axis=1)


def rect_3d(corners, k, size):
    """(x0, y0, x1, y1) pixel rect of a projected 3D box clipped to the image size (w, h).

    Returns None when any corner is behind the camera (the box straddles the
    plane, so its rect is unbounded) or when the clipped rect is empty.
    """
    c = np.asarray(corners, float).reshape(-1, 3)
    if len(c) == 0 or np.any(c[:, 2] < NEAR):
        return None
    return clip(project(c, k).min(axis=0).tolist() + project(c, k).max(axis=0).tolist(), size)


def rect_2d(bbox, size):
    """Clip a 2D (x0, y0, x1, y1) detector box to the image size; None when empty."""
    return clip(bbox, size)


def clip(rect, size):
    """Clamp (x0, y0, x1, y1) to [0, w] x [0, h] and round to ints; None if nothing is left."""
    w, h = size
    x0, y0, x1, y1 = (min(max(v, 0.0), lim) for v, lim in zip(rect, (w, h, w, h)))
    if x1 - x0 < 1 or y1 - y0 < 1:
        return None
    return int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))


def scale_rect(rect, factor):
    """Rect after the frame was decoded reduced by `factor` (IMREAD_REDUCED_COLOR_2 -> 0.5)."""
    return tuple(int(round(v * factor)) for v in rect)
