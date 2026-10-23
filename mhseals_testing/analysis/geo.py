"""Local-frame geometry for the map: GPS to ENU and a 2-D rigid fit; no ROS imports."""

import math

EARTH_RADIUS = 6378137.0
MIN_SPREAD = 1e-6  # metres; below this the source has no usable baseline


def enu(lat, lon, origin):
    """East and north metres of (lat, lon) from origin (lat, lon), in degrees.

    Equirectangular: under 1 cm of error within 1 km.
    """
    lat0, lon0 = origin
    east = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_RADIUS
    north = math.radians(lat - lat0) * EARTH_RADIUS
    return east, north


def geodetic(east, north, origin):
    """Inverse of enu: (lat, lon) in degrees."""
    lat0, lon0 = origin
    lat = lat0 + math.degrees(north / EARTH_RADIUS)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat0))))
    return lat, lon


def fit_rigid(pairs):
    """Least-squares rotation and translation taking source onto destination points.

    pairs is [((x, y), (x2, y2)), ...]. Returns (theta, tx, ty, rms) with
    dst = R(theta) @ src + (tx, ty); the 2-D Umeyama fit without scale.
    Raises ValueError on fewer than two pairs, a non-finite coordinate, or
    coincident source points, where the rotation is undefined.
    """
    count = len(pairs)
    if count < 2:
        raise ValueError('need at least two pairs')
    if not all(math.isfinite(c) for src, dst in pairs for c in (*src, *dst)):
        raise ValueError('non-finite coordinate')
    sx = sum(s[0] for s, _ in pairs) / count
    sy = sum(s[1] for s, _ in pairs) / count
    dx = sum(d[0] for _, d in pairs) / count
    dy = sum(d[1] for _, d in pairs) / count
    if max(math.hypot(s[0] - sx, s[1] - sy) for s, _ in pairs) < MIN_SPREAD:
        raise ValueError('source points are coincident')
    dot = cross = 0.0
    for (x, y), (u, v) in pairs:
        x, y, u, v = x - sx, y - sy, u - dx, v - dy
        dot += x * u + y * v
        cross += x * v - y * u
    theta = math.atan2(cross, dot)
    cos, sin = math.cos(theta), math.sin(theta)
    tx = dx - (cos * sx - sin * sy)
    ty = dy - (sin * sx + cos * sy)
    squares = sum((cos * x - sin * y + tx - u) ** 2 + (sin * x + cos * y + ty - v) ** 2
                  for (x, y), (u, v) in pairs)
    return theta, tx, ty, math.sqrt(squares / count)
