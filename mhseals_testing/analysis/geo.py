"""Local-frame geometry for the map: GPS to ENU and a 2-D rigid fit; no ROS imports."""

import math

EARTH_RADIUS = 6378137.0
MIN_SPREAD = 1e-6  # metres; below this the source has no usable baseline


def enu(lat, lon, origin):
    """East and north metres of (lat, lon) from origin (lat, lon), in degrees.

    Equirectangular: under 1 cm of error within 1 km.
    """
    lat0, lon0 = origin
    dlon = (lon - lon0 + 180.0) % 360.0 - 180.0  # shortest way round the antimeridian
    east = math.radians(dlon) * math.cos(math.radians(lat0)) * EARTH_RADIUS
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


class MapRotation:
    """Rolling fit of /odom/gps against enu(/gps/fix): the map frame's rotation.

    Samples are dropped while /odom/gps still sits at its (0, 0) start value
    and until /odom/global has been live for settle seconds, because the
    navsat datum and ekf_global are still converging. A map-frame jump over
    jump metres in either odom stream clears the pairs and restarts the wait.
    """

    PAIRS = 200      # newest pairs kept for the fit
    BASELINE = 5.0   # metres of spread the pairs need before a fit is trusted
    SETTLE = 10.0    # seconds of /odom/global before samples count
    JUMP = 5.0       # metres; the home re-projection threshold
    SPACING = 0.5    # metres of travel between kept pairs, so slow runs still fill the buffer

    def __init__(self, settle=SETTLE, jump=JUMP, spacing=SPACING):
        self.settle = settle
        self.jump = jump
        self.spacing = spacing
        self._pairs = []
        self._live = None   # time of the first /odom/global sample since a reset
        self._last = {}     # last odom point per stream, for jump detection

    def _jumped(self, name, point):
        last = self._last.get(name)
        self._last[name] = point
        return last is not None and math.hypot(point[0] - last[0], point[1] - last[1]) > self.jump

    def add(self, t, odom_gps, odom_global, enu_xy):
        """Offer one stamped sample: /odom/gps xy, /odom/global xy and enu(/gps/fix) xy.

        A sample with any non-finite coordinate is ignored whole.
        """
        if not all(map(math.isfinite, (*odom_gps, *odom_global, *enu_xy))):
            return
        if self._jumped('global', odom_global) | self._jumped('gps', odom_gps):
            self._pairs.clear()
            self._live = None
        if self._live is None:
            self._live = t
        if tuple(odom_gps) == (0.0, 0.0) or t - self._live < self.settle:
            return
        if self._pairs:
            last = self._pairs[-1][0]
            if math.hypot(enu_xy[0] - last[0], enu_xy[1] - last[1]) < self.spacing:
                return
        self._pairs.append((tuple(enu_xy), tuple(odom_gps)))
        del self._pairs[:-self.PAIRS]

    def fit(self):
        """(theta radians, rms metres) of the kept pairs, or None without enough baseline."""
        if len(self._pairs) < 2:
            return None
        first = self._pairs[0][0]
        if max(math.hypot(s[0] - first[0], s[1] - first[1]) for s, _ in self._pairs) < self.BASELINE:
            return None
        try:
            theta, _, _, rms = fit_rigid(self._pairs)
        except ValueError:
            return None
        return theta, rms


def ecef(lat, lon, alt):
    """WGS84 ECEF (x, y, z) metres of geodetic lat/lon in degrees and ellipsoid height in metres."""
    a, f = EARTH_RADIUS, 1 / 298.257223563
    e2 = f * (2 - f)
    sin_lat, cos_lat = math.sin(math.radians(lat)), math.cos(math.radians(lat))
    n = a / math.sqrt(1 - e2 * sin_lat ** 2)
    return ((n + alt) * cos_lat * math.cos(math.radians(lon)),
            (n + alt) * cos_lat * math.sin(math.radians(lon)),
            (n * (1 - e2) + alt) * sin_lat)


def baseline(base_ecef, lat, lon, alt):
    """ECEF distance in metres from the base (x, y, z) to the rover's lat/lon/alt (GPS_RAW_INT, no GPS_RTK)."""
    return math.dist(base_ecef, ecef(lat, lon, alt))
