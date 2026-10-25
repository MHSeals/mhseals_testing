"""Pure minimap state (minimap_layers M2): what the map layers draw from.

ROS-free. Ingest is always on and cheap: `ingest(kind, **fields)` keeps only the latest
message per layer, plus the trail crumbs, and never draws. Fields are plain numbers and
tuples already pulled out of the ROS messages by the feed (ros/minimap_feeds.py).
World points are (x, y) in the navsat ENU map frame, metres.

Kinds: pose_map, gps_raw, gps_cov, plan, local_plan, objects, detections, hits, costmap, tf_static, fov,
and from the mission feed (ui/feeds/mission.py) tasks (minimap.tasks.Task items) and mission_goal.
The trail is also kept smoothed (one Chaikin corner cut per crumb, O(1)) for drawing.
"""

import math
from collections import deque

from mhseals_testing.ui.minimap.layers import Crumb

SPACING = 0.25     # m of travel between crumbs
MAX_AGE = 1.0      # s; a crumb is also dropped after this long without one
RING = 4000        # crumbs kept
GOAL_MOVE = 0.3    # m the plan's end moves for a new goal
DATUM_JUMP = 5.0   # m of map-frame jump in the pose that starts a new epoch
FLOORS = (0.0, 0.3, 0.5, 0.7)  # detection confidence floors, `D` cycles them
LATEST = ('local_plan', 'hits', 'costmap')  # kept as given, newest only
UNKNOWN_SPEED = 65535  # GPSRAW vel when the FCU does not know
SMOOTH_MIN = 0.15  # m a crumb must be from the last smoothed point to be cut against it
NAVSAT_BANDS = {-1: 'none', 0: '3d', 1: 'dgps', 2: 'float'}   # NavSatStatus.status -> fix band


def segment_distance(p, a, b):
    """Distance from point ``p`` to the segment ``a``-``b``."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def path_distance(p, path):
    """Distance from ``p`` to the polyline ``path``; None for an empty path."""
    if len(path) == 1:
        return math.hypot(p[0] - path[0][0], p[1] - path[0][1])
    return min((segment_distance(p, a, b) for a, b in zip(path, path[1:])), default=None)


class MapState:
    """Latest layer data, trail crumbs, the goal's initial plan and the map epoch."""

    def __init__(self, floor=0.0, spacing=SPACING, ring=RING):
        self.floor = floor
        self.spacing = spacing
        self.crumbs = deque(maxlen=ring)
        self.epoch = 0
        self.pose = None          # (t, x, y, yaw)
        self.plan = []            # latest /plan
        self.initial_plan = []    # first /plan of the current goal: the xte reference
        self.objects = []
        self._detections = []     # raw detections; the floor is applied on read
        self.latest = {}          # local_plan, hits, costmap
        self.tf_static = {}       # (parent, child) -> (x, y, z, yaw), kept for the session
        self._gps = (0, 0.0, None)  # fix_type, h_acc_m, speed_mps
        self._cov = None          # (h_acc_m, band) from the navsat covariance, when GPSRAW is absent
        self._raw = False         # a GPSRAW has arrived: it is the accuracy source
        self._last = None         # last crumb (t, x, y)
        self.smooth = deque(maxlen=ring * 2)   # smoothed layers.Crumb tuples the trail painter walks
        self.fov = None           # {hfov_deg, range_m, frame} of the camera backend
        self.tasks = []           # minimap.tasks.Task per /task/hypotheses entry
        self.mission_goal = None  # (x, y, yaw, why) of the mission's current goal

    def ingest(self, kind, **f):
        """Take one message of ``kind``; unknown kinds raise ValueError."""
        if kind == 'pose_map':
            self._pose(f['t'], f['x'], f['y'], f.get('yaw', 0.0))
        elif kind == 'gps_raw':
            speed = f.get('vel_cms', UNKNOWN_SPEED)
            self._gps = (f['fix_type'], f['h_acc_m'], None if speed == UNKNOWN_SPEED else speed / 100)
            self._raw = True
        elif kind == 'gps_cov':
            self._cov = (f['h_acc_m'], NAVSAT_BANDS.get(f.get('status', 0), 'none'))
        elif kind == 'fov':
            self.fov = {k: f[k] for k in ('hfov_deg', 'range_m', 'frame')}
        elif kind == 'plan':
            self._plan(f['points'])
        elif kind == 'objects':
            self.objects = list(f['items'])
        elif kind == 'detections':
            self._detections = list(f['items'])
        elif kind == 'tasks':
            self.tasks = list(f['items'])
        elif kind == 'mission_goal':
            self.mission_goal = None if f.get('x') is None else (f['x'], f['y'], f.get('yaw', 0.0), f.get('why', ''))
        elif kind == 'tf_static':
            self.tf_static.update(f['transforms'])
        elif kind in LATEST:
            self.latest[kind] = f['data']
        else:
            raise ValueError(f'unknown minimap kind {kind!r}')

    def set_floor(self, floor):
        """Set the detection floor; applies at once, up or down, to the held detections."""
        self.floor = floor

    @property
    def detections(self):
        """Raw detections at or above the floor."""
        return [d for d in self._detections if d['score'] >= self.floor]

    def new_epoch(self):
        """Start a new map epoch (the datum moved); older crumbs stay and are drawn DIM.

        The initial plan is dropped: it is in the old frame, the next /plan becomes the reference.
        """
        self.epoch += 1
        self.initial_plan = []
        self._last = None

    def restart_trail(self):
        """`Y`: forget the trail, keep the epoch."""
        self.crumbs.clear()
        self.smooth.clear()
        self._last = None

    def accuracy(self):
        """``(h_acc_m, band)`` of the GPS fix, or None: GPSRAW h_acc (band from fix_type) when it
        has arrived, else the navsat covariance. One read-only source for the map and the popups."""
        from mhseals_testing.ui.minimap.layers import FIX_BANDS
        if self._raw:
            fix, h_acc, _ = self._gps
            return (h_acc, FIX_BANDS.get(fix, 'none')) if h_acc and math.isfinite(h_acc) else None
        return self._cov

    @property
    def speed(self):
        """GPSRAW ground speed in m/s, or None."""
        return self._gps[2]

    def course(self):
        """Course over ground in map degrees (CCW from east) from the last two crumbs, or None."""
        if len(self.crumbs) < 2:
            return None
        a, b = self.crumbs[-2], self.crumbs[-1]
        if a['epoch'] != b['epoch'] or math.hypot(b['x'] - a['x'], b['y'] - a['y']) < SMOOTH_MIN:
            return None
        return math.degrees(math.atan2(b['y'] - a['y'], b['x'] - a['x']))

    def camera(self):
        """``(x, y, yaw_rad)`` in the map of the backend's camera frame, or None without a pose or fov.

        The boat pose composed with the static chain base -> ``frame`` (walked child to parent in
        tf_static); an optical frame uses its parent, whose x is forward. Missing links leave
        the camera at the boat, looking along the bow.
        """
        if self.pose is None or not self.fov:
            return None
        parent = {child: (p, t) for (p, child), t in self.tf_static.items()}
        frame, chain = self.fov.get('frame') or '', []
        if 'optical' in frame and frame in parent:
            frame = parent[frame][0]
        while frame in parent and len(chain) < 16:
            frame, link = parent[frame]
            chain.append(link)
        x, y, yaw = 0.0, 0.0, 0.0
        for tx, ty, _, tyaw in chain:   # child-most first: fold each link under its parent
            c, s = math.cos(tyaw), math.sin(tyaw)
            x, y, yaw = tx + c * x - s * y, ty + s * x + c * y, tyaw + yaw
        _, bx, by, byaw = self.pose
        c, s = math.cos(byaw), math.sin(byaw)
        return bx + c * x - s * y, by + s * x + c * y, byaw + yaw

    def xte(self, point):
        """Distance from ``point`` to the initial plan of the current goal, or None."""
        return path_distance(point, self.initial_plan) if self.initial_plan else None

    def _plan(self, points):
        points = [tuple(p) for p in points]
        self.plan = points
        if not points:
            return
        end = self.initial_plan[-1] if self.initial_plan else None
        if end is None or math.hypot(points[-1][0] - end[0], points[-1][1] - end[1]) > GOAL_MOVE:
            self.initial_plan = points

    def _pose(self, t, x, y, yaw):
        if not all(map(math.isfinite, (t, x, y))):
            return
        if self.pose and math.hypot(x - self.pose[1], y - self.pose[2]) > DATUM_JUMP:
            self.new_epoch()
        self.pose = (t, x, y, yaw)
        if self._last and math.hypot(x - self._last[1], y - self._last[2]) < self.spacing \
                and t - self._last[0] < MAX_AGE:
            return
        fix, h_acc, speed = self._gps
        crumb = {'t': t, 'x': x, 'y': y, 'epoch': self.epoch, 'fix': fix,
                 'h_acc_m': h_acc, 'speed_mps': speed, 'xte_m': self.xte((x, y))}
        self.crumbs.append(crumb)
        self._last = (t, x, y)
        self._smooth(crumb)

    def _smooth(self, crumb):
        """Append to the smoothed trail: the previous point is cut into its 1/4 and 3/4 points
        against its neighbours (Chaikin), so a frame never smooths; an epoch break restarts it."""
        sm, c = self.smooth, Crumb(crumb['x'], crumb['y'], crumb['fix'], crumb['xte_m'], crumb['speed_mps'],
                                   crumb['epoch'], crumb['t'])
        if sm and sm[-1].epoch == c.epoch:
            if math.hypot(c.x - sm[-1].x, c.y - sm[-1].y) < SMOOTH_MIN:
                return
            if len(sm) >= 2 and sm[-2].epoch == c.epoch:
                a, b = sm[-2], sm.pop()
                sm.append(b._replace(x=0.75 * b.x + 0.25 * a.x, y=0.75 * b.y + 0.25 * a.y))
                sm.append(b._replace(x=0.75 * b.x + 0.25 * c.x, y=0.75 * b.y + 0.25 * c.y))
        sm.append(c)
