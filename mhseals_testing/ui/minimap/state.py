"""Pure minimap state (minimap_layers M2): what the map layers draw from.

ROS-free. Ingest is always on and cheap: `ingest(kind, **fields)` keeps only the latest
message per layer, plus the trail crumbs, and never draws. Fields are plain numbers and
tuples already pulled out of the ROS messages by the feed (ros/minimap_feeds.py).
World points are (x, y) in the navsat ENU map frame, metres.

Kinds: pose_map, gps_raw, plan, local_plan, objects, detections, hits, costmap, tf_static.
"""

import math
from collections import deque

SPACING = 0.25     # m of travel between crumbs
MAX_AGE = 1.0      # s; a crumb is also dropped after this long without one
RING = 4000        # crumbs kept
GOAL_MOVE = 0.3    # m the plan's end moves for a new goal
DATUM_JUMP = 5.0   # m of map-frame jump in the pose that starts a new epoch
FLOORS = (0.0, 0.3, 0.5, 0.7)  # detection confidence floors, `D` cycles them
LATEST = ('local_plan', 'hits', 'costmap')  # kept as given, newest only
UNKNOWN_SPEED = 65535  # GPSRAW vel when the FCU does not know


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
        self._last = None         # last crumb (t, x, y)

    def ingest(self, kind, **f):
        """Take one message of ``kind``; unknown kinds raise ValueError."""
        if kind == 'pose_map':
            self._pose(f['t'], f['x'], f['y'], f.get('yaw', 0.0))
        elif kind == 'gps_raw':
            speed = f.get('vel_cms', UNKNOWN_SPEED)
            self._gps = (f['fix_type'], f['h_acc_m'], None if speed == UNKNOWN_SPEED else speed / 100)
        elif kind == 'plan':
            self._plan(f['points'])
        elif kind == 'objects':
            self.objects = list(f['items'])
        elif kind == 'detections':
            self._detections = list(f['items'])
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
        self._last = None

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
        self.crumbs.append({'t': t, 'x': x, 'y': y, 'epoch': self.epoch, 'fix': fix,
                            'h_acc_m': h_acc, 'speed_mps': speed, 'xte_m': self.xte((x, y))})
        self._last = (t, x, y)
