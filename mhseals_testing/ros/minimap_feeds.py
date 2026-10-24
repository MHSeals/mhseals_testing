"""Live feeds for the map: lazy costmap subscriptions, the recorded costmap, trail.jsonl and the map rotation fact.

A costmap is 58-250 KB per message, so its subscription exists only while a
consumer holds a lease on it (``LazySub``). The cheap feeds (pose, plan,
local plan, objects, GPS quality, tf_static) are always on and go straight
into ``MapState``; the session owner also writes trail.jsonl (``want_trail``).
Everything but the ROS wiring in ``MapFeeds`` is pure and tested with fakes.
"""

import math
from pathlib import Path
import threading
import time

import numpy as np

from mhseals_testing import data
from mhseals_testing.analysis.fixlog import state_of
from mhseals_testing.analysis.geo import MapRotation, enu
from mhseals_testing.health.model import Fact
from mhseals_testing.ui.minimap.marks import Obj
from mhseals_testing.ui.minimap.state import UNKNOWN_SPEED, MapState

LOCAL, GLOBAL = '/local_costmap/costmap', '/global_costmap/costmap'
RECORDED = '/mhseals_testing/minimap/costmap'
RECORD_PERIOD = 5.0     # s between recorded costmaps (0.2 Hz)
RECORD_RES = 0.4        # m per cell of the recorded costmap
RECORD_HALF = 40.0      # m around the boat that the recorded costmap covers
LOCAL_HALF = 6.0        # view half-width (m) up to which the local costmap is the source
FACT_PERIOD = 1.0       # s between nav.map_rotation publishes
TRAIL_SCHEMA = 'mhseals.trail/1'
TRAIL_SPACING, TRAIL_PERIOD = 0.25, 1.0     # one crumb per 0.25 m or 1 s


def costmap_source(half_width, choice='auto'):
    """Topic for the costmap layer: 'local' / 'global' as chosen, auto by view half-width."""
    if choice == 'auto':
        choice = 'local' if half_width <= LOCAL_HALF else 'global'
    return LOCAL if choice == 'local' else GLOBAL


def path_points(message):
    """(x, y) of each pose of a nav_msgs/Path."""
    return [(p.pose.position.x, p.pose.position.y) for p in message.poses]


def object_marks(message):
    """marks.Obj per Detection3D of /objects/map: position, class_id as label, score; no result or a non-finite position means no object."""
    out = []
    for d in message.detections:
        if not d.results:
            continue
        hypothesis = d.results[0].hypothesis
        position = d.bbox.center.position
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            continue
        out.append(Obj(position.x, position.y, str(hypothesis.class_id), float(hypothesis.score)))
    return out


def static_transforms(message):
    """{(parent, child): (x, y, z, yaw)} of a /tf_static message."""
    out = {}
    for t in message.transforms:
        q, p = t.transform.rotation, t.transform.translation
        out[(t.header.frame_id, t.child_frame_id)] = (
            p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return out


def should_record(owner, recording, nav_running):
    """The recorded costmap is published only by the session owner, while recording, with Nav2 up."""
    return bool(owner and recording and nav_running)


def decimate(cells, width, height, resolution, origin, centre, out_res=RECORD_RES, half=RECORD_HALF):
    """Max-pool a grid to out_res cells, cropped to +-half m around centre.

    Max-pooling keeps a lethal cell (100) alive and lets unknown (-1) lose to
    any known cost. The crop snaps to the source grid; cells beyond the source
    are unknown. Returns (cells, size, origin) of the square result.
    """
    factor = max(1, round(out_res / resolution))
    size = round(2 * half / (factor * resolution))
    src = np.asarray(cells, dtype=np.int8).reshape(height, width)
    i0 = math.floor((centre[0] - half - origin[0]) / resolution / factor) * factor
    j0 = math.floor((centre[1] - half - origin[1]) / resolution / factor) * factor
    span = size * factor
    out = np.full((span, span), -1, dtype=np.int8)
    x0, y0 = max(i0, 0), max(j0, 0)
    x1, y1 = min(i0 + span, width), min(j0 + span, height)
    if x1 > x0 and y1 > y0:
        out[y0 - j0:y1 - j0, x0 - i0:x1 - i0] = src[y0:y1, x0:x1]
    pooled = out.reshape(size, factor, size, factor).max(axis=(1, 3))
    return pooled.ravel(), size, (origin[0] + i0 * resolution, origin[1] + j0 * resolution)


class LazySub:
    """A subscription that exists only while someone holds a lease on it."""

    def __init__(self, node, message_type, topic, callback, qos):
        self._args = (message_type, topic, callback, qos)
        self.node, self.sub, self.holders = node, None, set()
        self._lock = threading.Lock()

    def lease(self, who, on=True):
        with self._lock:
            self.holders.add(who) if on else self.holders.discard(who)
            if self.holders and self.sub is None:
                self.sub = self.node.create_subscription(*self._args)
            elif not self.holders and self.sub is not None:
                self.node.destroy_subscription(self.sub)
                self.sub = None


class TrailWriter:
    """Appends crumbs to <session>/minimap/trail.jsonl: one per 0.25 m or 1 s, whichever comes first."""

    def __init__(self, session_dir):
        self.target = Path(session_dir) / 'minimap' / 'trail.jsonl'
        self._last = None

    def add(self, t, x, y, epoch=0, fix=None, h_acc_m=None, speed_mps=None, xte_m=None):
        """Write a crumb if due; returns whether it was written. Non-finite values are stored as null."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if self._last and (t - self._last[0] < TRAIL_PERIOD
                           and math.hypot(x - self._last[1], y - self._last[2]) < TRAIL_SPACING):
            return False
        clean = lambda v: v if isinstance(v, (int, float)) and math.isfinite(v) else None
        data.append_jsonl(self.target, dict(
            t_ns=int(t * 1e9), x=x, y=y, epoch=epoch, fix=fix, h_acc_m=clean(h_acc_m),
            speed_mps=clean(speed_mps), xte_m=clean(xte_m)), TRAIL_SCHEMA)
        self._last = (t, x, y)
        return True


def rotation_fact(rotation, now):
    """The nav.map_rotation fact: |rotation| in degrees (signed value and fit rms in data), or None."""
    fit = rotation.fit()
    if fit is None:
        return None
    theta, rms = fit
    degrees = math.degrees(theta)
    return Fact(abs(degrees), label='map', data={'signed_deg': degrees, 'rms_m': rms}, time=now)


class MapFeeds:
    """ROS wiring: map rotation samples always, costmaps only on lease, the decimated costmap while recording."""

    def __init__(self, node, state=None, facts=None, clock=time.monotonic, wall=time.time, on_fix=None):
        """``clock`` (monotonic) ages the rotation fit and fact; ``wall`` stamps crumbs and trail.jsonl,
        as the bag and journal do. ``on_fix`` journals each GPSRAW fix_type change."""
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import NavSatFix
        from tf2_msgs.msg import TFMessage
        from vision_msgs.msg import Detection3DArray
        self.node, self.grid_type, self.clock, self.wall = node, OccupancyGrid, clock, wall
        self.state, self.facts = state if state is not None else MapState(), facts
        self.rotation, self.origin = MapRotation(), None
        self.costmaps = {}           # topic -> latest message
        self.fix_xy = self.gps_xy = self.global_xy = None
        self.on_fix = on_fix         # on_fix(fix=word, fix_type=n): journals a gps_fix transition
        self.fix_changes = []        # (wall s, fix_type) per change, kept live (gps.md section 7)
        self.trail, self.gps = None, (None, None, None)   # TrailWriter (owner only); fix, h_acc_m, speed_mps
        keep = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = {topic: LazySub(node, OccupancyGrid, topic, lambda m, t=topic: self._costmap(t, m), keep)
                     for topic in (LOCAL, GLOBAL)}
        node.create_subscription(NavSatFix, '/gps/fix', self._fix, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom/gps', self._odom('gps_xy'), 10)
        node.create_subscription(Odometry, '/odom/global', self._odom('global_xy', sample=True), 10)
        self._subscribe_state(node, PathMsg, Detection3DArray, TFMessage)
        self.fact_timer = node.create_timer(FACT_PERIOD, self.publish_fact)
        self.publisher = node.create_publisher(OccupancyGrid, RECORDED, keep)
        self.timer = node.create_timer(RECORD_PERIOD, self._publish)
        self.record = False
        self.layer_topic = None      # the one costmap topic the layers read into state

    def _subscribe_state(self, node, path_type, objects_type, tf_type):
        """The always-on feeds into MapState; GPSRAW only when mavros_msgs is installed."""
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        ingest = self.state.ingest
        node.create_subscription(path_type, '/plan', lambda m: ingest('plan', points=path_points(m)), 10)
        node.create_subscription(path_type, '/local_plan', lambda m: ingest('local_plan', data=path_points(m)), 10)
        node.create_subscription(objects_type, '/objects/map',
                                 lambda m: ingest('objects', items=object_marks(m)), 10)
        node.create_subscription(tf_type, '/tf_static',
                                 lambda m: ingest('tf_static', transforms=static_transforms(m)),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            from mavros_msgs.msg import GPSRAW
        except ImportError:
            return
        node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self._gps_raw, qos_profile_sensor_data)

    def _gps_raw(self, m):
        if m.fix_type != self.gps[0]:
            self.fix_changes.append((self.wall(), m.fix_type))
            if self.on_fix:
                self.on_fix(fix=state_of({'fix_type': m.fix_type}), fix_type=m.fix_type)
        self.state.ingest('gps_raw', fix_type=m.fix_type, h_acc_m=m.h_acc / 1000, vel_cms=m.vel)
        self.gps = (m.fix_type, m.h_acc / 1000, None if m.vel == UNKNOWN_SPEED else m.vel / 100)

    def want_trail(self, owner, session_dir):
        """The session owner writes <session>/minimap/trail.jsonl; anyone else (or no session) drops the writer."""
        if not (owner and session_dir):
            self.trail = None
        elif self.trail is None or self.trail.target.parent.parent != Path(session_dir):
            self.trail = TrailWriter(session_dir)

    def want_layers(self, costmap, hits, half_width, choice='auto'):
        """Lease the costmap the view needs while the c or r layer is on; drop the other source."""
        wanted = costmap_source(half_width, choice) if costmap or hits else None
        for topic, sub in self.subs.items():
            sub.lease('layers', topic == wanted)
            if topic != wanted and not sub.holders:
                self.costmaps.pop(topic, None)
        if wanted != self.layer_topic:
            self.layer_topic = wanted
            self.state.latest.pop('costmap', None)

    def want_record(self, owner, recording, nav_running):
        """Hold the global costmap and publish the decimated one under should_record's gate."""
        self.record = should_record(owner, recording, nav_running)
        self.subs[GLOBAL].lease('recorder', self.record)

    def _costmap(self, topic, message):
        """Keep the newest grid per topic; only the layers' topic goes to the map state (the recorder's may differ)."""
        self.costmaps[topic] = message
        if topic == self.layer_topic:
            self.state.ingest('costmap', data=message)

    def _fix(self, m):
        if m.status.status >= 0 and math.isfinite(m.latitude):
            self.origin = self.origin or (m.latitude, m.longitude)
            self.fix_xy = enu(m.latitude, m.longitude, self.origin)

    def _odom(self, name, sample=False):
        def callback(m):
            now = self.clock()
            setattr(self, name, (m.pose.pose.position.x, m.pose.pose.position.y))
            if sample:
                q, wall = m.pose.pose.orientation, self.wall()
                self.state.ingest('pose_map', t=wall, x=self.global_xy[0], y=self.global_xy[1],
                                  yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                if self.trail:
                    fix, h_acc, speed = self.gps
                    self.trail.add(wall, *self.global_xy, epoch=self.state.epoch, fix=fix, h_acc_m=h_acc,
                                   speed_mps=speed, xte_m=self.state.xte(self.global_xy))
                if self.fix_xy and self.gps_xy:
                    self.rotation.add(now, self.gps_xy, self.global_xy, self.fix_xy)
        return callback

    def fact(self):
        return rotation_fact(self.rotation, self.clock())

    def publish_fact(self):
        """Publish nav.map_rotation into the health facts; none while the fit lacks baseline."""
        fact = self.fact()
        if self.facts is not None:
            self.facts.publish('map', {} if fact is None else {'nav.map_rotation': fact})

    def _publish(self):
        grid = self.costmaps.get(GLOBAL)
        if not (self.record and grid and self.global_xy):
            return
        info = grid.info
        cells, size, origin = decimate(grid.data, info.width, info.height, info.resolution,
                                       (info.origin.position.x, info.origin.position.y), self.global_xy)
        out = self.grid_type()
        out.header.frame_id, out.header.stamp = grid.header.frame_id, self.node.get_clock().now().to_msg()
        out.info.resolution, out.info.width, out.info.height = RECORD_RES, size, size
        out.info.origin.position.x, out.info.origin.position.y = origin
        out.info.origin.orientation.w = 1.0
        out.data = cells.tolist()
        self.publisher.publish(out)
