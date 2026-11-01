"""The Fusion tab's data: lidar points, boxes and the extrinsic correction, only while the tab is shown.

The tab (ui/panels/fusion.py) wants it through ``want(on, camera=...)``. Off, every subscription this
feed made is destroyed and its data forgotten; /points and /tf_static are the session's own
subscriptions (RosIO taps), so the tab adds no lidar traffic and drops scans before decoding them.
Scans are decimated and projected into the camera at ``points_hz``; the camera frame (only in an
image tier on an unbudgeted link) is cut to a small greyscale on arrival. ``snapshot()`` is a plain
dict the panel paints from; nothing here draws. Projection and the correction are lidar_fusion's
(``axes_matrix``, ``small_rotation``), so the tab shows the points the fusion associates.
"""

import io
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'fusion_panel.yaml'
KEYS = {'topics': {'observations', 'fused', 'sources', 'camera_info'},
        'feed': {'points_hz', 'points_max', 'camera_hz', 'camera_width', 'max_range_m', 'residual_s', 'stale_s'},
        'ramp_m': None, 'rings_m': None, 'draw_hz': None,
        'check': {'near_m', 'du_pass_px', 'du_warn_px', 'dv_pass_px', 'dv_warn_px', 'hit_rate_min',
                  'range_tol', 'range_fail_tol', 'min_samples'}}
TF_STATIC = '/tf_static'   # its own latched subscription: the session's got the transforms before the tab opened
SCANS = 3            # projected scans held, so boxes pair with the scan nearest their capture


def load_config(path=CONFIG):
    """fusion_panel.yaml, every key required and none unknown."""
    doc = load_strict(Path(path).read_text(), path)
    if set(doc) != set(KEYS):
        raise ValueError(f'{path}: keys {sorted(doc)} != {sorted(KEYS)}')
    for group, keys in KEYS.items():
        if keys is not None and set(doc[group]) != keys:
            raise ValueError(f'{path}: {group} keys {sorted(doc[group])} != {sorted(keys)}')
    return doc


def fusion_math():
    """lidar_fusion's frame helpers, imported on first use (mhseals_nav)."""
    from mhseals_nav.lidar_fusion import axes_matrix, small_rotation
    return axes_matrix, small_rotation


def quat_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class StaticTF:
    """/tf_static edges as full rotations: ``between(target, source)`` maps source points into target."""

    def __init__(self):
        self.edges = {}       # child -> (parent, R, t): p_parent = R p_child + t

    def add(self, message):
        for tf in message.transforms:
            q, v = tf.transform.rotation, tf.transform.translation
            self.edges[tf.child_frame_id.lstrip('/')] = (
                tf.header.frame_id.lstrip('/'), quat_matrix(q.x, q.y, q.z, q.w), np.array([v.x, v.y, v.z]))

    def _to_root(self, frame):
        R, t, seen = np.eye(3), np.zeros(3), set()
        while frame in self.edges and frame not in seen:
            seen.add(frame)
            parent, Rp, tp = self.edges[frame]
            R, t, frame = Rp @ R, Rp @ t + tp, parent
        return frame, R, t

    def between(self, target, source):
        """(R, t) with p_target = R p_source + t, or None when the frames share no root."""
        root_s, Rs, ts = self._to_root(source.lstrip('/'))
        root_t, Rt, tt = self._to_root(target.lstrip('/'))
        if root_s != root_t:
            return None
        return Rt.T @ Rs, Rt.T @ (ts - tt)


def cloud_xyz(message, keep):
    """Nx3 float32 x, y, z of a PointCloud2, every k-th point so at most ``keep`` remain."""
    offsets = {f.name: f.offset for f in message.fields}
    if not {'x', 'y', 'z'} <= set(offsets) or not message.point_step:
        return np.zeros((0, 3), np.float32)
    rows = np.frombuffer(bytes(message.data), np.uint8)
    rows = rows[:len(rows) // message.point_step * message.point_step].reshape(-1, message.point_step)
    rows = rows[::max(1, -(-len(rows) // keep))]
    xyz = np.stack([rows[:, offsets[a]:offsets[a] + 4].copy().view('<f4')[:, 0] for a in 'xyz'], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def grey_frame(jpeg, width):
    """A JPEG as an HxW uint8 greyscale at most ``width`` px wide (Pillow on first frame), or None."""
    try:
        from PIL import Image
        image = Image.open(io.BytesIO(jpeg))
        image.draft('L', (width, width))          # JPEG DCT scaling: decodes at a fraction of the size
        image = image.convert('L')
        if image.width > width:
            image = image.resize((width, max(1, image.height * width // image.width)))
        return np.asarray(image)
    except Exception:
        return None


class FusionFeed:
    """Holds the newest boxes, projected scans, extrinsic samples and camera frame for the Fusion tab.

    ``subscribe(kind, topic, callback)`` returns a handle, ``unsubscribe(handle)`` destroys it; kinds are
    ``observations``, ``sources``, ``camera_info``, ``tf_static`` (latched: it redelivers on subscribe). ``camera`` is a CameraFeed (or None). Callbacks run in
    the executor thread, so each one guards against a malformed message.
    """

    def __init__(self, subscribe, unsubscribe, camera=None, cfg=None, clock=time.monotonic):
        self.cfg = cfg or load_config()
        self._subscribe, self._unsubscribe, self.camera, self._clock = subscribe, unsubscribe, camera, clock
        self._lock = threading.Lock()
        self.tf = StaticTF()          # kept while closed; /tf_static is redelivered on each open
        self._handles = {}
        self._camera_on = False
        self._reset()

    def _reset(self):
        self.K = self.size = None
        self.raw = self.fused = None          # (t, frame, [obs]) newest arrays
        self.sources = None                   # (t, [label]) of the newest fused array
        self.scans = deque(maxlen=SCANS)      # (t, uv Nx2 normalised, range N, optical x/z Mx2 of all in front)
        self.residuals = deque()              # (received, du_px, dv_px)
        self.frame = None                     # (grey HxW, received)
        self._points_at = self._frame_at = None
        self.version = 0

    @property
    def is_open(self):
        return bool(self._handles)

    def want(self, on, camera=False):
        """Subscribe while ``on`` (the tab shown); ``camera`` adds the frame (image tier, no link budget)."""
        if on and not self._handles:
            topics = self.cfg['topics']
            for kind, topic, callback in (('observations', topics['observations'], self._on_raw),
                                          ('observations', topics['fused'], self._on_fused),
                                          ('sources', topics['sources'], self._on_sources),
                                          ('camera_info', topics['camera_info'], self._on_camera_info),
                                          ('tf_static', TF_STATIC, self.on_tf_static)):
                self._handles[topic] = self._subscribe(kind, topic, callback)
        elif not on and self._handles:
            for handle in self._handles.values():
                self._unsubscribe(handle)
            self._handles = {}
            with self._lock:
                self._reset()
        camera = bool(on and camera and self.camera is not None)
        if camera != self._camera_on:
            self.camera.open() if camera else self.camera.close()
            self._camera_on = camera
            if not camera:
                with self._lock:
                    self.frame = None

    # -- callbacks (executor thread) -------------------------------------------------------------

    def on_tf_static(self, message):
        try:
            with self._lock:
                self.tf.add(message)
        except Exception:
            pass

    def points_due(self):
        """Whether the next scan is wanted (checked before it is decoded)."""
        now = self._clock()
        return self.is_open and (self._points_at is None or now - self._points_at >= 1.0 / self.cfg['feed']['points_hz'])

    def on_points(self, message):
        """A decoded PointCloud2: decimate, put it in the camera frame with the applied correction, project."""
        try:
            self._points_at = self._clock()
            with self._lock:
                frame, K, size = (self.raw or self.fused or (None, None))[1], self.K, self.size
                pose = None if frame is None else self.tf.between(frame, message.header.frame_id)
            if K is None or pose is None:
                return
            axes_matrix, small_rotation = fusion_math()
            R, t = pose
            optical = axes_matrix(frame, 'auto')
            p = (cloud_xyz(message, self.cfg['feed']['points_max']) @ R.T + t) @ optical.T
            p = p @ small_rotation(-np.radians(self.applied_deg())).T     # as lidar_fusion associates them
            rng = np.linalg.norm(p, axis=1)
            front = (p[:, 2] > 0.1) & (rng < self.cfg['feed']['max_range_m'])
            p, rng = p[front], rng[front]
            k = np.asarray(K, float).reshape(3, 3)
            uv = np.stack([(k[0, 0] * p[:, 0] / p[:, 2] + k[0, 2]) / size[0],
                           (k[1, 1] * p[:, 1] / p[:, 2] + k[1, 2]) / size[1]], axis=1)
            inside = (uv >= 0).all(axis=1) & (uv < 1).all(axis=1)
            with self._lock:
                self.scans.append((stamp(message.header), uv[inside].astype(np.float32),
                                   rng[inside].astype(np.float32), p[:, [0, 2]].astype(np.float32)))
                self.version += 1
        except Exception:
            pass

    def _on_camera_info(self, message):
        try:
            with self._lock:
                self.K, self.size = [float(v) for v in message.k], (int(message.width), int(message.height))
        except Exception:
            pass

    def _array(self, message):
        return (stamp(message.header), message.header.frame_id.lstrip('/'), [
            {'cls': str(o.class_id), 'box': tuple(float(v) for v in o.box2d) if o.has_box2d else None,
             'pos': (o.position.x, o.position.y, o.position.z), 'cov': tuple(o.position_covariance),
             'id': str(o.backend_id)} for o in message.observations])

    def _on_raw(self, message):
        try:
            array = self._array(message)
        except Exception:
            return
        with self._lock:
            self.raw, self.version = array, self.version + 1

    def _on_fused(self, message):
        try:
            array = self._array(message)
        except Exception:
            return
        with self._lock:
            self.fused, self.version = array, self.version + 1

    def _on_sources(self, message):
        """Source labels per fused box, and the extrinsic samples as pixel residuals.

        A sample is lidar_fusion's ``(x, y, z, b, r_az, r_el, ...)``: the box centre's normalised image coordinates
        and the cluster's angular offset from it, so the pixel offset (fusion_calib's du/dv, cluster centre minus
        box centre) is ``fx (tan(atan x + r_az) - x)``, not ``r_az fx``, which shrinks off-centre boxes.
        """
        try:
            doc = json.loads(message.data)
            labels = [str(s) for s in doc.get('sources', [])]
            samples = [(float(e[0]), float(e[1]), float(e[4]), float(e[5])) for e in doc.get('ex') or []]
        except Exception:
            return
        now = self._clock()
        with self._lock:
            self.sources = (float(doc.get('t', 0.0)), labels)
            if self.K is not None:
                for x, y, r_az, r_el in samples:
                    self.residuals.append((now, self.K[0] * (math.tan(math.atan(x) + r_az) - x),
                                           self.K[4] * (math.tan(math.atan(y) + r_el) - y)))
            while self.residuals and self.residuals[0][0] < now - self.cfg['feed']['residual_s']:
                self.residuals.popleft()

    # -- the panel's view ------------------------------------------------------------------------

    applied = (0.0, 0.0, 0.0)     # deg, optical x y z: set from the fusion status each frame (facts)

    def applied_deg(self):
        return self.applied

    def snapshot(self):
        """A plain dict for ui/panels/fusion.py (see its ``model`` docstring)."""
        now = self._clock()
        stale = self.cfg['feed']['stale_s']
        if self._camera_on and self.camera is not None:
            latest = self.camera.latest()
            if latest is not None and (self.frame is None or latest[1] != self._frame_at):
                self._frame_at = latest[1]
                grey = grey_frame(latest[0], self.cfg['feed']['camera_width'])
                with self._lock:
                    self.frame = None if grey is None else (grey, latest[1])
                    self.version += 1
        with self._lock:
            fresh = self._points_at is not None and now - self._points_at < stale + 1.0 / self.cfg['feed']['points_hz']
            dets = detections(self.raw, self.fused, self.sources)
            t = (self.fused or self.raw or (None,))[0]
            scan = min(self.scans, key=lambda s: abs(s[0] - t)) if self.scans and t is not None else (
                self.scans[-1] if self.scans else None)
            return {'version': self.version, 'size': self.size, 'K': self.K, 'dets': dets,
                    'fused': self.fused is not None, 'points': scan if fresh else None,
                    'frame': None if self.frame is None else self.frame[0],
                    'residuals': [(t - now, du, dv) for t, du, dv in self.residuals]}


def _range(pos):
    r = math.sqrt(sum(v * v for v in pos))
    return r if math.isfinite(r) and r > 0 else None


def _sigma(pos, cov):
    """Range SD along the ray from a 3x3 covariance (None when the backend gave none)."""
    if not any(cov):
        return None
    r = _range(pos)
    if r is None:
        return None
    u = np.asarray(pos) / r
    var = float(u @ np.asarray(cov, float).reshape(3, 3) @ u)
    return math.sqrt(var) if var > 0 else None


def detections(raw, fused, sources):
    """One row per box: class, box, fused and stereo range, source, range SD and optical x/z.

    Fused boxes pair with the stereo box of the same box2d (the fusion copies it); with no fused
    array every stereo box is a ``stereo`` row. Positions are in each array's frame; the panel puts
    them in optical axes.
    """
    stereo = {}
    for ob in (raw[2] if raw else ()):
        if ob['box'] is not None:
            stereo.setdefault(ob['box'], ob)
    if fused is None:
        rows = [(ob, 'stereo') for ob in (raw[2] if raw else ())]
        frame = raw[1] if raw else ''
    else:
        # labels belong to the fused array of the same stamp; another frame's, even with as many boxes, would mislabel
        same = sources and abs(sources[0] - fused[0]) < 1e-6 and len(sources[1]) == len(fused[2])
        labels = sources[1] if same else []
        rows = [(ob, labels[i] if labels else 'fused') for i, ob in enumerate(fused[2])]
        frame = fused[1]
    out = []
    for ob, source in rows:
        s = stereo.get(ob['box'])
        out.append({'cls': ob['cls'], 'box': ob['box'], 'source': source, 'frame': frame,
                    'range': _range(ob['pos']), 'stereo': _range(s['pos']) if s else None,
                    'sigma': _sigma(ob['pos'], ob['cov']), 'pos': ob['pos'],
                    'stereo_pos': s['pos'] if s else None})
    return out


def node_feed(ros, camera=None, cfg=None):
    """A FusionFeed on RosIO ``ros``: its node subscribes (/tf_static latched, so a late tab still gets it),
    the session's /points tap feeds it.

    None without the mhseals_nav messages."""
    from rclpy.qos import (DurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from mhseals_testing.ros.minimap_feeds import perception_msgs
    msgs = perception_msgs()
    if msgs is None:
        return None
    node = ros.node
    lossy = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    kinds = {'observations': (msgs.ObservationArray, lossy), 'sources': (String, 10), 'tf_static': (TFMessage, latched),
             'camera_info': (CameraInfo, qos_profile_sensor_data)}

    def subscribe(kind, topic, callback):
        message, qos = kinds[kind]
        return node.create_subscription(message, topic, callback, qos)

    feed = FusionFeed(subscribe, node.destroy_subscription, camera, cfg)

    def points(message):        # executor thread: a raise here would end the only spin
        if feed.points_due():
            try:
                feed.on_points(deserialize_message(message, PointCloud2) if isinstance(message, bytes) else message)
            except Exception:
                pass
    ros.taps['points'] = points
    return feed
