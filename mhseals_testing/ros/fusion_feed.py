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
from functools import partial
from pathlib import Path

import numpy as np

from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'fusion_panel.yaml'
KEYS = {'topics': {'observations', 'fused', 'sources', 'camera_info'},
        'feed': {'points_hz', 'points_max', 'points_link', 'points_link_framed', 'scan_dt_max_s', 'camera_hz', 'camera_width',
                 'max_range_m', 'residual_s', 'stale_s'},
        'ramp_m': None, 'rings_m': None, 'draw_hz': None,
        'check': {'near_m', 'du_pass_px', 'du_warn_px', 'dv_pass_px', 'dv_warn_px', 'hit_rate_min',
                  'range_tol', 'range_fail_tol', 'min_samples'}}
TF_STATIC = '/tf_static'   # its own latched subscription: the session's got the transforms before the tab opened
LIDAR_SOURCES = ('lidar', 'split', 'lidar_ray')   # fused rows ranged by the lidar (a hit)
SCANS = 3            # projected scans held, so boxes pair with the scan nearest their capture
RAW_S = 0.5          # stereo arrays held (s of stamps), so a fused array pairs with the stereo array of its stamp
STAMP_EPS = 1e-3     # s: a box array of the camera frame's own stamp (the backend ran on that image)
ASPECT_TOL = 0.02    # a frame whose aspect differs from camera_info's by more is not drawn (another crop)


def load_config(path=CONFIG):
    """fusion_panel.yaml, every key required and none unknown."""
    doc = load_strict(Path(path).read_text(), path)
    if set(doc) != set(KEYS):
        raise ValueError(f'{path}: keys {sorted(doc)} != {sorted(KEYS)}')
    for group, keys in KEYS.items():
        if keys is not None and set(doc[group]) != keys:
            raise ValueError(f'{path}: {group} keys {sorted(doc[group])} != {sorted(keys)}')
    return doc


def sample_rule(path=None):
    """lidar_fusion.yaml's sample filter (sample_classes, border_px, isolation_deg): the boxes it samples and
    fusion_calib grades. Installed share first, else the config beside the imported mhseals_nav source."""
    import yaml
    if path is None:
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(Path(get_package_share_directory('mhseals_nav')) / 'config')
        except Exception:
            pass
        import mhseals_nav
        candidates += [Path(d).parent / 'config' for d in mhseals_nav.__path__]
        path = next((c / 'lidar_fusion.yaml' for c in candidates if (c / 'lidar_fusion.yaml').is_file()), None)
        if path is None:
            raise FileNotFoundError(f'lidar_fusion.yaml not in {[str(c) for c in candidates]}')
    cfg = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    return {k: cfg[k] for k in ('sample_classes', 'border_px', 'isolation_deg')}


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

    def __init__(self, subscribe, unsubscribe, camera=None, cfg=None, clock=time.monotonic, rule=None):
        self.cfg = cfg or load_config()
        self.rule = rule or sample_rule()   # which near boxes lidar_fusion samples (and the check grades)
        self._subscribe, self._unsubscribe, self.camera, self._clock = subscribe, unsubscribe, camera, clock
        self._lock = threading.Lock()
        self.tf = StaticTF()          # kept while closed; /tf_static is redelivered on each open
        self._handles = {}
        self._camera_on = False
        self.tap = None               # tap(on): node_feed's /points tap, installed only while open
        self._reset()

    def _reset(self):
        self.K = self.size = None
        self.raw = self.fused = None          # (t, frame, [obs]) newest arrays
        self.raws = deque()                   # stereo arrays within RAW_S of the newest stamp, oldest first
        self.sources = None                   # (t, [label]) of the newest fused array
        self.skip = None                      # (received, skip cause) of the newest sources JSON
        self.scans = deque(maxlen=SCANS)      # (t, uv Nx2 normalised, range N, optical x/z Mx2 of all in front)
        self.residuals = deque()              # (received, du_px, dv_px)
        self.grades = deque()                 # (received, id, hit, L/S or None): near boxes of fused, unskipped frames
        self.fuseds = deque()                 # fused arrays within RAW_S, as raws, so a camera frame finds its own
        self.labelled = deque()               # (t, [label]) sources within RAW_S
        self.frame = None                     # (grey HxW, received, stamp, view at the frame's stamp or None)
        self.pending = None                   # (grey, received, stamp): a frame waiting for its boxes
        self._points_at = self._frame_at = None
        self._info_rank = 0         # of the camera_info topic K and size came from (want() resets it)
        self.version = 0

    @property
    def is_open(self):
        return bool(self._handles)

    def want(self, on, camera=False):
        """Subscribe while ``on`` (the tab shown); ``camera`` adds the frame (image tier, no link budget)."""
        if on and not self._handles:
            topics = self.cfg['topics']
            self._info_rank = len(topics['camera_info'])
            for kind, topic, callback in (('observations', topics['observations'], self._on_raw),
                                          ('observations', topics['fused'], self._on_fused),
                                          ('sources', topics['sources'], self._on_sources),
                                          *(('camera_info', topic, partial(self._on_camera_info, rank=rank))
                                            for rank, topic in enumerate(topics['camera_info'])),
                                          ('tf_static', TF_STATIC, self.on_tf_static)):
                self._handles[topic] = self._subscribe(kind, topic, callback)
            if self.tap:
                self.tap(True)
        elif not on and self._handles:
            for handle in self._handles.values():
                self._unsubscribe(handle)
            self._handles = {}
            if self.tap:
                self.tap(False)
            with self._lock:
                self._reset()
        camera = bool(on and camera and self.camera is not None)
        if camera != self._camera_on:
            self.camera.open() if camera else self.camera.close()
            self._camera_on = camera
            if not camera:
                with self._lock:
                    self.frame = self.pending = None
                    self._frame_at = None

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
            rng = np.hypot(p[:, 0], p[:, 2])          # horizontal rho, as the box labels and lidar_fusion
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

    def _on_camera_info(self, message, rank=0):
        """The config's order is the priority: the first-listed topic wins once it has published, a later one counts
        only until a better one has (the sim's relay beside the ZED's own info would otherwise alternate)."""
        try:
            with self._lock:
                if rank > self._info_rank:
                    return
                self._info_rank = rank
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
            self.raws.append(array)
            while self.raws and abs(self.raws[0][0] - array[0]) > RAW_S:     # also a stamp jump back (bag loop)
                self.raws.popleft()

    def _stereo_for(self, fused):
        """The stereo array a fused array came from (the same stamp; lidar_fusion publishes it a sweep later, when
        newer stereo arrays have arrived), the newest one without a fused array, else None."""
        if fused is None:
            return self.raw
        return next((a for a in reversed(self.raws) if abs(a[0] - fused[0]) < 1e-6), None)

    def _on_fused(self, message):
        try:
            array = self._array(message)
        except Exception:
            return
        with self._lock:
            self.fused, self.version = array, self.version + 1
            self.fuseds.append(array)
            while self.fuseds and abs(self.fuseds[0][0] - array[0]) > RAW_S:
                self.fuseds.popleft()

    def _on_sources(self, message):
        """Source labels per fused box, and the extrinsic samples as pixel residuals.

        A sample is lidar_fusion's ``(x, y, z, b, r_az, r_el, ...)``: the box centre's normalised image coordinates
        and the cluster's angular offset from it, so the pixel offset (fusion_calib's du/dv, cluster centre minus
        box centre) is ``fx (tan(atan x + r_az) - x)``, not ``r_az fx``, which shrinks off-centre boxes.
        """
        try:
            doc = json.loads(message.data)
            labels = [str(s) for s in doc.get('sources', [])]
            skipped = bool(doc.get('skip'))
            samples = [(float(e[0]), float(e[1]), float(e[4]), float(e[5])) for e in doc.get('ex') or []]
        except Exception:
            return
        now = self._clock()
        with self._lock:
            self.sources = (float(doc.get('t', 0.0)), labels)
            self.labelled.append(self.sources)
            while self.labelled and abs(self.labelled[0][0] - self.sources[0]) > RAW_S:
                self.labelled.popleft()
            self.skip = (now, str(doc.get('skip') or ''))
            if self.K is not None:
                for x, y, r_az, r_el in samples:
                    self.residuals.append((now, self.K[0] * (math.tan(math.atan(x) + r_az) - x),
                                           self.K[4] * (math.tan(math.atan(y) + r_el) - y)))
            if not skipped:
                self._grade(now)
            for held in (self.residuals, self.grades):
                while held and held[0][0] < now - self.cfg['feed']['residual_s']:
                    held.popleft()

    def _grade(self, now):
        """Hits and L/S of this fused frame's near boxes that lidar_fusion would sample (extrinsic_samples):
        stereo < near_m, a sample class, border_px clear of the image edge, no other box within isolation_deg in u.

        Only frames the fusion processed (no ``skip``) and labelled; ``held`` rows are neither a hit nor a miss."""
        if self.fused is None or not self.sources or abs(self.sources[0] - self.fused[0]) > 1e-6 or self.K is None:
            return
        rule, (W, H) = self.rule, self.size
        dets = [d for d in detections(self._stereo_for(self.fused), self.fused, self.sources) if d['box'] is not None]
        px = [(d['box'][0] * W, d['box'][1] * H, d['box'][2] * W, d['box'][3] * H) for d in dets]
        iso, edge = self.K[0] * math.tan(math.radians(rule['isolation_deg'])), rule['border_px']
        for i, det in enumerate(dets):
            u0, v0, u1, v1 = px[i]
            if det['stereo'] is None or det['stereo'] >= self.cfg['check']['near_m'] \
                    or det['source'] in ('held', 'fused') or not det['cls'].endswith(tuple(rule['sample_classes'])) \
                    or u0 < edge or v0 < edge or u1 > W - edge or v1 > H - edge \
                    or any(j != i and min(o[2], u1 + iso) - max(o[0], u0 - iso) > 0 for j, o in enumerate(px)):
                continue
            hit = det['source'] in LIDAR_SOURCES
            ratio = det['range'] / det['stereo'] if hit and det['range'] else None
            self.grades.append((now, det['id'], hit, ratio))

    # -- the panel's view ------------------------------------------------------------------------

    applied = (0.0, 0.0, 0.0)     # deg, optical x y z: set from the fusion status each frame (facts)

    def applied_deg(self):
        return self.applied

    def snapshot(self):
        """A plain dict for ui/panels/fusion.py (see its ``model`` docstring)."""
        now = self._clock()
        stale = self.cfg['feed']['stale_s']
        if self._camera_on and self.camera is not None:
            latest = self.camera.stamped()
            if latest is not None and latest[1] != self._frame_at and self.pending is None:
                self._frame_at = latest[1]
                grey = grey_frame(latest[0], self.cfg['feed']['camera_width'])
                if grey is not None:
                    self.pending = (grey, latest[1], latest[2])
        with self._lock:
            if self.pending is not None:
                self._settle(now)
            fresh = self._points_at is not None and now - self._points_at < stale + 1.0 / self.cfg['feed']['points_hz']
            dets = detections(self._stereo_for(self.fused), self.fused, self.sources)
            t = (self.fused or self.raw or (None,))[0]
            scan = min(self.scans, key=lambda s: abs(s[0] - t)) if self.scans and t is not None else (
                self.scans[-1] if self.scans else None)
            # lidar_fusion skipped its newest frame for motion (the boat turning): the unmotion-compensated scan is off
            motion = self.skip is not None and self.skip[1] == 'motion' and now - self.skip[0] < stale
            return {'version': self.version, 'size': self.size, 'K': self.K, 'dets': dets,
                    'fused': self.fused is not None, 'points': scan if fresh else None,
                    'scan_dt': scan[0] - t if fresh and scan is not None and t is not None else None, 'motion': motion,
                    'frame': None if self.frame is None else self.frame[0],
                    'at_frame': None if self.frame is None else self.frame[3],
                    'frame_aspect': self._aspect_ok(),
                    'residuals': [(t - now, du, dv) for t, du, dv in self.residuals],
                    'grades': [(t - now, i, hit, ratio) for t, i, hit, ratio in self.grades],
                    'applied': self.applied}


    def _settle(self, now):
        """Show the pending frame once the boxes of its own stamp have come (the fusion publishes a sweep later),
        or after RAW_S with the array nearest its stamp: the Camera tile then draws the frame with the boxes and
        scan of its stamp, so what is under the points is what the camera saw when they were measured."""
        grey, received, t = self.pending
        if t is None:
            self.frame, self.pending, self.version = (grey, received, None, None), None, self.version + 1
            return
        arrays = self.fuseds if self.fused is not None else self.raws
        exact = next((a for a in arrays if abs(a[0] - t) < STAMP_EPS), None)
        if exact is None and now - received < RAW_S:
            return
        array = exact or min(arrays, key=lambda a: abs(a[0] - t), default=None)
        view = None
        if array is not None:
            if self.fused is not None:
                labels = next((s for s in self.labelled if abs(s[0] - array[0]) < 1e-6), None)
                dets = detections(self._stereo_for(array), array, labels)
            else:
                dets = detections(array, None, None)
            scan = min(self.scans, key=lambda s: abs(s[0] - array[0]), default=None)
            view = {'dets': dets, 'points': scan, 'scan_dt': None if scan is None else scan[0] - array[0],
                    'frame_dt': array[0] - t}
        self.frame, self.pending, self.version = (grey, received, t, view), None, self.version + 1

    def _aspect_ok(self):
        """Whether the frame has camera_info's aspect (the points are normalised by its size), or None."""
        if self.frame is None or self.size is None:
            return None
        h, w = self.frame[0].shape[:2]
        return abs((w / h) / (self.size[0] / self.size[1]) - 1.0) <= ASPECT_TOL


def _flat(pos, frame):
    """``pos`` with its vertical axis zeroed (optical y down, body z up): lidar_fusion ranges the horizontal rho."""
    x, y, z = pos
    return (x, 0.0, z) if 'optical' in frame else (x, y, 0.0)


def _range(pos, frame=''):
    r = math.sqrt(sum(v * v for v in _flat(pos, frame)))
    return r if math.isfinite(r) and r > 0 else None


def _sigma(pos, cov, frame=''):
    """Horizontal range SD from a 3x3 covariance (lidar_fusion's radial one); None when the backend gave none."""
    if not any(cov):
        return None
    r = _range(pos, frame)
    if r is None:
        return None
    u = np.asarray(_flat(pos, frame)) / r
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
        sframe = frame
    else:
        # labels belong to the fused array of the same stamp; another frame's, even with as many boxes, would mislabel
        same = sources and abs(sources[0] - fused[0]) < 1e-6 and len(sources[1]) == len(fused[2])
        labels = sources[1] if same else []
        rows = [(ob, labels[i] if labels else 'fused') for i, ob in enumerate(fused[2])]
        frame = fused[1]
        sframe = raw[1] if raw else frame
    out = []
    for ob, source in rows:
        s = stereo.get(ob['box'])
        out.append({'cls': ob['cls'], 'box': ob['box'], 'source': source, 'frame': frame,
                    'range': _range(ob['pos'], frame), 'stereo': _range(s['pos'], sframe) if s else None,
                    'sigma': _sigma(ob['pos'], ob['cov'], frame), 'pos': ob['pos'], 'id': ob['id'],
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
    feed.tap = lambda on: ros.taps.update(points=points) if on else ros.taps.pop('points', None)
    return feed
