"""Camera view frames: a throttled preview subscription that exists only while the view is open.

The preview is the compressed topic, never raw or 1080p, so the LAN carries a
small JPEG a few times a second. Opening creates a BEST_EFFORT depth-1
subscription; closing destroys it, so a closed view costs no traffic.
"""

import io
import threading
import time

PREVIEW_TOPIC = '/front_camera/preview/compressed'
RATES_HZ = (1, 2, 4)  # frame rates the view offers; cycling wraps around
RAW_MODES = {'rgb8': 'RGB', 'bgr8': 'RGB', 'mono8': 'L'}  # raw encodings the view can show


def raw_to_jpeg(message):
    """JPEG bytes of a raw sensor_msgs/Image (sim publishes only raw; Pillow loads on first frame)."""
    import numpy as np
    from PIL import Image
    if message.encoding not in RAW_MODES:
        raise ValueError(f'unsupported image encoding {message.encoding!r}')
    channels = 1 if message.encoding == 'mono8' else 3
    rows = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    array = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding == 'bgr8':
        array = array[:, :, ::-1]
    out = io.BytesIO()
    Image.fromarray(array[:, :, 0] if channels == 1 else np.ascontiguousarray(array),
                    RAW_MODES[message.encoding]).save(out, 'JPEG', quality=85)
    return out.getvalue()


class CameraFeed:
    """Holds the latest preview frame, accepting at most ``hz`` frames per second.

    ``subscribe(topic, callback)`` returns a handle and ``unsubscribe(handle)``
    destroys it; the ROS node supplies both, tests supply fakes. ``encode`` turns a
    message into JPEG bytes (raw topics); None takes ``message.data`` as the JPEG.
    """

    def __init__(self, subscribe, unsubscribe, topic=PREVIEW_TOPIC, hz=1, clock=time.monotonic,
                 encode=None):
        self._subscribe = subscribe
        self._unsubscribe = unsubscribe
        self.topic = topic
        self._clock = clock
        self._encode = encode
        self._lock = threading.Lock()
        self._handle = None
        self._frame = None
        self._last = None
        self.hz = self._valid(hz)

    @staticmethod
    def _valid(hz):
        if hz not in RATES_HZ:
            raise ValueError(f'camera rate must be one of {RATES_HZ} Hz, not {hz!r}')
        return hz

    @property
    def is_open(self):
        return self._handle is not None

    def open(self):
        """Subscribe (idempotent)."""
        if self._handle is None:
            self._handle = self._subscribe(self.topic, self._on_message)

    def close(self):
        """Unsubscribe and drop the frame, so a reopened view never shows a stale image."""
        if self._handle is not None:
            self._unsubscribe(self._handle)
        with self._lock:
            self._handle = None
            self._frame = None
            self._last = None

    def set_rate(self, hz):
        self.hz = self._valid(hz)

    def next_rate(self):
        """Step 1 -> 2 -> 4 -> 1 Hz and return the new rate."""
        self.hz = RATES_HZ[(RATES_HZ.index(self.hz) + 1) % len(RATES_HZ)]
        return self.hz

    def now(self):
        """The feed clock, so a frame's age is measured on the clock that stamped it."""
        return self._clock()

    def latest(self):
        """The newest accepted frame (or None) as ``(data, received_at)``."""
        with self._lock:
            return self._frame

    def _on_message(self, message):
        now = self._clock()
        with self._lock:
            if self._handle is None:
                return  # a callback that raced the close
            if self._last is not None and now - self._last < 1.0 / self.hz:
                return
            self._last = now
            self._frame = (self._jpeg(message), now)

    def _jpeg(self, message):
        """The frame's JPEG bytes; an undecodable message becomes b'' so the view shows 'bad frame'.

        This runs in the executor thread, where a raise would end the only ROS spin.
        """
        try:
            return self._encode(message) if self._encode else bytes(message.data)
        except Exception:
            return b''


def node_feed(node, raw=False, **kwargs):
    """A CameraFeed on ``node``: BEST_EFFORT depth-1, raw Image (``raw``) or CompressedImage."""
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
    qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    kind = Image if raw else CompressedImage
    return CameraFeed(lambda topic, callback: node.create_subscription(kind, topic, callback, qos),
                      node.destroy_subscription, encode=raw_to_jpeg if raw else None, **kwargs)


def camera_source(sensors):
    """``(topic, raw)`` for a profile's camera (kind image is raw, compressed is not), else the preview."""
    cameras = [s for s in sensors if s.kind in ('image', 'compressed')]
    camera = next((s for s in cameras if s.name == 'camera'), cameras[0] if cameras else None)
    return (PREVIEW_TOPIC, False) if camera is None else (camera.topic, camera.kind == 'image')


def profile_feed(node, sensors, **kwargs):
    """The feed for a profile's camera on ``node``."""
    topic, raw = camera_source(sensors)
    return node_feed(node, raw=raw, topic=topic, **kwargs)
