"""Camera view frames: a throttled preview subscription that exists only while the view is open.

The preview is the compressed topic, never raw or 1080p, so the LAN carries a
small JPEG a few times a second. Opening creates a BEST_EFFORT depth-1
subscription; closing destroys it, so a closed view costs no traffic.
"""

import threading
import time

PREVIEW_TOPIC = '/front_camera/preview/compressed'
RATES_HZ = (1, 2, 4)  # frame rates the view offers; cycling wraps around


class CameraFeed:
    """Holds the latest preview frame, accepting at most ``hz`` frames per second.

    ``subscribe(topic, callback)`` returns a handle and ``unsubscribe(handle)``
    destroys it; the ROS node supplies both, tests supply fakes.
    """

    def __init__(self, subscribe, unsubscribe, topic=PREVIEW_TOPIC, hz=1, clock=time.monotonic):
        self._subscribe = subscribe
        self._unsubscribe = unsubscribe
        self.topic = topic
        self._clock = clock
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
            self._frame = (bytes(message.data), now)


def node_feed(node, **kwargs):
    """A CameraFeed whose subscription is a BEST_EFFORT depth-1 CompressedImage on ``node``."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    return CameraFeed(lambda topic, callback: node.create_subscription(
        CompressedImage, topic, callback, qos_profile_sensor_data), node.destroy_subscription, **kwargs)
