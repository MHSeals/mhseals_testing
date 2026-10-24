"""The camera view subscribes only while open and keeps to 1, 2 or 4 frames per second."""

from types import SimpleNamespace

import pytest

from mhseals_testing.ros.camera_feed import PREVIEW_TOPIC, RATES_HZ, CameraFeed


class Bus:
    def __init__(self):
        self.live = {}
        self.t = 0.0

    def subscribe(self, topic, callback):
        handle = object()
        self.live[handle] = (topic, callback)
        return handle

    def unsubscribe(self, handle):
        del self.live[handle]

    def feed(self, hz=1):
        return CameraFeed(self.subscribe, self.unsubscribe, hz=hz, clock=lambda: self.t)

    def publish(self, data=b'jpg'):
        for _, callback in list(self.live.values()):
            callback(SimpleNamespace(data=data))


def test_no_subscription_until_opened_and_none_after_close():
    bus = Bus()
    feed = bus.feed()
    assert not bus.live and not feed.is_open
    feed.open()
    feed.open()
    assert [topic for topic, _ in bus.live.values()] == [PREVIEW_TOPIC]
    feed.close()
    assert not bus.live and not feed.is_open
    feed.close()


def test_close_drops_the_frame_and_late_callbacks():
    bus = Bus()
    feed = bus.feed()
    feed.open()
    callback = next(iter(bus.live.values()))[1]
    bus.publish()
    feed.close()
    assert feed.latest() is None
    callback(SimpleNamespace(data=b'late'))
    assert feed.latest() is None


@pytest.mark.parametrize('hz', RATES_HZ)
def test_accepts_at_most_hz_frames_per_second(hz):
    bus = Bus()
    feed = bus.feed(hz)
    feed.open()
    accepted = 0
    for _ in range(100):  # 30 Hz source for ~3.3 s
        before = feed.latest()
        bus.publish()
        accepted += feed.latest() is not before
        bus.t += 1 / 30
    assert accepted == pytest.approx(hz * 100 / 30, abs=1.01)


def test_rate_cycles_and_rejects_other_values():
    feed = Bus().feed()
    assert [feed.next_rate() for _ in range(4)] == [2, 4, 1, 2]
    with pytest.raises(ValueError):
        feed.set_rate(3)
    with pytest.raises(ValueError):
        Bus().feed(hz=8)


def test_raw_image_becomes_jpeg_and_profile_picks_the_sim_topic():
    from io import BytesIO

    from PIL import Image

    from mhseals_testing.profiles import SensorSpec
    from mhseals_testing.ros import camera_feed
    msg = SimpleNamespace(encoding='bgr8', width=4, height=2, step=12, data=bytes([0, 0, 255] * 8))
    assert Image.open(BytesIO(camera_feed.raw_to_jpeg(msg))).size == (4, 2)

    spec = SensorSpec('camera', '/front_camera/rgb/image', 'image')
    assert camera_feed.camera_source([spec]) == ('/front_camera/rgb/image', True)
    assert camera_feed.camera_source([SensorSpec('c', '/x/compressed', 'compressed')]) == ('/x/compressed', False)
    assert camera_feed.camera_source([]) == (PREVIEW_TOPIC, False)
