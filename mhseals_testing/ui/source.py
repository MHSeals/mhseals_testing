"""Where the dashboard's data comes from.

``snapshot`` is what ``layout.render`` consumes. ``live`` gates every App
path that touches the real host or boat (host reads, config watching, arming,
procedures); a replay source sets it False.
"""


class LiveSource:
    """Messages arrive through ``ros.ingest`` callbacks, so ``tick`` has nothing to do."""

    live = True

    def __init__(self, monitor, profile, meta=None):
        self.monitor = monitor
        self.profile = profile
        self.meta = meta or {}

    def snapshot(self):
        return self.monitor.snapshot(self.profile.required_tf)

    def tick(self, now):
        pass


class ReviewSource:
    """Isolated dashboard monitor fed by the existing segmented bag reader."""

    live = False

    def __init__(self, path, profile, review, **reader_args):
        from mhseals_testing.harness.bagread import BagSource
        self.profile, self.review = profile, review
        self.meta = {'host': review.host, 'title': review.title, 'profile': profile.name}
        self.now = review.t0
        self.kinds = {sensor.topic: sensor.kind for sensor in profile.sensors}
        self.kinds.update({'/tf': 'tf', '/tf_static': 'tf_static',
                           '/calibrated_velocity_controller/drift': 'drift',
                           '/mhseals_testing/events': 'event'})
        self.reader = BagSource(path, self.kinds, self.ingest, self.reset,
                                span=(int(review.t0 * 1e9), int(review.t1 * 1e9)), **reader_args)
        self.seek(review.t0)

    def reset(self):
        from mhseals_testing.monitor import BoatMonitor
        self.monitor = BoatMonitor(clock=lambda: self.now)

    def ingest(self, topic, message, stamp):
        import json
        from mhseals_testing.ros.ingest import ingest
        self.now = stamp
        if self.kinds[topic] == 'event' and hasattr(message, 'data'):
            message = json.loads(message.data)
        ingest(self.monitor, self.profile, topic, self.kinds[topic], message, stamp)

    def seek(self, seconds):
        self.reader.seek(int(seconds * 1e9))
        self.now = seconds

    def tick(self, now):
        if self.review.due is not None:
            return
        target = int(self.review.playhead * 1e9)
        if target > self.reader.t_ns:
            self.reader.play()
            self.reader.tick((target - self.reader.t_ns) / 1e9)
        self.now = self.review.playhead

    def snapshot(self):
        return self.monitor.snapshot(self.profile.required_tf)
