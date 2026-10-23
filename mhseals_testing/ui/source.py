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
