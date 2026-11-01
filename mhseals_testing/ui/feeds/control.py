"""Control feed: the control tracker (harness/control.py) for this client, owner and viewer alike.

It re-reads the session registry when the file changes (checked at most every POLL_S) and recomputes the
tracker on every tick from that entry and the merged ``authority_record``, so every client shows the same
controller within a tick of a change. A change of controller or viewer count journals ``control``
(the owner only, so the session journal has one line per change). ``view()['control_tracker']``.
"""

import os
import time

from mhseals_testing.harness import control
from mhseals_testing.harness.registry import alive
from mhseals_testing.ui import feeds as feeds_module

POLL_S = 0.5


class ControlFeed:
    def __init__(self, registry, me, *, alive=alive, clock=time.time, emit=None):
        self.registry, self.me, self.alive, self.clock, self.emit = registry, me, alive, clock, emit
        self.entry, self.stamp, self.polled = {}, None, None
        self.value, self.key = None, None

    def poll(self, now):
        if self.polled is not None and now - self.polled < POLL_S:
            return
        self.polled = now
        try:
            stat = os.stat(self.registry.path)
            stamp = (stat.st_mtime_ns, stat.st_size, stat.st_ino)
        except OSError:
            stamp = None
        if stamp != self.stamp or stamp is None:
            self.stamp, self.entry = stamp, self.registry.state()

    def tick(self, app, snapshot, now):
        now = self.clock()
        self.poll(now)
        record = feeds_module.merged([f for f in app.feeds if f is not self]).get('authority_record')
        self.update(record, now)

    def update(self, record, now):
        self.value = control.tracker(self.entry, record, self.me, self.alive, now)
        ctl = self.value['controller']
        key = (ctl['kind'], ctl.get('pid'), ctl.get('claim'), self.value['viewers'])
        if key != self.key:
            if self.key is not None and self.emit is not None and self.value['role'] == 'control':
                self.emit('control', summary=control.summary(self.value), controller=ctl,
                          viewers=self.value['viewers'])
            self.key = key
        return self.value

    def view(self):
        return {'control_tracker': self.value} if self.value else {}


def start(app, registry, emit=None):
    """The tracker feed for a live client; None for a review or without a registry."""
    if registry is None or app.review or app.args.review is not None:
        return None
    feed = ControlFeed(registry, os.getpid(), emit=emit)
    app.feeds.append(feed)
    return feed
