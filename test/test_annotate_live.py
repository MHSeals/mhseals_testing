"""Field annotation: the live feed that journals automatic segment edges."""

from types import SimpleNamespace

import pytest

from mhseals_testing.annotate.live import AnnotateFeed

pytestmark = pytest.mark.smoke   # tools/smoke

S = 1_000_000_000


class FakeApp:
    def __init__(self):
        self.lines, self.armed, self.manual, self.running = [], True, True, None
        self.command, self.feeds, self.rtk = (0.0, 0.0, 0.0), [], SimpleNamespace(current=None)

    def emit(self, phase, **details):
        self.lines.append((phase, details))


def test_feed_journals_fix_holds_owner_and_steady_windows():
    app, clock = FakeApp(), [0]
    feed = AnnotateFeed(wall=lambda: clock[0] * S)

    def tick(t, fix=6, holds=(), vx=0.5):
        clock[0] = t
        snapshot = {'sensors': {'gpsraw': {'status': 'LIVE', 'values': {'fix': fix}},
                                'odometry': {'values': {'vx': vx, 'vy': 0.0, 'wz': 0.0}}},
                    'control': {'holds': [{'cause': c} for c in holds]}}
        feed.tick(app, snapshot, t)

    tick(0)
    app.command = (0.4, 0.0, 0.0)
    for t in range(1, 8):
        tick(t, fix=5 if t >= 3 else 6)
    tick(8, holds=('link lost',))
    app.command = (0.0, 0.0, 0.0)
    tick(9)
    phases = [p for p, _ in app.lines]
    assert phases[:2] == ['gnss_fix', 'owner']
    assert ('gnss_fix', {'fix': 'FLOAT', 'prev': 'FIXED'}) in app.lines         # held 2 s before it counts
    assert ('hold_on', {'cause': 'link lost'}) in app.lines and ('hold_off', {'cause': 'link lost'}) in app.lines
    steady = [d for p, d in app.lines if p == 'steady']
    assert len(steady) == 1 and steady[0]['command'] == [0.4, 0.0, 0.0] and steady[0]['velocity'][0] == 0.5
    assert (steady[0]['t_ns'] - steady[0]['t0_ns']) / S == 8
