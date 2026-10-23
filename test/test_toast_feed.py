"""The health feed turns engine transitions into toasts: grace summary, no info, modal, x."""

import io
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from rich.console import Console

from mhseals_testing.health.engine import Transition
from mhseals_testing.health.model import Result
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.feeds.toasts import ToastFeed
from mhseals_testing.ui.toasts import Toasts

from test_app import make_app
from test_dashboard import monitor, view


class FakeHealth:
    """What ToastFeed needs of Health: drain(), specs and engine.last."""

    def __init__(self, **titles):
        self.specs = [SimpleNamespace(id=i, title=t) for i, t in titles.items()]
        self.engine = SimpleNamespace(last={}, muted=set())
        self.queue = []

    def move(self, id, old, new, detail='', now=0.0):
        self.queue.append(Transition(id, old, new, Result(new, detail=detail), now))
        self.engine.last[id] = Result(new)

    def drain(self):
        taken, self.queue = self.queue, []
        return taken


@pytest.fixture
def rig(tmp_path):
    app, _ = make_app(tmp_path)
    health = FakeHealth(gps='GPS', imu='IMU', cpu='CPU')
    feed = ToastFeed(health, grace=10.0)
    app.feeds = [feed]
    app.toasts = Toasts()

    def at(now):
        app.clock.now = now
        feed.tick(app, None, now)
        return app.view()['toasts']
    at(0.0)
    at(11.0)                                    # past the grace period with nothing wrong
    return app, health, at


def test_warn_to_fail_raises_one_toast(rig):
    app, health, at = rig
    health.move('gps', 'ok', 'warn', 'low')
    assert [t.kind for t in at(12.0)] == ['warn']
    health.move('gps', 'warn', 'fail', 'lost')
    shown = at(13.0)
    assert [(t.kind, t.text) for t in shown] == [('fail', 'GPS: lost')]


def test_info_to_ok_does_not_toast(rig):
    app, health, at = rig
    health.move('imu', 'ok', 'info')
    health.move('imu', 'info', 'ok')
    assert at(12.0) == []


def test_at_most_three_show(rig):
    app, health, at = rig
    for id in ('gps', 'imu', 'cpu'):
        health.move(id, 'ok', 'fail')
    health.move('gps', 'fail', 'ok')
    health.move('gps', 'ok', 'fail')
    assert len(at(12.0)) == 3


def test_a_modal_suppresses_toasts_and_shows_them_after(rig):
    app, health, at = rig
    app.modal = dict(kind='message', title='Hi', rows=[])
    health.move('gps', 'ok', 'fail', 'lost')
    health.move('imu', 'ok', 'fail', 'stuck')
    assert at(12.0) == []
    app.modal = None
    shown = at(13.0)                            # standing failures stay, plus one summary of the backlog
    assert [t.id for t in shown] == ['gps', 'imu', 'summary'] and '2 new' in shown[-1].text


def test_grace_period_then_one_summary_of_what_is_still_bad(tmp_path):
    app, _ = make_app(tmp_path)
    health = FakeHealth(gps='GPS', imu='IMU', cpu='CPU', gps2='GPS2')
    feed = ToastFeed(health, grace=10.0)
    feed.tick(app, None, 0.0)
    health.move('gps', 'ok', 'fail')
    health.move('imu', 'ok', 'warn')
    health.move('cpu', 'ok', 'warn')
    health.move('cpu', 'warn', 'ok')
    health.move('gps2', 'ok', 'fail')
    health.engine.muted.add('gps2')                    # muted checks stay out of the counts
    feed.tick(app, None, 5.0)
    assert app.toasts.visible(5.0) == []
    feed.tick(app, None, 10.0)
    (summary,) = app.toasts.visible(10.0)
    assert summary.text == f'Health: 1 fail{theme.SEPARATOR}1 warn{theme.SEPARATOR}! details'
    health.move('cpu', 'ok', 'fail')
    feed.tick(app, None, 11.0)
    assert len(app.toasts.visible(11.0)) == 2          # the summary and the new failure


def test_x_acks_the_newest_toast_before_exporting(rig):
    app, health, at = rig
    health.move('gps', 'ok', 'fail')
    health.move('imu', 'ok', 'fail')
    at(12.0)
    app.dispatch('x')
    assert [t.id for t in app.toasts.visible(12.0)] == ['gps']


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_toast_golden_120():
    toasts = Toasts()
    toasts.push('gps', 'fail', 'GPS: no fix for 12 s', 0.0)
    toasts.push('cpu', 'warn', 'CPU: 91% busy', 0.0)
    console = Console(width=120, height=40, file=io.StringIO(), force_terminal=False)
    console.print(layout.render(monitor().snapshot(),
                                view(load_profile('boat'), toasts=toasts.visible(0.0), now=0.0), 120, 40))
    golden = Path(__file__).parent / 'golden' / 'health' / 'toast_120.txt'
    if os.environ.get('UPDATE_GOLDEN'):
        golden.parent.mkdir(exist_ok=True)
        golden.write_text(console.file.getvalue())
    assert console.file.getvalue() == golden.read_text(), 'UPDATE_GOLDEN=1 to regenerate'


def test_app_ack_toasts_clears_standing_toasts(tmp_path):
    app, _ = make_app(tmp_path)
    app.toasts.push('gps', 'fail', 'GPS: lost', 0.0)
    app.toasts.push('imu', 'fail', 'IMU: stuck', 0.0)
    app.ack_toasts('gps')
    assert [t.id for t in app.toasts.visible(0.0)] == ['imu']
    app.ack_toasts()
    assert app.toasts.visible(0.0) == []
