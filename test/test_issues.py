"""Issues: a device, part or transport problem shows (header glyph, row glyph, toast) only past its grace."""

from types import SimpleNamespace

from mhseals_testing.harness.components import Stack
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.feeds.issues import IssuesFeed, collect
from mhseals_testing.ui.toasts import Toasts
from test_components import FakeProcesses, FakeUnits, Profile, items

RUNNING = {'state': 'running', 'how': 'unit', 'running': True}
ZED = ['/front/zed_node']


class App:
    def __init__(self):
        self.stack = Stack(items(Profile), FakeProcesses(), lambda: 0.0, FakeUnits())
        self.nodes, self.events, self.toasts = list(ZED), [], Toasts()
        self.ros = SimpleNamespace(nodes=lambda: self.nodes,
                                   emit=lambda phase, **k: self.events.append((phase, k)))
        self.stack.units.records['camera'] = dict(RUNNING)


def tick(feed, app, now):
    feed.tick(app, {}, now)
    return [(toast.kind, toast.text) for toast in app.toasts.visible(now)]


def test_an_ssh_blip_under_the_grace_shows_nothing():
    app, feed = App(), IssuesFeed(grace_s=15, discovery_grace_s=20)
    units = app.stack.units
    units.errors['camera'] = 'ssh: timeout'
    units.failing['blastoise-jetson'] = ('ssh: timeout', 0.0, 0.0)
    for now in (0.0, 5.0, 14.0):
        assert tick(feed, app, now) == [] and feed.view() == {'issues': {}}
    units.errors.clear()
    units.failing.clear()                                  # answered again before 15 s
    assert tick(feed, app, 16.0) == [] and app.events == []
    assert app.stack.state('camera', []) == ('running', 'unit on blastoise-jetson')


def test_a_persistent_failure_shows_the_glyph_and_a_toast_then_clears():
    app, feed = App(), IssuesFeed(grace_s=15, discovery_grace_s=20)
    units = app.stack.units
    units.errors['camera'] = 'ssh: timeout'
    units.failing['blastoise-jetson'] = ('ssh: timeout', 0.0, 0.0)
    units.now = 15.0
    tick(feed, app, 0.0)
    toasts = tick(feed, app, 15.0)
    assert toasts == [('warn', 'blastoise-jetson does not answer (ssh: timeout); its parts show '
                               'its last report (from 15 s ago)')]
    assert list(feed.view()['issues']) == ['device:blastoise-jetson']
    assert app.events[0][0] == 'issue'
    # The row keeps its state and gains the glyph; the header leads with the count.
    assert app.stack.state('camera', []) == ('running', 'unit on blastoise-jetson')
    assert feed.on_row(app.stack.items['camera'])
    chip = layout.issue_chip(feed.view()['issues']).plain
    assert chip == f"{theme.PALETTE.glyph('issue')} 1 issue"
    units.errors.clear()
    units.failing.clear()
    toasts = tick(feed, app, 30.0)
    assert ('ok', 'resolved: blastoise-jetson does not answer (ssh: timeout); its parts show its '
                  'last report (from 15 s ago)') in toasts
    assert feed.view() == {'issues': {}} and app.events[-1][0] == 'issue_resolved'


def test_a_discovery_dropout_is_a_transport_issue_only_past_its_grace():
    app, feed = App(), IssuesFeed(grace_s=15, discovery_grace_s=20)
    app.nodes = []                                         # the camera's node left the graph
    assert tick(feed, app, 0.0) == []
    assert tick(feed, app, 5.0) == []
    app.nodes = list(ZED)                                  # back after 5 s: nothing was shown
    assert tick(feed, app, 6.0) == [] and app.stack.state('camera', [])[0] == 'running'
    app.nodes = []
    tick(feed, app, 10.0)
    assert tick(feed, app, 30.0) == [('warn', 'Camera runs on blastoise-jetson, but zed_node is '
                                              'not discovered here (DDS)')]
    assert app.stack.state('camera', app.nodes)[0] == 'running'


def test_an_exited_part_is_an_issue_and_ascii_has_its_own_glyph():
    app = App()
    app.stack.units.records['camera'] = {'state': 'exited', 'code': 1, 'log': '/x.log'}
    assert collect(app.stack, ZED)['part:camera'][1] == 'part'
    assert theme.PALETTE.glyph('issue', ascii=True) == '(!)'
    assert theme.PALETTE.glyph('issue', ascii=False) != theme.PALETTE.glyph('warn', ascii=False)
