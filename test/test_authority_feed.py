"""The session owner runs the arbiter and keeps its latest record; guests and review run none."""

from types import SimpleNamespace

from mhseals_testing.ui.feeds import authority as feed_module

from test_app import make_app


class FakeBus:
    """What bind would install: collects what the arbiter publishes."""

    def __init__(self):
        self.published, self.bound = [], []

    def bind(self, ros, arbiter):
        self.bound.append(arbiter)
        arbiter.publish = self.published.append


def start(app, clock):
    bus = FakeBus()
    feed = feed_module.start(app, app.ros, bind=bus.bind)
    feed.clock = feed.arbiter.clock = lambda: clock[0]
    feed.arbiter.on_mux({'input': 'auto', 't_ns': int(clock[0] * 1e9)})
    return feed, bus


def test_procedure_start_gives_an_ours_record_and_an_authority_event(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, bus = start(app, clock)
    feed.arbiter.tick()
    assert bus.bound == [feed.arbiter] and feed.record['owner'] == 'none'
    app.running, app.run_id = 'Characterize', 'r1'
    clock[0] += 0.25
    feed.arbiter.tick()
    record = feed.view()['authority_record']
    assert record['owner'] == 'procedure' and 'Characterize' in record['source']
    assert bus.published[-1]['owner'] == 'procedure'      # sent on the bound transport too
    assert app.feeds[-1] is feed
    changes = [f for phase, f in app.ros.lines if phase == 'authority']
    assert changes and changes[-1]['to']['owner'] == 'procedure'


def test_a_stale_record_reads_unknown_and_paused(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    assert feed.view() == {'authority_record': None, 'authority_paused': True}   # nothing yet
    feed.arbiter.tick()
    assert feed.view()['authority_paused'] is False
    clock[0] += 10.0
    assert feed.view() == {'authority_record': None, 'authority_paused': True}


def test_guests_and_review_create_no_arbiter(tmp_path):
    app, _ = make_app(tmp_path)
    bus = FakeBus()
    app.watching = 4242
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    app.watching, app.review = 0, SimpleNamespace()
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    app.review, app.args.review = None, '/some/session'
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    assert bus.bound == []


def test_an_unclaimed_publisher_with_a_nonzero_cmd_vel_is_external(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    app.ros.endpoints = lambda topic: [{'node': '/some_agent', 'gid': 'aa', 'self': False}]
    feed.on_cmd(True)
    feed.arbiter.tick()
    assert feed.record['owner'] == 'external' and '/some_agent' in feed.record['source']
