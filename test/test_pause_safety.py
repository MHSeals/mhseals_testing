"""Resume requires live finite sensing, including throughout the neutral settle."""
import math
from types import SimpleNamespace

import pytest

from test_app import make_app
from mhseals_testing.harness.conditions import Conditions
from mhseals_testing.ros.authority import Arbiter
from mhseals_testing.runner import Aborted


@pytest.mark.parametrize('sample', [None, (-3, 0, 0, 0), (0, math.nan, 0, 0),
                                     (0, 0, math.inf, 0), (0, 0, 0, math.nan)])
def test_missing_stale_or_nonfinite_stillness_blocks_resume(sample):
    conditions = Conditions(clock=lambda: 0)
    if sample:
        t, e, n, rate = sample
        conditions.feed(t, (e, n), 0, rate, True, 0)
    assert conditions.moving()


def paused_app(tmp_path, monkeypatch, poll):
    app, boat = make_app(tmp_path)
    app.keys = app.reader
    app.arming.armed = True
    app.arbiter = Arbiter(lambda r: None, lambda *a, **k: None, clock=app.clock, tui_owner=True)
    app.arbiter.on_mux({'input': 'auto'})
    frames, outputs, feeds = [], [], []
    monkeypatch.setattr(app, 'refresh', lambda **kw: frames.append((app.clock(), app.notice)))
    monkeypatch.setattr(app.source, 'tick', lambda now: poll(app, now))
    app.feeds = [SimpleNamespace(tick=lambda *a: feeds.append(app.clock()), view=lambda: {})]
    app.test_pulses = [1600] * 4
    app.publish_pulses = lambda p: outputs.append((app.clock(), list(p)))
    original = boat.publish
    boat.publish = lambda cmd: (outputs.append((app.clock(), cmd)), original(cmd))
    # Receipt advances only via poll, not the key fake's automatic odometry refresh.
    def read(timeout):
        app.clock.now += timeout
        return None
    monkeypatch.setattr(app.keys, 'read', read)
    return app, boat, frames, outputs, feeds


@pytest.mark.parametrize('bad', ['missing', 'stale', 'nan', 'inf', 'yaw', 'turning', 'moving', 'authority_stale', 'authority_unknown', 'authority_missing'])
@pytest.mark.parametrize('loop', ['hold', 'supervise'])
def test_actual_app_pause_cannot_resume_without_fresh_finite_stillness(tmp_path, monkeypatch, bad, loop):
    def poll(app, now):
        if bad == 'authority_missing':
            app.arbiter = None
        elif bad == 'authority_stale':
            app.arbiter.on_mux({'input': 'auto'})
            app.arbiter.mux_at = now - 10
        elif bad == 'authority_unknown':
            app.arbiter.on_mux({'input': 'bogus'})
        else:
            app.arbiter.on_mux({'input': 'rc' if now < .5 else 'auto'})
        if bad == 'missing':
            app.monitor.sensors.pop('odometry', None)
        elif bad != 'stale':
            vx = math.nan if bad == 'nan' else math.inf if bad == 'inf' else .5 if bad == 'moving' else 0
            app.monitor.observe('odometry', {'vx': vx, 'vy': 0, 'wz': math.nan if bad == 'yaw' else .5 if bad == 'turning' else 0})
    app, boat, frames, outputs, feeds = paused_app(tmp_path, monkeypatch, poll)
    app.clock.now = 3  # the initial odometry is stale
    with pytest.raises(Aborted, match='override_timeout'):
        with app.observe('test') as runner:
            app.arbiter.on_mux({'input': 'rc'})
            runner.pause_limit, runner.settle = 1, .25
            getattr(runner, loop)(*((.2, 0, 0), 1) if loop == 'hold' else (1,))
    assert not outputs and frames and feeds
    assert all('PAUSED' in label for _, label in frames)
    assert boat.sent[-1] == 'neutral' and not app._runner_paused


@pytest.mark.parametrize('loop', ['hold', 'supervise'])
@pytest.mark.parametrize('interruption', ['override', 'stale', 'nan', 'moving'])
def test_actual_app_requires_full_settle_after_sensing_or_override_returns(tmp_path, monkeypatch, loop, interruption):
    def poll(app, now):
        override = now < .5 or (interruption == 'override' and 1 <= now < 1.5)
        app.arbiter.on_mux({'input': 'rc' if override else 'auto'})
        if interruption == 'stale' and 1 <= now < 1.5:
            app.monitor.sensors['odometry']['times'][-1] = -10
        else:
            vx = (math.nan if interruption == 'nan' else .5) if interruption in ('nan', 'moving') and 1 <= now < 1.5 else 0
            app.monitor.observe('odometry', {'vx': vx, 'vy': 0, 'wz': 0})
    app, boat, frames, outputs, feeds = paused_app(tmp_path, monkeypatch, poll)
    callbacks = []
    with app.observe('test') as runner:
        app.arbiter.on_mux({'input': 'rc'})
        runner.period, runner.settle = .25, .75
        runner.on_resume = lambda: callbacks.append(('correction', app.clock()))
        if loop == 'hold':
            start, end = runner.hold((.2, 0, 0), .5)
            assert start == 2.25 and end == 2.75
            assert callbacks == [('correction', 2.25)]
        else:
            runner.supervise(.5, on_pause=lambda: callbacks.append(('cancel', app.clock())),
                             on_resume=lambda: callbacks.append(('goal', app.clock())))
            assert callbacks == [('cancel', 0), ('goal', 2.25)]
    assert all(t >= 2.25 for t, _ in outputs)
    assert len(feeds) > 8 and len(frames) > 8 and not app._runner_paused


@pytest.mark.parametrize('loop', ['hold', 'supervise'])
@pytest.mark.parametrize('key', ['E', 'space', 'z', 'Q'])
def test_paused_app_routes_case_sensitive_controls_and_configured_abort(tmp_path, monkeypatch, loop, key):
    from mhseals_testing.ui import keymap
    shipped, _ = keymap.load(user=None)
    shipped.doc['runner']['abort'].append('z')
    monkeypatch.setattr(keymap, 'load', lambda: (shipped, None))
    def poll(app, now):
        app.arbiter.on_mux({'input': 'hold'})
        app.monitor.observe('odometry', {'vx': 0, 'vy': 0, 'wz': 0})
    app, boat, frames, outputs, _ = paused_app(tmp_path, monkeypatch, poll)
    app.arbiter.table.claim('agent', 'test', 0)
    revoked = []
    app.arbiter.neutral = boat.neutral
    app.arbiter.hold = lambda cause, by: (boat.hold(cause), revoked.append(by))
    keys = iter(['e', key, 'space'])
    def read(timeout):
        app.clock.now += timeout
        return next(keys, 'space')
    monkeypatch.setattr(app.keys, 'read', read)
    with pytest.raises(Aborted):
        with app.observe('test') as runner:
            app.arbiter.on_mux({'input': 'hold'})
            getattr(runner, loop)(*((.2, 0, 0), 1) if loop == 'hold' else (1,))
    assert not outputs and not app._runner_paused
    assert revoked == (['tui'] if key == 'E' else [])
    assert ('keys' in getattr(boat, 'holds', [])) == (key in ('E', 'space'))
    assert boat.sent[-1] == 'neutral'


@pytest.mark.parametrize('confirm', ['accept', 'cancel', 'space'])
def test_paused_u_confirmation_ticks_without_raw_output_and_needs_mux_ack(tmp_path, monkeypatch, confirm):
    releases = []
    def poll(app, now):
        app.arbiter.on_mux({'input': 'auto' if releases and now >= 1 else 'hold'})
        app.monitor.observe('odometry', {'vx': 0, 'vy': 0, 'wz': 0})
    app, boat, frames, outputs, _ = paused_app(tmp_path, monkeypatch, poll)
    boat.release_hold = lambda: releases.append(app.clock()) or True
    app.running = 'test'
    modal_keys = iter(['up', 'enter'] if confirm == 'accept' else ['enter'] if confirm == 'cancel' else ['space', 'escape'])
    delivered = []
    def read(timeout):
        app.clock.now += timeout
        if app.modal:
            assert app._runner_paused
            return next(modal_keys)
        if not delivered:
            delivered.append('U')
            return 'U'
        return 'space' if app.clock() > 2 else None
    monkeypatch.setattr(app.keys, 'read', read)
    with app.observe('test') as runner:
        app.arbiter.on_mux({'input': 'hold'})
        runner.settle = .25
        if confirm == 'accept':
            start, _ = runner.hold((.2, 0, 0), .1)
            assert start >= 1.25
        else:
            with pytest.raises(Aborted):
                runner.hold((.2, 0, 0), .1)
    assert len(releases) == (confirm == 'accept')
    assert all(t >= 1.25 or command == [1500] * 4 for t, command in outputs)
    assert not app._runner_paused and app.modal is None


def test_paused_callback_failure_leaves_neutral_and_clears_app_pause_flag(tmp_path, monkeypatch):
    def poll(app, now):
        raise RuntimeError('telemetry read failed')
    app, boat, _, outputs, _ = paused_app(tmp_path, monkeypatch, poll)
    with pytest.raises(RuntimeError, match='telemetry read failed'):
        with app.observe('test') as runner:
            app.arbiter.on_mux({'input': 'rc'})
            runner.hold((.2, 0, 0), 1)
    assert not outputs and boat.sent[-1] == 'neutral' and not app._runner_paused


def test_paused_runner_honors_configured_marker_passthrough(tmp_path, monkeypatch):
    from mhseals_testing.ui import keymap
    shipped, _ = keymap.load(user=None)
    shipped.doc['global']['mark'] = ['v']
    monkeypatch.setattr(keymap, 'load', lambda: (shipped, None))
    def poll(app, now):
        app.arbiter.on_mux({'input': 'hold'})
    app, boat, _, outputs, _ = paused_app(tmp_path, monkeypatch, poll)
    app.running = 'test'
    keys = iter(['v', 'space'])
    def read(timeout):
        app.clock.now += timeout
        return next(keys)
    monkeypatch.setattr(app.keys, 'read', read)
    with pytest.raises(Aborted):
        with app.observe('test') as runner:
            app.arbiter.on_mux({'input': 'hold'})
            runner.hold((.2, 0, 0), 1)
    assert 'marker' in app.ros.events and not outputs
