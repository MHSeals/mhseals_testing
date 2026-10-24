"""Selected Bags dispatch uses real segmented replay and preserves live owners."""
from functools import partial
import json

from mhseals_testing.ui.source import ReviewSource
import pytest
from test_app import make_app
from test_bagread import FakeReader, S
from test_ingest import odometry


def selected_app(tmp_path, monkeypatch):
    app, boat = make_app(tmp_path)
    for name in ('other', 'selected'):
        path = tmp_path / 'sessions' / name
        path.mkdir(parents=True)
        (path / 'session.json').write_text(json.dumps(
            {'id': name, 'profile': 'boat', 'host': 'recorded-host', 'title': name}))
        (path / 'events.jsonl').write_text('\n'.join(json.dumps(
            {'phase': 'marker', 't_ns': t * S, 'id': t}) for t in (10, 40)))
        for segment in ('00', '01'):
            folder = path / 'bag' / segment
            folder.mkdir(parents=True)
            (folder / 'metadata.yaml').write_text('inert test reader')
    opens = []

    def opener(segment):
        opens.append(segment)
        rows = []
        for stamp in ((10, 15, 20) if segment.name == '00' else (25, 30, 40)):
            message = odometry()
            message.pose.pose.position.x = float(stamp)
            rows.append(('/odom/local', message, stamp * S))
        return FakeReader(rows), {'/odom/local': 'nav_msgs/msg/Odometry'}

    monkeypatch.setattr('mhseals_testing.ui.app.ReviewSource', partial(
        ReviewSource, opener=opener, decode=lambda kind, message: message))
    app.show_tab('Bags')
    app.cursors['list'] = next(i for i, b in enumerate(app.bags_view()[0]) if b.id == 'selected')
    return app, boat, opens


def settle(app):
    app.clock.now += .2
    app.tick()


def test_selected_enter_seek_rewind_escape_preserves_live_recording(tmp_path, monkeypatch):
    app, boat, opens = selected_app(tmp_path, monkeypatch)
    owners = (app.monitor, app.ros, app.processes, app.recorder, app.paths)
    old = (app.source, app.tabs, app.focus, app.cursors, app.minimap, app.layerbox)
    live_ticks = []
    # Exercise the real owner tick routing, but prohibit host/device sampling.
    monkeypatch.setattr(app, 'refresh_advisories', lambda: live_ticks.append(app.source))
    monkeypatch.setattr(app, 'watch_config', lambda: None)
    app.feeds = []
    app.dispatch('enter')
    assert app.review.title == 'selected' and app.review.host == 'recorded-host'
    assert app.tab == 'Drive' and app.focus == 'activity' and not app.source.live
    assert app.source.reader.path == tmp_path / 'sessions/selected'
    assert app.view()['host'] == 'recorded-host'
    assert app.view()['profile'] == 'boat'
    from test_review_app import _text
    assert 'REVIEW' in _text(app, 120, 40)
    assert app.snapshot()['sensors']['odometry']['values']['x'] == 10.
    boat.feed((.1, 0., 0.))  # Live callbacks keep writing the original monitor.
    assert app.snapshot()['sensors']['odometry']['values']['x'] == 10.
    assert all(path.parent.parent.name == 'selected' for path in opens)
    for _ in range(3):
        app.dispatch('right')
    settle(app)
    assert app.review.playhead == 25
    assert app.snapshot()['sensors']['odometry']['values']['x'] == 25.
    assert {p.name for p in opens} == {'00', '01'}
    app.dispatch('left')
    settle(app)
    assert app.snapshot()['sensors']['odometry']['values']['x'] == 20.
    assert app.source.monitor.sensors['odometry']['count'] == 3  # rewind rebuilt10..20
    app.dispatch('p')
    app.clock.now += 5
    app.tick()
    app.dispatch('p')
    assert app.review.playhead == 25
    assert app.snapshot()['sensors']['odometry']['values']['x'] == 25.
    assert live_ticks and all(source is old[0] for source in live_ticks)
    assert (app.monitor, app.ros, app.processes, app.recorder, app.paths) == owners
    assert app.processes.running('bag') and boat.sent == [] and app.pulse_log == []
    app.dispatch('escape')
    assert not app.review and app._review_live is None
    assert (app.source, app.tabs, app.focus, app.cursors, app.minimap, app.layerbox) == old
    assert app.tab == 'Bags' and app.selected_bag().id == 'selected'


def test_review_global_controls_cannot_actuate_but_space_keeps_hold(tmp_path, monkeypatch):
    app, boat, _ = selected_app(tmp_path, monkeypatch)
    app.dispatch('enter')
    for method in ('release_hold', 'open', 'revoke_agent', 'run_procedure'):
        monkeypatch.setattr(app, method, lambda *a: pytest.fail('live control reached in review'))
    for key in ('U', 'H', 'E', 'a', 'f', 'm', 'b', 'B', 'x', 'F'):
        app.dispatch(key)
    app.focus = 'status'
    app.dispatch('enter')
    assert not app.armed and app.arming.calls == [] and boat.sent == []
    app.review.playing = True
    app.dispatch('space')
    assert not app.review.playing and boat.holds == ['keys'] and 'neutral' in boat.sent
    assert app.source.live is False


@pytest.mark.parametrize('state', ['armed', 'manual', 'running', 'pulses'])
def test_busy_entry_refuses_without_loading_or_changing_owners(tmp_path, monkeypatch, state):
    app, boat, opens = selected_app(tmp_path, monkeypatch)
    if state == 'armed':
        app.arming.armed = True
    else:
        field = {'manual': 'manual', 'running': 'running', 'pulses': 'test_pulses'}[state]
        setattr(app, field, True)
    source = app.source
    app.dispatch('enter')
    assert app.review is None and app.source is source and not opens
    assert 'stop motion' in app.notice and boat.sent == []


@pytest.mark.parametrize('invalid', ['{}\n', '{"t_ns": "bad"}\n'])
def test_invalid_selected_session_is_notice_with_no_output(tmp_path, monkeypatch, invalid):
    app, boat, opens = selected_app(tmp_path, monkeypatch)
    selected = app.selected_bag()
    (selected.path / 'events.jsonl').write_text(invalid)
    app.dispatch('enter')
    assert app.review is None and app.source.live and not opens
    assert ('no timed events' in app.notice or 'invalid session timeline' in app.notice)
    assert boat.sent == [] and app.pulse_log == []


def test_recording_or_finalizing_selected_session_cannot_open(tmp_path, monkeypatch):
    app, boat, opens = selected_app(tmp_path, monkeypatch)
    selected = app.selected_bag()
    app.finalizing = selected.path.resolve()
    app.dispatch('enter')
    assert 'being finalized' in app.notice and app.review is None and not opens
    app.finalizing = None
    app.paths = type(app.paths).from_output(selected.path)
    app.dispatch('enter')
    assert 'recording' in app.notice and app.review is None and not opens and boat.sent == []


def test_offline_selected_review_needs_no_ros_process_or_output(tmp_path, monkeypatch):
    import builtins
    from types import SimpleNamespace
    from mhseals_testing.ui.review import Offline
    app, _, _ = selected_app(tmp_path, monkeypatch)
    monitor = app.monitor
    app.source = SimpleNamespace(live=False, tick=lambda now: None,
                                 snapshot=lambda: monitor.snapshot(app.profile.required_tf))
    app.ros = app.processes = Offline()
    app.arming = app.plant = None
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name.split('.')[0] in ('rclpy', 'rosbag2_py'):
            pytest.fail('offline replay attempted ROS initialization')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guarded_import)
    monkeypatch.setattr(app, 'tick_live', lambda: pytest.fail('offline replay did live work'))
    # There is no live owner to maintain for this source.
    app.dispatch('enter')
    for key in ('right', 'left', 'a', 'f', 'm', 'U'):
        app.dispatch(key)
    settle(app)
    assert not app.source.live and not app.processes.running('bag')
    app.dispatch('escape')
    assert app.review is None and not app.source.live and app.pulse_log == []


def test_review_odometry_popup_is_read_only(tmp_path, monkeypatch):
    app, _, _ = selected_app(tmp_path, monkeypatch)
    app.dispatch('enter')
    shown = []
    monkeypatch.setattr(app, 'show', lambda *a, **kw: shown.append((a, kw)))
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: pytest.fail('live controls opened'))
    app.ros.get_parameters = lambda *a: pytest.fail('read live ROS parameters')
    app.odometry()
    assert shown[0][0] == ('Odometry (review)',)
