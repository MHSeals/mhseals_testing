"""Session operations use scratch data and the real key dispatcher."""
from mhseals_testing import data
from mhseals_testing.harness import bags
from mhseals_testing.ui.app import SessionPaths
from test_app import make_app


def setup_app(tmp_path):
    app, boat = make_app(tmp_path)
    path = tmp_path / 'sessions' / 'live'
    path.mkdir(parents=True)
    data.write_json(path / 'session.json', {}, bags.SCHEMA)
    app.paths = SessionPaths.from_output(path)
    app.focus_on('list')
    app.tabs.current = 'Bags'
    return app, boat, path


def test_cache_tracks_writer_transitions_and_adoption(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    live = [False]
    monkeypatch.setattr(app.processes, 'running', lambda key: live[0])
    first = app.bags_view()[0]
    assert not first[0].recording
    assert app.bags_view()[0] is first
    live[0] = True
    assert app.bags_view()[0][0].recording
    assert app.recorder.segment is None  # an adopted writer still protects the whole session
    assert app.bag_protected(first[0])
    app.recorder.segment = path / 'bag' / '00'
    assert app.recording_paths() == (path.resolve(), app.recorder.segment.resolve())
    live[0] = False
    assert not app.bags_view()[0][0].recording
    assert not app.bag_protected(first[0])


def test_rows_refresh_and_clamp_cursor(tmp_path):
    app, _, path = setup_app(tmp_path)
    assert app.rows('list') == 1
    app.cursors['list'] = 5
    (path / 'session.json').unlink()
    path.rmdir()
    app.invalidate_bags()
    assert app.rows('list') == 0
    assert app.cursors['list'] == 0
