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
    app.processes.running = lambda key: False
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


def test_dispatch_pin_title_and_cancel_delete(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    app.dispatch('p')
    assert data.read_json(path / 'session.json')['pinned']
    assert data.read_json(path / 'summary.json')['pinned']
    assert data.read_json(tmp_path / 'INDEX.json')['sessions'][0]['pinned']
    app.dispatch('p')
    monkeypatch.setattr(app, 'ask_text', lambda *a: 'Wind trial')
    app.dispatch('n')
    assert data.read_json(path / 'session.json')['title'] == 'Wind trial'
    assert data.read_json(path / 'summary.json')['title'] == 'Wind trial'
    assert data.read_json(tmp_path / 'INDEX.json')['sessions'][0]['title'] == 'Wind trial'
    def cancel(*a, **kw):
        assert kw['selected'] == 1
        return 'Cancel'
    monkeypatch.setattr(app, 'choose', cancel)
    app.dispatch('d')
    assert path.exists()


def test_delete_rechecks_recording_and_pins_after_confirmation(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    live = [False]
    monkeypatch.setattr(app.processes, 'running', lambda key: live[0])
    def confirm(*a, **kw):
        live[0] = True
        return 'Delete'
    monkeypatch.setattr(app, 'choose', confirm)
    app.dispatch('d')
    assert path.exists() and 'recording' in app.notice
    live[0] = False
    def pin(*a, **kw):
        data.write_json(path / 'session.json', {'pinned': True}, bags.SCHEMA)
        return 'Delete'
    monkeypatch.setattr(app, 'choose', pin)
    app.dispatch('d')
    assert path.exists() and 'pinned' in app.notice


def test_delete_and_prune_respect_finalizer(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    app.finalizing = path.resolve()
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    app.dispatch('d')
    app.dispatch('P')
    assert path.exists()


def test_prune_mixed_legacy_and_partial_failure_rebuild_once(tmp_path, monkeypatch):
    import os
    from mhseals_testing.ui import app as app_module

    app, _, path = setup_app(tmp_path)
    data.write_json(path / 'session.json', {'created': 2}, bags.SCHEMA)
    header = (path / 'session.json').read_bytes()
    legacy = tmp_path / 'legacy'
    legacy.mkdir()
    (legacy / 'empty.mcap').touch()
    report = tmp_path / 'legacy.report.json'
    report.write_text('{}')
    unrelated = tmp_path / 'legacy.extra'
    unrelated.mkdir()
    os.utime(legacy, (1, 1))
    app.invalidate_bags()
    real_delete = bags.delete
    real_rebuild = app_module.index.rebuild
    attempts, rebuilds = [], []
    def delete(bag):
        attempts.append(bag.path)
        if not bag.legacy:
            raise OSError('fixture failure')
        real_delete(bag)
    def rebuild(root):
        rebuilds.append(root)
        return real_rebuild(root)
    monkeypatch.setattr(bags, 'delete', delete)
    monkeypatch.setattr(app_module.index, 'rebuild', rebuild)
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    app.dispatch('P')
    assert attempts == [legacy, path]
    assert rebuilds == [tmp_path]
    rows = data.read_json(tmp_path / 'INDEX.json')['sessions']
    assert [row['id'] for row in rows] == ['live']
    assert path.is_dir() and (path / 'session.json').read_bytes() == header
    assert not legacy.exists() and not report.exists() and unrelated.is_dir()
    assert 'fixture failure' in app.notice


def test_legacy_only_prune_rebuilds_actual_root(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    (path / 'session.json').unlink()
    path.rmdir()
    legacy = tmp_path / 'legacy'
    legacy.mkdir()
    (legacy / 'empty.mcap').touch()
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    from mhseals_testing.ui import app as app_module
    rebuilds = []
    monkeypatch.setattr(app_module.index, 'rebuild', lambda root: rebuilds.append(root))
    app.dispatch('P')
    assert rebuilds == [tmp_path] and not legacy.exists()


def test_delete_legacy_glob_characters_are_literal(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    (path / 'session.json').unlink()
    path.rmdir()
    selected = tmp_path / '[a]'
    selected.mkdir()
    (selected / 'data.mcap').write_bytes(b'bag')
    own = tmp_path / '[a].report.json'
    own.write_text('{}')
    unrelated = tmp_path / 'a.report.json'
    unrelated.write_bytes(b'unrelated report')
    app.invalidate_bags()
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    app.dispatch('d')
    assert not selected.exists() and not own.exists()
    assert unrelated.read_bytes() == b'unrelated report'


def test_partial_subtree_delete_rebuilds_index_once(tmp_path, monkeypatch):
    from mhseals_testing.ui import app as module
    for key in ('d', 'P'):
        root = tmp_path / key
        root.mkdir()
        monkeypatch.setenv('MHSEALS_DATA', str(root))
        app, _, path = setup_app(root)
        folder = path / 'bag' / '00'
        folder.mkdir(parents=True)
        mcap = folder / 'data.mcap'
        mcap.write_bytes(b'recorded data')
        original = module.index.rebuild
        original(root)
        before = data.read_json(root / 'INDEX.json')['sessions'][0]['bytes']
        rebuilds = []
        def rebuild(target):
            rebuilds.append(target)
            return original(target)
        def partial(bag):
            mcap.unlink()
            raise OSError('partial subtree failure')
        with monkeypatch.context() as patch:
            patch.setattr(module.index, 'rebuild', rebuild)
            patch.setattr(bags, 'delete', partial)
            patch.setattr(bags, 'budget_bytes', lambda: 0)
            patch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
            app.dispatch(key)
        after = data.read_json(root / 'INDEX.json')['sessions'][0]['bytes']
        assert before == len(b'recorded data') and after == 0
        assert rebuilds == [root] and path.exists() and not mcap.exists()
        assert (path / 'session.json').exists()


def segment(path, content=b''):
    folder = path / 'bag' / '00'
    folder.mkdir(parents=True)
    mcap = folder / 'open.mcap'
    mcap.write_bytes(content)
    return folder, mcap


def test_finalize_active_mcaps_untouched(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    folder, mcap = segment(path)
    app.bags_view()  # stale row before the writer starts
    monkeypatch.setattr(app.processes, 'running', lambda key: True)
    app.recorder.segment = folder
    calls = []
    app.quiet_run = lambda argv: calls.append(argv)
    monkeypatch.setattr(app, 'refresh_bag_index', lambda bag: calls.append('index'))
    for content in (b'', b'recorded bytes'):
        mcap.write_bytes(content)
        before = mcap.stat().st_mtime_ns
        app.dispatch('F')
        assert mcap.read_bytes() == content and mcap.stat().st_mtime_ns == before
    assert not calls and app.finalizing is None


def test_finalize_worker_rechecks_writer_started_after_click(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    folder, mcap = segment(path)
    from mhseals_testing.ui import app as module
    pending = []
    class DeferredThread:
        def __init__(self, target, args, **kw):
            pending.append(lambda: target(*args))
        def start(self):
            pass
    monkeypatch.setattr(module.threading, 'Thread', DeferredThread)
    app.dispatch('F')
    assert app.finalizing == path.resolve()
    monkeypatch.setattr(app.processes, 'running', lambda key: True)
    app.recorder.segment = folder
    before = mcap.stat().st_mtime_ns
    pending[0]()
    assert mcap.exists() and mcap.stat().st_mtime_ns == before
    assert app.finalizing is None and 'refused' in app.notice


def test_background_finalize_space_second_job_and_recording_start(tmp_path, monkeypatch):
    import threading
    app, boat, path = setup_app(tmp_path)
    folder, mcap = segment(path, b'valid')
    entered, release = threading.Event(), threading.Event()
    def run(argv):
        entered.set()
        assert release.wait(5)
        (folder / 'metadata.yaml').write_text('indexed')
        return 0
    app.quiet_run = run
    rebuilds = []
    monkeypatch.setattr(app, 'refresh_bag_index', lambda bag: rebuilds.append(bag.path))
    app.dispatch('F')
    try:
        assert entered.wait(5)
        first = app.finalize_thread
        app.dispatch('F')
        assert app.finalize_thread is first
        app.set_recording('lean')
        assert app.recorder.segment is None and 'refused' in app.notice
        app.dispatch('d')
        app.dispatch('P')
        assert path.exists()
        app.dispatch('space')
        assert boat.holds == ['keys'] and not app.armed
    finally:
        release.set()
        app.finalize_thread.join(5)
    assert not app.finalize_thread.is_alive()
    assert app.finalizing is None and rebuilds == [path]


def test_historical_finalize_while_other_session_records_and_partial_error(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    folder, mcap = segment(path)
    second = path / 'bag' / '01'
    second.mkdir()
    (second / 'data.mcap').write_bytes(b'data')
    other = tmp_path / 'sessions' / 'other'
    app.paths = SessionPaths.from_output(other)
    app.recorder.segment = other / 'bag' / '00'
    monkeypatch.setattr(app.processes, 'running', lambda key: True)
    def fail(argv):
        raise RuntimeError('runner failure')
    app.quiet_run = fail
    rebuilds = []
    monkeypatch.setattr(app, 'refresh_bag_index', lambda bag: rebuilds.append(bag.path))
    app.dispatch('F')
    app.finalize_thread.join(5)
    assert app.finalizing is None and 'runner failure' in app.notice
    assert not mcap.exists() and (second / 'data.mcap').read_bytes() == b'data'
    assert rebuilds == [path]


def test_mixed_prune_success_rebuild_once(tmp_path, monkeypatch):
    app, _, path = setup_app(tmp_path)
    legacy = tmp_path / 'old'
    legacy.mkdir()
    (legacy / 'empty.mcap').touch()
    app.bags_view()
    app.bag_scan = (app.clock(), sorted(app.bag_scan[1], key=lambda b: not b.legacy), (0, 0, 0))
    rebuilds = []
    from mhseals_testing.ui import app as module
    original = module.index.rebuild
    def rebuild(root):
        rebuilds.append(root)
        return original(root)
    monkeypatch.setattr(module.index, 'rebuild', rebuild)
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    app.dispatch('P')
    assert rebuilds == [tmp_path]
    assert not path.exists() and not legacy.exists()
    assert data.read_json(tmp_path / 'INDEX.json')['sessions'] == []
