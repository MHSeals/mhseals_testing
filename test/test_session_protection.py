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
    assert (tmp_path / 'INDEX.json').exists()
    app.dispatch('p')
    monkeypatch.setattr(app, 'ask_text', lambda *a: 'Wind trial')
    app.dispatch('n')
    assert data.read_json(path / 'session.json')['title'] == 'Wind trial'
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
    app, _, path = setup_app(tmp_path)
    legacy = tmp_path / 'legacy'
    legacy.mkdir()
    (legacy / 'empty.mcap').touch()
    (tmp_path / 'legacy.report.json').write_text('{}')
    unrelated = tmp_path / 'legacy.extra'
    unrelated.mkdir()
    app.invalidate_bags()
    real_delete = bags.delete
    calls = []
    def delete(bag):
        if not bag.legacy:
            raise OSError('fixture failure')
        real_delete(bag)
    monkeypatch.setattr(bags, 'delete', delete)
    monkeypatch.setattr(app, 'refresh_bag_index', lambda bag: calls.append(bag.path.parent))
    monkeypatch.setattr(app, 'choose', lambda *a, **kw: 'Delete')
    app.bag_scan = (app.clock(), sorted(app.bags_view()[0], key=lambda b: not b.legacy), (0, 0, 0))
    app.dispatch('P')
    assert calls == [tmp_path]
    assert not legacy.exists() and path.exists() and unrelated.exists()


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
