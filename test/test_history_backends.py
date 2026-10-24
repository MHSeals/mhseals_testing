from mhseals_testing.history import History, adapters, backends, revert
from test_app import make_app


def seed(tmp_path):
    path = tmp_path / 'rtk.yaml'
    history = History(tmp_path / 'h')
    path.write_text('# one\nbaud: 9600\n')
    first = adapters.record_file(history, 'rtk', 'test', path)
    path.write_text('baud: 115200\n')
    adapters.record_file(history, 'rtk', 'test', path)
    return history, path, first


def test_file_backend_reverts_the_file_comments_and_all(tmp_path):
    history, path, first = seed(tmp_path)
    reloads = []
    back = {'rtk': backends.FileBackend('rtk', path, lambda: reloads.append(1))}
    plan = revert.plan(history, back, revert.Env(), first['id'], 'rtk')
    out = revert.execute(history, back, revert.Env(), plan)
    assert out.status == 'reverted' and path.read_text() == '# one\nbaud: 9600\n' and reloads == [1]
    assert history.head('rtk')['revert_of'] == first['id']


def test_driver_disagreeing_on_a_live_key_fails_verify(tmp_path):
    version = {'value': {'overrides': {'gain': 1}, 'effective': {'gain': 1, 'pin': 3}}}
    path = tmp_path / 'hw.yaml'
    path.write_text('gain: 1\n')
    back = backends.FileBackend('hardware', path, live_keys=frozenset({'gain'}), driver=lambda keys: {'gain': 2})
    assert back.verify(version, 1) is False
    back.driver = lambda keys: {'gain': 1}
    assert back.verify(version, 1) is False  # missing pin evidence
    back.driver = lambda keys: {'gain': 1, 'pin': 3}
    assert back.verify(version, 1) is True


def test_app_wires_a_backend_for_every_file_subsystem(tmp_path):
    app, _ = make_app(tmp_path)
    assert set(app.config_backends) >= {'hardware', 'rtk', 'health', 'keymap'}
    assert all(isinstance(b, revert.Backend) for b in app.config_backends.values())


def test_hardware_revert_to_an_invalid_version_rolls_back(tmp_path):
    from types import SimpleNamespace
    path = tmp_path / 'hardware.yaml'
    history = History(tmp_path / 'h')
    path.write_text('output_backend: nonsense_backend\nchannel_map: 7\n')
    bad = adapters.record_file(history, 'hardware', 'test', path)
    path.write_text('{}\n')
    adapters.record_file(history, 'hardware', 'test', path)
    reload = lambda: SimpleNamespace(ok='nonsense' not in path.read_text(), error='bad config')
    back = {'hardware': backends.FileBackend('hardware', path, reload)}
    plan = revert.plan(history, back, revert.Env(), bad['id'], 'hardware')
    head = history.head('hardware')
    out = revert.execute(history, back, revert.Env(), plan)
    assert out.status == 'rollback_unverified' and path.read_text() == '{}\n'
    assert history.head('hardware')['id'] == head['id'] and head.get('revert_of') is None


def test_an_app_hardware_revert_is_recorded_once_not_also_as_an_external_edit(tmp_path):
    app, _ = make_app(tmp_path)
    config = tmp_path / 'hardware.yaml'
    config.write_text('frequency: 50.0\n')
    first = adapters.record_file(app.history, 'hardware', 'test', config)
    config.write_text('frequency: 60.0\n')
    adapters.record_file(app.history, 'hardware', 'test', config)
    plan = revert.plan(app.history, app.config_backends, revert.Env(), first['id'], 'hardware')
    count = len(app.history.changes('hardware'))
    out = revert.execute(app.history, app.config_backends, revert.Env(), plan)
    assert out.status == 'rollback_unverified'
    assert len(app.history.changes('hardware')) == count
    assert app.history.head('hardware').get('revert_of') is None


def test_hardware_requires_complete_driver_evidence(tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('{}\n')
    back = backends.FileBackend('hardware', path, live_keys={'frequency', 'command_timeout'})
    value, raw, meta = back.read()
    version = dict(value=value, raw=raw, meta=meta)
    assert back.persisted(version)
    for seen in (None, {}, {'frequency': value['effective']['frequency']}):
        back.driver = lambda keys: seen
        assert not back.verify(version, 1)
    back.driver = lambda keys: {k: value['effective'][k] for k in keys}
    assert back.verify(version, 1)


def test_hardware_unverified_rollback_reports_file_evidence(tmp_path):
    path = tmp_path / 'hardware.yaml'
    history = History(tmp_path / 'h')
    path.write_text('frequency: 50.0\n')
    first = adapters.record_file(history, 'hardware', 'test', path)
    path.write_text('frequency: 60.0\n')
    head = adapters.record_file(history, 'hardware', 'test', path)
    back = {'hardware': backends.FileBackend('hardware', path, live_keys={'frequency'})}
    plan = revert.plan(history, back, revert.Env(), first['id'], 'hardware')
    out = revert.execute(history, back, revert.Env(), plan)
    assert out.status == 'rollback_unverified'
    assert path.read_text() == 'frequency: 60.0\n'
    assert history.head('hardware')['id'] == head['id']


def test_non_live_pin_revert_cannot_verify_from_unchanged_timeout(tmp_path):
    path = tmp_path / 'hardware.yaml'
    history = History(tmp_path / 'h')
    path.write_text('mosfet_line: 29\n')
    first = adapters.record_file(history, 'hardware', 'test', path)
    path.write_text('mosfet_line: 30\n')
    head = adapters.record_file(history, 'hardware', 'test', path)
    runtime = dict(adapters.read('hardware', path)[0]['effective'])
    requested = []
    def driver(keys):
        requested.append(set(keys))
        return {key: runtime[key] for key in keys}
    backend = backends.FileBackend('hardware', path, live_keys={'command_timeout'}, driver=driver)
    back = {'hardware': backend}
    env = revert.Env()
    plan = revert.plan(history, back, env, first['id'], 'hardware')
    out = revert.execute(history, back, env, plan)
    assert out.status != 'reverted'
    assert 'mosfet_line' in requested[0]
    assert runtime['mosfet_line'] == 30
    assert history.head('hardware')['id'] == head['id']
