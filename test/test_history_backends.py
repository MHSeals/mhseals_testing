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
    assert back.verify(version, 1) is True


def test_app_wires_a_backend_for_every_file_subsystem(tmp_path):
    app, _ = make_app(tmp_path)
    assert set(app.config_backends) >= {'hardware', 'rtk', 'health', 'keymap'}
    assert all(isinstance(b, revert.Backend) for b in app.config_backends.values())
