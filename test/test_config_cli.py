"""boat_trial config: every exit code, and the preview-before-write rule."""

import io
import json

import pytest

from mhseals_testing.harness.config_cli import ApplyFailed, RollbackFailed, flatten, main, parse_since
from mhseals_testing.history import History


class Fake:
    """A subsystem whose live value is a dict; failure modes are switches."""

    def __init__(self, value, **kw):
        self.value, self.fail, self.blocked, self.confirm_word = dict(value), None, [], kw.get('word')
        self.applied = []

    def read(self):
        return dict(self.value), None, {}

    def blockers(self):
        return list(self.blocked)

    def calibration(self, path):
        return path.startswith('CAL_')

    def needs_reboot(self, paths):
        return 'EK3_SRC1_YAW' in paths

    def parse(self, text, current):
        value = dict(current)
        for line in text.splitlines():
            name, _, number = line.partition(',')
            if name not in current:
                raise ValueError(f'unknown parameter {name}')
            value[name] = float(number)
        return value, None

    def apply(self, version, reboot=False):
        if self.fail == 'rollback':
            raise RollbackFailed('could not restore')
        if self.fail:
            raise ApplyFailed('verify timed out')
        self.value = dict(version['value'])
        self.applied.append(reboot)
        return 'pending' if reboot else 'confirmed'


@pytest.fixture
def env(tmp_path):
    hist = History(tmp_path / 'h')
    hw, fcu = Fake({'a': 1, 'b': 2}), Fake({'EK3_SRC1_YAW': 1.0, 'CAL_X': 5.0}, word='FCU')
    adapters = {'hardware': hw, 'fcu': fcu}
    hist.record('hardware', {'a': 1, 'b': 2}, source='tui:pins')       # c0001
    hist.record('hardware', {'a': 9, 'b': 2}, source='tui:pins')       # c0002
    hw.value = {'a': 9, 'b': 2}
    hist.record('fcu', {'EK3_SRC1_YAW': 1.0, 'CAL_X': 5.0}, source='t')    # c0003
    hist.record('fcu', {'EK3_SRC1_YAW': 8.0, 'CAL_X': 7.0}, source='t')    # c0004
    fcu.value = {'EK3_SRC1_YAW': 8.0, 'CAL_X': 7.0}

    def run(*argv, ask=None):
        out = io.StringIO()
        code = main(list(argv), adapters=adapters, history=hist, out=out, ask=ask)
        return code, out.getvalue()
    return run, hist, adapters


def test_log_diff_name_roundtrip(env):
    run, hist, _ = env
    code, text = run('log', '--json')
    assert code == 0 and [c['id'] for c in json.loads(text)] == ['c0001', 'c0002', 'c0003', 'c0004']
    assert run('log', 'fcu')[1].count('\n') == 2
    code, text = run('diff', '--sub', 'hardware', '--json')
    assert json.loads(text)['changes'][0]['path'] == 'a' and code == 0
    assert run('diff', 'c0001', 'c0002')[0] == 0
    assert run('name', 'c0002', 'split C', '--note', 'dock fit')[0] == 0
    assert hist.resolve('split C')['id'] == 'c0002'


def test_unknown_refs_and_bad_since_are_refused(env):
    run, *_ = env
    assert run('diff', 'c9999')[0] == 2
    assert run('name', 'nope', 'x')[0] == 2
    assert run('log', '--since', 'soon')[0] == 2
    assert parse_since('3d').tzinfo is not None


def test_observe_exit_codes(env):
    run, hist, adapters = env
    assert run('observe')[0] == 1                       # live equals head everywhere
    adapters['hardware'].value['b'] = 3
    code, text = run('observe', 'hardware')
    assert code == 0 and 'observed' in text and hist.head('hardware')['applied'] == 'observed'
    assert run('observe', 'nope')[0] == 2















