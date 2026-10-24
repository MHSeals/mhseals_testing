"""boat_trial config: every exit code, and the preview-before-write rule."""

import io
import json

import pytest

from mhseals_testing.harness.config_cli import ApplyFailed, RollbackFailed, entry, flatten, main, parse_since
from mhseals_testing.history import History


class Fake:
    """A subsystem whose live value is a dict; failure modes are switches."""

    def __init__(self, value, **kw):
        self.value, self.fail, self.blocked, self.confirm_word = dict(value), None, [], kw.get('word')
        self.applied = []
        self.after = 'same'          # what read() reports once an apply has happened: same | moved | none
        self.meta, self.moved = {}, False

    def read(self):
        if self.moved and self.after == 'none':
            return None
        return dict(self.value), None, dict(self.meta)

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
        if self.after == 'moved':       # the device normalises on write: extra default keys, new defaults hash
            self.value['z'] = 0
            self.meta = {'defaults_sha': 'new'}
        self.moved = True
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
    assert run('revert', 'nope')[0] == 2
    assert run('log', '--since', 'soon')[0] == 2
    assert parse_since('3d').tzinfo is not None


def test_observe_exit_codes(env):
    run, hist, adapters = env
    assert run('observe')[0] == 1                       # live equals head everywhere
    adapters['hardware'].value['b'] = 3
    code, text = run('observe', 'hardware')
    assert code == 0 and 'observed' in text and hist.head('hardware')['applied'] == 'observed'
    assert run('observe', 'nope')[0] == 2


def test_revert_without_yes_previews_and_writes_nothing(env):
    run, hist, adapters = env
    code, text = run('revert', 'c0001')
    assert code == 2 and 'preview only' in text and 'a  9 -> 1' in text
    assert adapters['hardware'].applied == [] and hist.head('hardware')['id'] == 'c0002'
    assert run('revert', 'c0001', '--dry-run')[0] == 0


def test_revert_applies_and_records(env):
    run, hist, adapters = env
    code, text = run('revert', 'c0001', '--yes', '--json')
    assert code == 0 and json.loads(text)['outcome'] == 'applied'
    head = hist.head('hardware')
    assert head['revert_of'] == 'c0001' and head['source'] == 'cli:revert' and adapters['hardware'].value['a'] == 1
    assert run('revert', 'current', '--yes')[0] == 1    # already there


def test_revert_blocked_failed_and_unrollbackable(env):
    run, hist, adapters = env
    hw = adapters['hardware']
    hw.blocked = ['armed']
    assert run('revert', 'c0001', '--yes')[0] == 2
    hw.blocked, hw.fail = [], 'verify'
    assert run('revert', 'c0001', '--yes')[0] == 3
    hw.fail = 'rollback'
    hw.value['b'] = 4                                   # the failed write left something behind
    assert run('revert', 'c0001', '--yes')[0] == 2      # outside change: review first
    assert hist.head('hardware')['applied'] == 'observed'
    assert run('revert', 'c0001', '--yes')[0] == 4
    assert hist.head('hardware')['applied'] == 'observed'


def test_fcu_revert_needs_the_typed_word(env):
    run, _, adapters = env
    assert run('revert', 'c0003', '--yes')[0] == 2                              # headless
    assert run('revert', 'c0003', '--yes', ask=lambda _: 'yes')[0] == 2
    assert adapters['fcu'].applied == []


def test_fcu_revert_leaves_calibration_unless_asked(env):
    run, hist, adapters = env
    word = lambda _: 'FCU'
    assert run('revert', 'c0003', '--yes', ask=word)[0] == 0
    assert adapters['fcu'].value == {'EK3_SRC1_YAW': 1.0, 'CAL_X': 7.0}
    assert run('revert', 'c0003', '--yes', '--include-calibration', ask=word)[0] == 0
    assert adapters['fcu'].value == {'EK3_SRC1_YAW': 1.0, 'CAL_X': 5.0}


def test_revert_keeps_nested_shape_and_live_calibration(tmp_path):
    hist = History(tmp_path / 'h')
    nested = Fake({})
    nested.calibration = lambda path: path.startswith('imu.')
    old = {'pins': {'l': 1}, 'imu': {'off': 0}}
    live = {'pins': {'l': 5}, 'imu': {'off': 3, 'scale': 2}}
    hist.record('hardware', old, source='t')
    nested.value = live
    hist.record('hardware', live, source='t')
    out = io.StringIO()
    assert main(['revert', 'c0001', '--yes'], adapters={'hardware': nested}, history=hist, out=out, ask=None) == 0
    expect = {'pins': {'l': 1}, 'imu': {'off': 3, 'scale': 2}}
    assert nested.value == expect
    assert hist.version(hist.changes('hardware')[-1])['value'] == expect


def test_set_fcu_needs_reboot_flag_terminal_and_typed_word(env, tmp_path):
    run, hist, adapters = env
    param = tmp_path / 'p.param'
    param.write_text('EK3_SRC1_YAW,1\n')
    assert run('set', 'fcu', str(param), '--yes')[0] == 2                      # no --reboot
    assert run('set', 'fcu', str(param), '--reboot', '--yes')[0] == 2          # headless: nobody can type FCU
    assert run('set', 'fcu', str(param), '--reboot', '--yes', ask=lambda _: 'yes')[0] == 2
    assert adapters['fcu'].applied == []
    assert run('set', 'fcu', str(param), '--reboot')[0] == 2                   # preview
    code, text = run('set', 'fcu', str(param), '--reboot', '--yes', ask=lambda _: 'FCU')
    assert code == 0 and adapters['fcu'].applied == [True] and hist.head('fcu')['source'] == 'cli:set'
    bad = tmp_path / 'bad.param'
    bad.write_text('NOPE,1\n')
    assert run('set', 'fcu', str(bad), '--reboot')[0] == 2
    assert run('set', 'fcu', str(tmp_path / 'missing'), '--reboot')[0] == 2


def test_set_plain_subsystem_and_flatten(env, tmp_path):
    run, hist, adapters = env
    f = tmp_path / 'c.txt'
    f.write_text('a,9\n')
    assert run('set', 'hardware', str(f), '--yes')[0] == 1                    # same as live
    assert run('set', 'nope', str(f))[0] == 2
    assert flatten({'x': {'y': [1, 2]}}) == {'x.y': [1, 2]}


@pytest.mark.parametrize('verb', ['revert', 'set'])
def test_commit_records_the_live_read_not_the_target(env, tmp_path, verb):
    run, hist, adapters = env
    adapters['hardware'].after = 'moved'
    f = tmp_path / 'c.txt'
    f.write_text('a,5\n')
    argv = ('revert', 'c0001', '--yes') if verb == 'revert' else ('set', 'hardware', str(f), '--yes')
    assert run(*argv)[0] == 0
    head = hist.version(hist.head('hardware'))
    assert head['value'] == adapters['hardware'].value and head['value']['z'] == 0
    assert head['meta'] == {'defaults_sha': 'new'}
    assert run('observe', 'hardware')[0] == 1 and 'nothing changed' in run('observe', 'hardware')[1]


@pytest.mark.parametrize('verb', ['revert', 'set'])
def test_commit_records_the_target_when_read_is_gone_after_apply(env, tmp_path, verb):
    run, hist, adapters = env
    adapters['hardware'].after = 'none'
    f = tmp_path / 'c.txt'
    f.write_text('a,5\n')
    argv = ('revert', 'c0001', '--yes') if verb == 'revert' else ('set', 'hardware', str(f), '--yes')
    assert run(*argv)[0] == 0
    want = 1 if verb == 'revert' else 5
    assert hist.version(hist.head('hardware'))['value'] == {'a': want, 'b': 2}


def test_main_never_reads_stdin_but_entry_may(env, monkeypatch):
    """main(ask=None) must not block on input(); only entry() wires the terminal."""
    run, *_ = env
    monkeypatch.setattr('sys.stdin', type('Tty', (), {'isatty': lambda self: True})())
    monkeypatch.setattr('builtins.input', lambda *_: pytest.fail('main read stdin'))
    assert run('revert', 'c0003', '--yes')[0] == 2
    answers = []
    monkeypatch.setattr('builtins.input', lambda prompt: answers.append(prompt) or 'FCU')
    out = io.StringIO()
    assert entry(['revert', 'c0003', '--yes'], adapters=env[2], history=env[1], out=out) == 0
    assert answers                                      # a terminal was found, so the word was asked
