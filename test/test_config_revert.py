import contextlib

import pytest

from mhseals_testing.history import History
from mhseals_testing.history import revert


class Fake(revert.Backend):
    """A subsystem whose live value is a dict; ``fail_on`` makes verify time out for those values."""
    live_keys = frozenset({'gain'})

    def __init__(self, value, fail_on=()):
        self.value, self.fail_on, self.applied = value, list(fail_on), []

    def read(self):
        return dict(self.value), None, {}

    def apply(self, version):
        self.value = dict(version['value'])
        self.applied.append(version['sha'])

    def verify(self, version, timeout):
        if version['value'] in self.fail_on:
            raise TimeoutError
        return True


def setup(tmp_path, sub='hardware', **kw):
    h = History(tmp_path)
    first = h.record(sub, {'gain': 1, 'pin': 7}, source='t')
    h.record(sub, {'gain': 2, 'pin': 7}, source='t')
    h.record(sub, {'gain': 2, 'pin': 9}, source='t')
    b = Fake({'gain': 2, 'pin': 9}, **kw)
    return h, {sub: b}, first


def make_env(armed=False, held=None, **probes):
    held = held if held is not None else []

    @contextlib.contextmanager
    def hold(token):
        held.append(('on', token))
        yield
        held.append(('off', token))
    return revert.Env(probes={k: (lambda v=v: v) for k, v in probes.items()}, armed=lambda: armed, hold=hold)


def test_plan_lists_changed_keys_and_applies(tmp_path):
    h, b, first = setup(tmp_path)
    p = revert.plan(h, b, make_env(), first['id'], 'hardware')
    assert p.keys == ['gain', 'pin'] and p.ok and not p.hold
    out = revert.execute(h, b, make_env(), p)
    assert out.status == 'reverted' and b['hardware'].value == {'gain': 1, 'pin': 7}
    head = h.head('hardware')
    assert head['revert_of'] == first['id'] and head['source'] == 'tui:revert'


@pytest.mark.parametrize('sub,probe', [('hardware', 'procedure'), ('hardware', 'takeover'), ('hardware', 'locked'),
                                       ('hardware', 'hold'), ('hardware', 'pulses'), ('fcu', 'armed'),
                                       ('fcu', 'disconnected'), ('fcu', 'loading'), ('nav2', 'goal')])
def test_each_blocker_refuses_plan_and_execute(tmp_path, sub, probe):
    h, b, first = setup(tmp_path, sub)
    env = make_env(armed=probe == 'armed', **({} if probe == 'armed' else {probe: True}))
    p = revert.plan(h, b, env, first['id'], sub)
    assert p.blockers and not p.ok
    out = revert.execute(h, b, env, p)
    assert out.status == 'blocked' and b[sub].applied == []


def test_probe_of_another_subsystem_does_not_block(tmp_path):
    h, b, first = setup(tmp_path, 'nav2')
    assert revert.plan(h, b, make_env(pulses=True, armed=True), first['id'], 'nav2').ok


def test_armed_with_pin_diff_is_refused(tmp_path):
    h, b, first = setup(tmp_path)
    p = revert.plan(h, b, make_env(armed=True), first['id'], 'hardware')
    assert any('pin' in text for text in p.blockers)
    assert revert.execute(h, b, make_env(armed=True), p).status == 'blocked'


def test_armed_live_only_holds_neutral_around_apply(tmp_path):
    h = History(tmp_path)
    first = h.record('hardware', {'gain': 1, 'pin': 7}, source='t')
    h.record('hardware', {'gain': 2, 'pin': 7}, source='t')
    b = {'hardware': Fake({'gain': 2, 'pin': 7})}
    held = []
    env = make_env(armed=True, held=held)
    p = revert.plan(h, b, env, first['id'], 'hardware')
    assert p.keys == ['gain'] and p.hold and p.ok
    assert revert.execute(h, b, env, p).status == 'reverted'
    assert [k for k, _ in held] == ['on', 'off'] and held[0][1] == held[1][1]


def test_verify_timeout_rolls_back_and_records_nothing(tmp_path):
    h, b, first = setup(tmp_path, fail_on=[{'gain': 1, 'pin': 7}])
    n = len(h.changes())
    out = revert.execute(h, b, make_env(), revert.plan(h, b, make_env(), first['id'], 'hardware'))
    assert out.status == 'rolled_back'
    assert b['hardware'].value == {'gain': 2, 'pin': 9} and len(h.changes()) == n


def test_failed_restore_observes_true_state(tmp_path):
    h, b, first = setup(tmp_path, fail_on=[{'gain': 1, 'pin': 7}, {'gain': 2, 'pin': 9}])
    out = revert.execute(h, b, make_env(), revert.plan(h, b, make_env(), first['id'], 'hardware'))
    assert out.status == 'restore_failed'
    assert len(h.changes()) == 3 and b['hardware'].value == {'gain': 2, 'pin': 9}   # nothing recorded; live equals head


def test_live_drift_is_observed_and_not_applied(tmp_path):
    h, b, first = setup(tmp_path)
    p = revert.plan(h, b, make_env(), first['id'], 'hardware')
    b['hardware'].value = {'gain': 3, 'pin': 9}                     # edited outside after the preview
    assert revert.execute(h, b, make_env(), p).status == 'stale'
    assert h.head('hardware')['source'] == 'external' and b['hardware'].applied == []


def test_revert_to_current_is_a_noop(tmp_path):
    h, b, _ = setup(tmp_path)
    p = revert.plan(h, b, make_env(), 'current', 'hardware')
    assert not p.ok and revert.execute(h, b, make_env(), p).status == 'noop'
    assert b['hardware'].applied == []
