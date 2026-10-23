"""Engine: staleness, missing policy, severity cap and hysteresis."""

import dataclasses

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import CheckSpec, Fact, Result


@dataclasses.dataclass(frozen=True)
class Params:
    pass


@pytest.fixture(autouse=True)
def probe_kind():
    """A kind whose status is the value of the fact 'level'."""
    registry.KINDS.pop('t_level', None)

    @registry.kind('t_level', Params, needs=('p',))
    def evaluate(params, facts):
        return Result(facts['level'].value)

    yield
    registry.KINDS.pop('t_level', None)


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


def make(**overrides):
    clock = Clock()
    spec = CheckSpec(id='c', kind='t_level', params=Params(), **overrides)
    return Engine([spec], periods={'p': 1.0}, clock=clock), clock


def step(engine, clock, level, at, probes=None):
    clock.now = at
    facts = {'level': Fact(level)}
    probes = {'p': (at, None)} if probes is None else probes
    return engine.tick(facts, probes).results.get('c')


def test_warn_is_shown_only_after_it_persists_and_clears_after_clear_after():
    engine, clock = make()
    assert step(engine, clock, 'warn', 0).status == 'ok'
    assert step(engine, clock, 'warn', 1.9).status == 'ok'
    assert step(engine, clock, 'warn', 2.0).status == 'warn'
    assert step(engine, clock, 'ok', 3.0).status == 'warn'
    assert step(engine, clock, 'ok', 6.0).status == 'ok'


def test_a_flicker_never_shows_and_fail_is_immediate():
    engine, clock = make()
    step(engine, clock, 'warn', 0)
    step(engine, clock, 'ok', 1)
    step(engine, clock, 'ok', 4)                                # better for clear_after
    assert step(engine, clock, 'warn', 5).status == 'ok'        # the timer restarted
    assert step(engine, clock, 'warn', 6.5).status == 'ok'
    engine, clock = make(severity='fail')
    assert step(engine, clock, 'fail', 0).status == 'fail'


def test_transitions_are_reported_once():
    engine, clock = make(severity='fail')
    clock.now = 0
    report = engine.tick({'level': Fact('fail')}, {'p': (0, None)})
    assert [(t.id, t.old, t.new) for t in report.transitions] == [('c', 'ok', 'fail')]
    clock.now = 1
    assert engine.tick({'level': Fact('fail')}, {'p': (1, None)}).transitions == ()


def test_severity_caps_the_shown_status():
    engine, clock = make(severity='info')
    step(engine, clock, 'fail', 0)
    assert step(engine, clock, 'fail', 2).status == 'info'


def test_stale_or_failed_probe_is_unknown_after_ten_seconds():
    engine, clock = make()
    assert step(engine, clock, 'ok', 0).status == 'ok'
    result = step(engine, clock, 'ok', 20, probes={'p': (10.0, None)})
    assert result.status == 'ok'                    # unknown not yet persisted
    result = step(engine, clock, 'ok', 31, probes={'p': (10.0, None)})
    assert result.status == 'unknown'
    assert 'no p sample for 21.0 s' in result.detail
    result = step(engine, clock, 'ok', 32, probes={'p': (31.0, 'boom')})
    assert 'boom' in result.detail


def test_missing_policy_skip_hides_and_fail_fails():
    engine, clock = make(missing='skip')
    assert step(engine, clock, 'ok', 0, probes={}) is None
    engine, clock = make(missing='fail', severity='fail')
    assert step(engine, clock, 'ok', 0, probes={}).status == 'fail'


def test_when_mismatch_reports_ok():
    engine, clock = make(when={'armed': True})
    clock.now = 0
    report = engine.tick({'level': Fact('fail')}, {'p': (0, None)}, session={'armed': False})
    assert report.results['c'].status == 'ok'


def test_a_kind_that_raises_or_returns_a_bad_status_reads_unknown():
    for name, body in (('t_raise', lambda p, f: f['absent'].value),
                       ('t_bad', lambda p, f: Result('bad'))):
        registry.kind(name, Params, needs=('p',))(body)
        try:
            engine = Engine([CheckSpec(id='c', kind=name, params=Params(), missing='unknown')],
                            periods={'p': 1.0}, clock=lambda: 0.0)
            result = engine.tick({}, {'p': (0.0, None)}).results['c']
        finally:
            registry.KINDS.pop(name, None)
        assert result.status == 'ok' and name in result.detail     # unknown is debounced in


def run(levels, seconds=60, dt=0.5, **overrides):
    """Shown statuses when `levels(i)` gives the raw level at each tick."""
    engine, clock = make(**overrides)
    return [step(engine, clock, levels(i), i * dt).status for i in range(int(seconds / dt))]


@pytest.mark.parametrize('other', ['unknown', 'info'])
def test_alternating_bad_results_are_shown_as_the_worst(other):
    shown = run(lambda i: 'warn' if i % 2 else other)
    assert shown[-1] == 'warn'


def test_a_mostly_bad_check_with_brief_ok_blips_is_shown():
    shown = run(lambda i: 'ok' if i % 4 == 0 else 'warn')       # warn 75% of the time
    assert shown[-1] == 'warn'
    shown = run(lambda i: 'unknown' if i % 4 == 0 else 'warn')
    assert shown[-1] == 'warn'


def test_a_shown_warn_clears_only_after_continuous_better_results():
    engine, clock = make()
    step(engine, clock, 'warn', 0)
    step(engine, clock, 'warn', 2)
    step(engine, clock, 'ok', 3)
    assert step(engine, clock, 'warn', 4).status == 'warn'      # the clear timer restarted
    step(engine, clock, 'ok', 5)
    assert step(engine, clock, 'ok', 8).status == 'ok'


def test_gate_blocks_on_fail_warns_on_warn_and_honours_bench_and_mutes():
    engine, clock = make(severity='fail', gates=('motion', 'p'), title='Level')
    assert engine.gate('p')[1] == ['health has not been evaluated yet']
    step(engine, clock, 'fail', 0)
    assert engine.gate('p') == (['Level: fail'], [])
    assert engine.gate('p', bench=True)[0]          # named by key, so bench does not lift it
    assert engine.gate('other')[0]                # motion applies to every key
    assert engine.gate('other', bench=True) == ([], [])
    engine.mute('c')
    assert engine.gate('p') == ([], [])
    engine.mute('c', False)
    clock.now = 10
    engine.tick({'level': Fact('warn')}, {'p': (10, None)})
    clock.now = 13
    engine.tick({'level': Fact('warn')}, {'p': (13, None)})
    assert engine.gate('p')[1] and not engine.gate('p')[0]


def test_motion_gate_is_lifted_by_the_bench_override():
    engine, clock = make(severity='fail', gates=('motion',))
    step(engine, clock, 'fail', 0)
    assert engine.gate('x')[0] and engine.gate('x', bench=True) == ([], [])


def test_a_muted_check_does_not_transition():
    engine, clock = make(severity='fail')
    engine.mute('c')
    clock.now = 0
    report = engine.tick({'level': Fact('fail')}, {'p': (0, None)})
    assert report.transitions == () and report.muted == {'c'}


def test_an_old_fact_is_ignored_even_when_its_probe_just_ran():
    engine, clock = make(severity='fail')
    clock.now = 10
    old = {'level': Fact('fail', time=5.0)}          # 5 s old, limit is 3 periods of 1 s
    engine.tick(old, {'p': (10, None)})
    clock.now = 21                                   # unknown is shown after 10 s
    report = engine.tick({'level': Fact('fail', time=5.0)}, {'p': (21, None)})
    assert report.results['c'].status == 'unknown'
    clock.now = 10
    fresh = {'level': Fact('fail', time=9.0)}
    assert Engine(engine.specs, {'p': 1.0}, clock).tick(fresh, {'p': (10, None)}).results['c'].status == 'fail'


def test_motion_gate_ignores_procedures_that_do_not_move_the_boat():
    engine, clock = make(severity='fail', gates=('motion',))
    step(engine, clock, 'fail', 0)
    assert engine.gate('x', motion=False) == ([], [])
    assert engine.gate('x', motion=True)[0]
