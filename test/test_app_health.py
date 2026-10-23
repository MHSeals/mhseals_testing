"""App: health gates procedures and is snapshotted before teardown."""

import dataclasses
from types import SimpleNamespace

import pytest

from mhseals_testing.health.engine import Engine, HealthReport, Transition
from mhseals_testing.health.model import CheckSpec, Fact, Result
from mhseals_testing.health import registry

from test_app import make_app, procedure


def engine_with(status, key):
    registry.KINDS.pop('t_fixed', None)
    registry.kind('t_fixed', type('P', (), {}), needs=())(lambda p, f: Result(f['s'].value))
    spec = CheckSpec(id='c', kind='t_fixed', params=None, severity='fail', gates=(key,), title='Compass')
    engine = Engine([spec])
    engine.tick({'s': Fact(status)}, {})
    return engine


def test_a_failing_gated_check_makes_the_procedure_unavailable(tmp_path):
    app, _ = make_app(tmp_path)
    p = procedure('o')
    assert 'Compass' not in app.availability(p)
    app.health.engine = engine_with('fail', p.key)
    assert app.availability(p).startswith('Compass')
    app.health.engine.mute('c')
    assert 'Compass' not in app.availability(p)


def test_health_is_snapshotted_and_a_bad_window_is_flagged(tmp_path):
    app, _ = make_app(tmp_path)
    p = procedure('o')
    app.health.engine = engine_with('ok', p.key)
    events = []
    app.ros.emit = lambda phase, **d: events.append((phase, d))
    app.health_snapshot('start', p)
    app.journal.record(HealthReport({}, (Transition('c', 'ok', 'fail', Result('fail'), 5.0),)))
    app.health_window(p, began=0.0)
    phases = [e[0] for e in events]
    assert phases == ['health_snapshot', 'health_snapshot', 'trial_window_invalid']
    assert events[-1][1]['reasons'] == ['Compass was fail']


def motion_engine():
    registry.KINDS.pop('t_fixed', None)
    registry.kind('t_fixed', type('P', (), {}), needs=())(lambda p, f: Result(f['s'].value))
    spec = CheckSpec(id='c', kind='t_fixed', params=None, severity='fail', gates=('motion',), title='Link')
    engine = Engine([spec])
    engine.tick({'s': Fact('fail')}, {})
    return engine


def test_a_failing_motion_gate_blocks_motion_procedures_only(tmp_path):
    app, _ = make_app(tmp_path)
    app.health.engine = motion_engine()
    assert 'Link' not in app.availability(procedure('n'))
    assert 'Link' not in app.availability(procedure('o'))
    app.arming = SimpleNamespace(armed=True, fault=lambda: "")
    assert app.availability(procedure('f')).startswith('Link')


def test_end_snapshot_is_taken_when_the_procedure_raises(tmp_path):
    app, _ = make_app(tmp_path)
    p = procedure('o')
    app.health.engine = engine_with('ok', p.key)
    events = []
    app.ros.emit = lambda phase, **d: events.append(phase)
    app.availability = lambda proc: ''
    def boom(ctx):
        raise RuntimeError('x')
    with pytest.raises(RuntimeError):
        app.run_procedure(dataclasses.replace(p, run=boom))
    assert events == ['health_snapshot', 'health_snapshot']
    assert [s['label'] for s in app.journal.snapshots] == ['start', 'end']


def test_health_tick_journals_transitions_so_the_window_goes_invalid(tmp_path):
    app, _ = make_app(tmp_path)
    p = procedure('o')
    app.health.engine = engine_with('ok', p.key)
    ticks = iter([5.0, 6.0])
    app.health.engine.clock = lambda: next(ticks)
    events = []
    app.ros.emit = lambda phase, **d: events.append((phase, d))
    app.health_tick({'s': Fact('fail')}, {})
    app.health.engine.clock = lambda: 10.0
    app.health_window(p, began=0.0)
    assert events[-1][0] == 'trial_window_invalid'


def test_invalid_window_notice_survives_the_complete_notice(tmp_path):
    app, _ = make_app(tmp_path)
    p = procedure('o')
    app.health.engine = engine_with('ok', p.key)
    results = []
    app.availability = lambda proc: None
    app.snapshot = lambda: {'results': list(results)}
    app.journal.window_problems = lambda *a: ['Compass was fail']
    stub = SimpleNamespace(title='Obs', key=p.key, needs=(), run=lambda ctx: results.append({'verdict': 'OK'}))
    app.run_procedure(stub)
    assert app.notice == 'Obs: window invalid, Compass was fail'
    app.journal.window_problems = lambda *a: []
    app.run_procedure(stub)
    assert app.notice == 'Obs complete'
