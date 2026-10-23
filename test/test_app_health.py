"""App: health gates procedures and is snapshotted before teardown."""

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
