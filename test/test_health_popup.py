"""M7: ``!`` opens the Health popup, listing the checks that are not ok, worst first."""

from types import SimpleNamespace

from mhseals_testing.health.model import Result
from mhseals_testing.ui.popups import OPENERS, health


def engine(**results):
    specs = [SimpleNamespace(id=name, title=name.title()) for name in results]
    return SimpleNamespace(specs=specs, last=results, muted={'muted'})


def test_rows_list_the_not_ok_checks_worst_first_and_leave_muted_ones_out():
    found = health.rows(engine(net=Result('warn', detail='ping 100 %'), temp=Result('fail', detail='80 C'),
                               gps=Result('ok'), muted=Result('fail')))
    assert found == [('FAIL', 'Temp · 80 C'), ('WARN', 'Net · ping 100 %')]
    assert health.rows(engine(gps=Result('ok'))) == [('OK', 'all checks ok')]


def test_the_bang_key_opens_it(tmp_path):
    from test_app import make_app
    app, _ = make_app(tmp_path)
    opened = []
    real = OPENERS['health']
    OPENERS['health'] = opened.append
    try:
        app.health = SimpleNamespace(engine=engine(temp=Result('fail')))
        app.dispatch('!')
    finally:
        OPENERS['health'] = real
    assert opened == [app]
