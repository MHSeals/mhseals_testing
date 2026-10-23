from mhseals_testing.harness.ekf import CHILD, EkfRestart, command
from mhseals_testing.history import History


class Procs:
    def __init__(self, log):
        self.log = log

    def stop(self, name):
        self.log.append(('stop', name))

    def start(self, name, cmd):
        self.log.append(('start', name, cmd))


def make(tmp_path, log, goal=False, velocity=False, procedure=False, carry=True, overlay=None,
         ready=lambda: True, **extra):
    path = tmp_path / 'ekf.yaml'
    if overlay is not None:
        path.write_text(overlay)
    events = []
    return EkfRestart(Procs(log), History(tmp_path / 'h'), sim=False, goal_active=lambda: goal,
                      procedure_active=lambda: procedure, velocity_active=lambda: velocity, emit=lambda p, **d: events.append((p, d)),
                      to_ll=lambda: log.append('toLL') or (1.5, 2.5, 0.0),
                      datum_ready=lambda: log.append('ready') or ready(),
                      set_datum=lambda *a: log.append(('datum', a)), carry_datum=carry,
                      overlay=path, **extra), events


def test_blocked_during_goal_procedure_or_velocity_control(tmp_path):
    for kw, reason in (({'goal': True}, 'goal'), ({'procedure': True}, 'procedure'),
                       ({'velocity': True}, 'velocity')):
        log = []
        restart, _ = make(tmp_path, log, **kw)
        result = restart.restart()
        assert not result['restarted'] and reason in result['blocked'][0]
        assert log == [] and restart.history.changes() == []


def test_datum_calls_in_order_and_history_record(tmp_path):
    log = []
    restart, events = make(tmp_path, log, overlay='ekf_local:\n  ros__parameters:\n    frequency: 20.0\n')
    assert restart.restart()['datum'] == (1.5, 2.5, 0.0)
    assert [e if isinstance(e, str) else e[0] for e in log] == ['toLL', 'stop', 'start', 'ready', 'datum']
    assert log[2][2] == command(False, tmp_path / 'ekf.yaml') and log[1] == ('stop', CHILD)
    assert [c['subsystem'] for c in restart.history.changes()] == ['ekf']
    assert events == [('ekf_restart', {'datum': [1.5, 2.5, 0.0]})]


def test_without_flag_the_redatum_is_logged(tmp_path):
    log = []
    restart, events = make(tmp_path, log, carry=False)
    assert restart.restart()['restarted']
    assert 'toLL' not in log and all(e[0] != 'datum' for e in log if isinstance(e, tuple))
    assert events == [('ekf_restart', {'datum': None}), ('map_redatumed', {'carried': False})]


def test_datum_waits_for_navsat_then_times_out(tmp_path):
    log, now, answers = [], [0.0], iter([False, False, True])

    def tick(seconds):
        now[0] += seconds

    restart, events = make(tmp_path, log, ready=lambda: next(answers), sleep=tick,
                           clock=lambda: now[0])
    assert restart.restart()['datum'] == (1.5, 2.5, 0.0)
    assert log.count('ready') == 3 and now[0] > 0 and log[-1][0] == 'datum'
    log.clear()
    now[0] = 0.0
    restart, events = make(tmp_path, log, ready=lambda: False, sleep=tick, clock=lambda: now[0])
    result = restart.restart()
    assert result['restarted'] and result['datum'] is None and now[0] >= 10
    assert all(e[0] != 'datum' for e in log if isinstance(e, tuple))
    assert events[-1] == ('map_redatumed', {'carried': False})


def test_broken_overlay_is_refused_without_restart(tmp_path):
    log = []
    restart, events = make(tmp_path, log, overlay='a: [unclosed\n')
    result = restart.restart()
    assert not result['restarted'] and 'not valid YAML' in result['blocked'][0]
    assert log == [] and events == []
