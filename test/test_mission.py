import pytest

from mhseals_testing.analysis.mission import score
from mhseals_testing.procedures.mission import MISSION_DIR, available

RUBRIC = dict(time_limit=100, required={'gate_cleared': 2}, points={'gate_cleared': 10},
              max_counts={'gate_cleared': 2})


def events(*names, outcome='SUCCESS', elapsed=60.0):
    rows = [dict(event=name, elapsed=10.0 * index) for index, name in enumerate(names)]
    return rows + [dict(event='task_finished', outcome=outcome, elapsed=elapsed)]


def test_complete_run_passes_with_capped_points():
    result = score(events('gate_cleared', 'gate_cleared', 'gate_cleared'), RUBRIC)
    assert result['verdict'] == 'PASS' and result['points'] == 20 and result['possible'] == 20


@pytest.mark.parametrize('run,reason', [
    (events('gate_cleared'), 'missing'),
    (events('gate_cleared', 'gate_cleared', outcome='FAILURE'), 'outcome'),
    (events('gate_cleared', 'gate_cleared', elapsed=150.0), 'late'),
    ([dict(event='gate_cleared', elapsed=5.0)], 'outcome'),  # never finished
])
def test_incomplete_runs_fail_and_say_why(run, reason):
    result = score(run, RUBRIC)
    assert result['verdict'] == 'FAIL'
    assert {'missing': result['missing'], 'outcome': result['outcome'] != 'SUCCESS',
            'late': result['late']}[reason]


def test_every_shipped_rubric_is_scorable():
    import yaml
    assert set(available()) == {'nav_channel', 'follow_path', 'docking', 'speed_challenge',
                                'return_home', 'mission'}
    for name in available():
        rubric = yaml.safe_load((MISSION_DIR / f'{name}.yaml').read_text())
        assert rubric['task'] == name
        assert score([], rubric)['verdict'] == 'FAIL'
