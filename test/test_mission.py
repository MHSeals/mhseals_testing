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
                                'return_home', 'mission', 'pair_test',
                                'circle_test'}
    for name in available():
        rubric = yaml.safe_load((MISSION_DIR / f'{name}.yaml').read_text())
        assert rubric['task'] == name
        assert score([], rubric)['verdict'] == 'FAIL'


@pytest.mark.parametrize('task,labels', [
    ('pair_test', {'red_pole_buoy', 'green_pole_buoy'}),
    ('circle_test', {'yellow_buoy'}),
])
def test_commissioning_courses_match_detector_labels_and_truth(task, labels, tmp_path, monkeypatch):
    import json
    from mhseals_testing.course import compile_course
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    path = MISSION_DIR.parent / 'courses' / f'{task}.yaml'
    course = json.loads(compile_course(path).read_text())
    assert {item['label'] for item in course['elements']} == labels
    assert all(item['task'] == task for item in course['elements'])


def test_mission_judges_the_wall_clock_window_and_records_the_pause(monkeypatch):
    from contextlib import contextmanager
    from types import SimpleNamespace
    from mhseals_testing.procedures import mission
    runner = SimpleNamespace(paused=5.0)

    def supervise(*args, **kwargs):
        runner.paused += 3.0       # a 3 s override inside this run
        return 10.0, 17.0, True    # end excludes the pause (wall end is 20)
    runner.supervise = supervise
    watcher = SimpleNamespace(events=[], status={}, reset=lambda: None)
    seen = []

    @contextmanager
    def supervised(*args, **kwargs):
        yield runner
    ctx = SimpleNamespace(
        choose=lambda *a: 'gate', prompt=lambda *a: None, supervise=supervised,
        profile=SimpleNamespace(use_sim_time=False), record=lambda r: None,
        services={'task_watcher': lambda: watcher, 'start': lambda *a: None,
                  'stop': lambda *a: None, 'running': lambda *a: False})
    monkeypatch.setattr(mission, 'available', lambda: ['gate'])
    monkeypatch.setattr(mission, 'score', lambda events, rubric, outcome=None: {})
    monkeypatch.setattr(mission, 'judge', lambda *a: seen.append(a[-2:]))
    monkeypatch.setattr(mission.yaml, 'safe_load', lambda text: {})
    monkeypatch.setattr(mission.Path, 'read_text', lambda self: '')
    result = mission.run(ctx)
    assert seen == [(10.0, 20.0)] and result['paused_s'] == 3.0


FINISHED_FAILURE = events('gate_cleared', 'gate_cleared', outcome='FAILURE')
UNFINISHED = events('gate_cleared', 'gate_cleared')[:2]


@pytest.mark.parametrize('stream,code,outcome', [
    (FINISHED_FAILURE, 0, 'FAILURE'),   # the stream's task_finished outranks the exit code
    (UNFINISHED, 3, 'PREEMPTED'),       # a human override stop
    ([], 4, 'REFUSED'),                 # started while a human had control: never ran
    (FINISHED_FAILURE, 4, 'FAILURE'),   # the stream still wins
    (UNFINISHED, 7, 'FAILURE'),         # an unknown code
    (UNFINISHED, None, 'INCOMPLETE'),   # still running when the suite stopped it
])
def test_mission_outcome_from_the_stream_then_the_exit_code(monkeypatch, stream, code, outcome):
    from contextlib import contextmanager
    from types import SimpleNamespace
    from mhseals_testing.procedures import mission
    runner = SimpleNamespace(paused=0.0, supervise=lambda *a, **k: (0.0, 5.0, True))
    watcher = SimpleNamespace(events=stream, status={}, reset=lambda: None)
    recorded, started = [], []

    @contextmanager
    def supervised(*args, **kwargs):
        yield runner
    ctx = SimpleNamespace(
        choose=lambda *a: 'gate', prompt=lambda *a: None, supervise=supervised,
        profile=SimpleNamespace(use_sim_time=True), record=recorded.append,
        services={'task_watcher': lambda: watcher, 'start': lambda *a: started.append(a),
                  'stop': lambda *a: None, 'running': lambda *a: False,
                  'returncode': lambda name: code})
    monkeypatch.setattr(mission, 'available', lambda: ['gate'])
    monkeypatch.setattr(mission, 'judge', lambda *a: None)
    monkeypatch.setattr(mission.yaml, 'safe_load', lambda text: dict(RUBRIC))
    monkeypatch.setattr(mission.Path, 'read_text', lambda self: '')
    result = mission.run(ctx)
    # ros2 launch exits 0 whatever its node's code, so the runner must be the child itself.
    assert started == [('task_gate', ['ros2', 'run', 'mhseals_nav', 'run_task', '--task', 'gate',
                                      '--ros-args', '-p', 'use_sim_time:=true'])]
    assert result['outcome'] == outcome and result['verdict'] == 'FAIL'
    assert result['exit_code'] == code and recorded == [result]


def test_a_sim_mission_refuses_to_start_out_of_sight_of_the_course(tmp_path):
    # The 10-26 demo: the player kept running, the boat sat 100 m south of the course while its
    # odometry said (0, 0), and nav_channel searched empty water for 3 min.
    import json
    from types import SimpleNamespace
    from mhseals_testing.procedures import mission
    truth = tmp_path / 'course_truth.json'
    truth.write_text(json.dumps({'startX': -2.3, 'startY': -4.8, 'elements': [
        {'label': 'red_pole_buoy', 'x': 4.8, 'y': -3.3}, {'label': '', 'x': 11.0, 'y': -100.0}]}))
    profile = SimpleNamespace(course_truth=str(truth), ground_truth='/crane/odom', use_sim_time=True)

    def at(x, y):
        return lambda: [(0.0, {'x': 50.0, 'y': 50.0}), (1.0, {'x': x, 'y': y})]
    assert mission.off_course(profile, at(-2.3, -4.8)) == ''
    assert mission.off_course(profile, at(20.0, -3.3)) == ''          # 15 m: in sight
    reason = mission.off_course(profile, at(11.1, -102.8))
    assert reason.startswith('the boat is 100 m from the nearest course element')
    assert mission.off_course(profile, lambda: []) == ''               # no truth sample: no block
    boat = SimpleNamespace(course_truth=str(truth), ground_truth='/crane/odom', use_sim_time=False)
    assert mission.off_course(boat, at(11.1, -102.8)) == ''            # a boat profile never refuses
    assert mission.off_course(SimpleNamespace(course_truth=None, ground_truth=None), at(1e3, 0)) == ''
    recorded = []
    ctx = SimpleNamespace(services={'task': 'nav_channel'}, profile=profile, record=recorded.append,
                          history=lambda name: at(11.1, -102.8)(),
                          prompt=lambda *a: pytest.fail('a refused mission never prompts'))
    result = mission.run(ctx)
    assert result == recorded[0] and result['verdict'] == 'REFUSED' and result['task'] == 'nav_channel'
