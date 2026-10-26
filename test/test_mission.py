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
    watcher = SimpleNamespace(events=[], statuses=[], status={}, reset=lambda: None)
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
    monkeypatch.setattr(mission, 'judge', lambda *a, **k: seen.append(a[-2:]))
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
    watcher = SimpleNamespace(events=stream, statuses=[], status={}, reset=lambda: None)
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
    monkeypatch.setattr(mission, 'judge', lambda *a, **k: None)
    monkeypatch.setattr(mission.yaml, 'safe_load', lambda text: dict(RUBRIC))
    monkeypatch.setattr(mission.Path, 'read_text', lambda self: '')
    result = mission.run(ctx)
    # ros2 launch exits 0 whatever its node's code, so the runner must be the child itself.
    assert started == [('task_gate', ['ros2', 'run', 'mhseals_nav', 'run_task', '--task', 'gate',
                                      '--ros-args', '-p', 'use_sim_time:=true'])]
    assert result['outcome'] == outcome and result['verdict'] == 'FAIL'
    assert result['exit_code'] == code and recorded == [result]


def test_a_sim_mission_refuses_to_start_out_of_sight_of_the_course(tmp_path, monkeypatch):
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
    recorded, order = [], []
    ctx = SimpleNamespace(services={'task': 'nav_channel'}, profile=profile, record=recorded.append,
                          history=lambda name: at(11.1, -102.8)(),
                          prompt=lambda *a: order.append('prompt'))
    monkeypatch.setattr(mission, 'reset_to_start', lambda ctx: order.append('reset') or '')
    result = mission.run(ctx)
    assert result == recorded[0] and result['verdict'] == 'REFUSED' and result['task'] == 'nav_channel'
    assert 'from the nearest course element' in result['reason']
    assert order == ['prompt', 'reset']      # checked where the boat is after the reset


def _reset_ctx(tmp_path, *, sim=True, at=(-2.3, -4.8, 0.5), reset='ok', ground_truth='/crane/odom',
               stale_during_call=False):
    """A mission context on a course whose truth starts at (-2.3, -4.8, yaw 0.5).

    stale_during_call: a pre-teleport (drifted) sample arrives while the reset call runs, and the
    post-teleport sample only on a later poll."""
    import json
    import time
    from types import SimpleNamespace
    truth = tmp_path / 'course_truth.json'
    truth.write_text(json.dumps({'startX': -2.3, 'startY': -4.8, 'startYaw': 0.5, 'elements': [
        {'label': 'red_pole_buoy', 'x': 4.8, 'y': -3.3}]}))
    profile = SimpleNamespace(course_truth=str(truth), ground_truth=ground_truth, use_sim_time=sim)
    calls, recorded, order, during, polls = [], [], [], [], []

    def call(timeout):
        calls.append(timeout)
        order.append('reset')
        if stale_during_call:
            during.append((time.monotonic(), {'x': 90.0, 'y': 40.0, 'yaw': 2.0}))
            time.sleep(0.01)
        if isinstance(reset, Exception):
            raise reset
        return reset == 'ok', 'x=-2.3 y=-4.8 yaw=0.5' if reset == 'ok' else 'no boat body to reset'

    def history(since=float('-inf'), until=float('inf'), name='odometry'):
        assert name == 'ground truth'
        stale = (since - 1.0, {'x': 90.0, 'y': 40.0, 'yaw': 2.0})   # before the reset: ignored
        polls.append(since)
        if at is None or (stale_during_call and len(polls) < 2):
            return [stale] + during
        return [stale] + during + [(time.monotonic(), dict(zip('xy', at), yaw=at[2]))]
    services = {'task': 'nav_channel'}
    if reset is not None:
        services['reset_to_start'] = call
    ctx = SimpleNamespace(services=services, profile=profile, record=recorded.append,
                          history=history, prompt=lambda *a: order.append('prompt'), order=order)
    return ctx, calls, recorded


def _settle_fast(tmp_path):
    config = tmp_path / 'start_reset.yaml'
    config.write_text('service: /crane/reset_to_start\nposition_m: 2.0\nyaw_deg: 15.0\n'
                      'timeout_s: 5.0\nsettle_s: 0.1\n')
    return config


def test_a_sim_mission_resets_the_boat_to_the_course_start_and_verifies_it(tmp_path):
    from mhseals_testing.procedures import mission
    ctx, calls, _ = _reset_ctx(tmp_path, at=(-1.0, -4.0, 0.6))     # 1.5 m and 6 deg: inside
    assert mission.reset_to_start(ctx) == ''
    assert calls == [5.0]


def test_the_reset_runs_after_the_operator_prompt_right_before_the_task_starts(tmp_path, monkeypatch):
    # A long wait at the prompt let the idle sim boat drift off the start it was reset to.
    from contextlib import contextmanager
    from types import SimpleNamespace
    from mhseals_testing.procedures import mission
    ctx, calls, recorded = _reset_ctx(tmp_path)
    runner = SimpleNamespace(paused=0.0, supervise=lambda *a, **k: (0.0, 5.0, True))

    @contextmanager
    def supervised(*args, **kwargs):
        yield runner
    watcher = SimpleNamespace(events=[], statuses=[], status={}, reset=lambda: ctx.order.append('watch'))
    ctx.supervise = supervised
    ctx.services.update({'task_watcher': lambda: watcher, 'running': lambda *a: False,
                         'start': lambda *a: ctx.order.append('start'), 'stop': lambda *a: None})
    monkeypatch.setattr(mission.reset_to_start, '__defaults__', (_settle_fast(tmp_path),))
    monkeypatch.setattr(mission, 'judge', lambda *a, **k: None)
    result = mission.run(ctx)
    assert calls == [5.0] and result['verdict'] != 'REFUSED'
    assert ctx.order == ['prompt', 'reset', 'watch', 'start']


@pytest.mark.parametrize('at,said', [
    ((2.7, -4.8, 0.5), '5.0 m and 0 deg off the course start'),    # still drifted
    ((-2.3, -4.8, 1.0), '0.0 m and 29 deg off the course start'),  # turned
])
def test_a_sim_mission_refuses_with_the_offset_left_after_the_reset(tmp_path, at, said):
    from mhseals_testing.procedures import mission
    ctx, calls, recorded = _reset_ctx(tmp_path, at=at)
    result = mission.run(ctx)
    assert calls and result == recorded[0] and result['verdict'] == 'REFUSED'
    assert ctx.order == ['prompt', 'reset']   # the operator's wait comes first, then the reset
    assert said in result['reason'] and 'limits 2 m and 15 deg' in result['reason']


def test_the_reset_is_never_called_on_a_stack_that_is_not_a_verified_sim(tmp_path):
    from mhseals_testing.procedures import mission
    for kwargs in (dict(sim=False), dict(ground_truth=None)):
        ctx, calls, _ = _reset_ctx(tmp_path, **kwargs)
        assert mission.reset_to_start(ctx) == '' and calls == []
    ctx, calls, _ = _reset_ctx(tmp_path)
    ctx.profile.use_sim_time = 1                       # truthy is not enough: only True is a sim
    assert mission.reset_to_start(ctx) == '' and calls == []


@pytest.mark.parametrize('reset,said', [
    (None, 'this session has no /crane/reset_to_start client'),
    (TimeoutError('/crane/reset_to_start is not available'), 'is not available'),
    ('refused', 'refused (no boat body to reset)'),
])
def test_a_missing_or_failed_reset_service_refuses_the_mission_clearly(tmp_path, reset, said):
    from mhseals_testing.procedures import mission
    ctx, _, recorded = _reset_ctx(tmp_path, reset=reset)
    result = mission.run(ctx)
    assert result['verdict'] == 'REFUSED' and 'cannot reset to the course start' in result['reason']
    assert said in result['reason']


def test_a_reset_without_fresh_ground_truth_is_not_trusted(tmp_path):
    from mhseals_testing.procedures import mission
    ctx, calls, _ = _reset_ctx(tmp_path, at=None)
    reason = mission.reset_to_start(ctx, config_path=_settle_fast(tmp_path))
    assert calls and 'no ground truth arrived' in reason


def test_a_drifted_sample_received_during_the_reset_call_is_not_taken_as_fresh(tmp_path):
    # The pre-teleport pose arrived while the service ran; only a sample after it returns counts.
    from mhseals_testing.procedures import mission
    ctx, calls, _ = _reset_ctx(tmp_path, stale_during_call=True)
    assert mission.reset_to_start(ctx, config_path=_settle_fast(tmp_path)) == ''
    assert calls == [5.0]


def test_a_sim_profile_whose_course_truth_is_missing_refuses_the_reset(tmp_path):
    from mhseals_testing.procedures import mission
    ctx, calls, recorded = _reset_ctx(tmp_path)
    ctx.profile.course_truth = str(tmp_path / 'gone.json')
    result = mission.run(ctx)
    assert result == recorded[0] and result['verdict'] == 'REFUSED' and calls == []
    assert 'course truth' in result['reason'] and 'gone.json is missing' in result['reason']


def test_the_tasks_claimed_phases_are_kept_next_to_its_events(tmp_path):
    # The referee sees what the boat said it was doing: every /task/status, on the track's clock.
    import json
    from types import SimpleNamespace
    from mhseals_testing.procedures import mission
    truth = tmp_path / 'course_truth.json'
    truth.write_text(json.dumps({'startX': 0.0, 'startY': 0.0, 'elements': []}))
    session = tmp_path / 'session'
    session.mkdir()
    status = {'schema': 'mhseals.mission_status/1', 'task': 'nav_channel', 'status': 'RUNNING'}
    ctx = SimpleNamespace(profile=SimpleNamespace(course_truth=str(truth), ground_truth='/crane/odom'),
                          services={'session_dir': str(session)},
                          history=lambda *a, **k: [(1.0, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})])
    statuses = [dict(status, phase='gate', received=1.0), dict(status, phase='channel', received=2.0)]
    mission.judge(ctx, 'nav_channel', RUBRIC, [dict(event='task_started', received=1.0)], 0.0, 3.0,
                  statuses=statuses)
    rows = [json.loads(line) for line in (session / 'task_status.jsonl').read_text().splitlines()]
    assert [(r['phase'], r['time'], r['task']) for r in rows] == [('gate', 1.0, 'nav_channel'),
                                                                 ('channel', 2.0, 'nav_channel')]
    assert all(r['schema'] == 'mhseals.mission_status/1' for r in rows)
    assert (session / 'task_events.jsonl').exists()
