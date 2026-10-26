"""Mission rehearsal: run one of mhseals_nav's task trees and score it.

The task runs as a supervised child (ros2 run mhseals_nav run_task); the suite only watches,
with the interlocks live, and stops the task on any abort. Scoring:
analysis/mission.py against config/missions/<task>.yaml, from what the task
reports. In the simulator the referee (analysis/referee.py) also judges what
the boat really did, from the course truth and the true track.
"""

import json
import math
import time
from pathlib import Path

import yaml

from mhseals_testing import data
from mhseals_testing.analysis import referee
from mhseals_testing.analysis.mission import score
from mhseals_testing.profiles import PROFILE_DIR

MISSION_DIR = PROFILE_DIR.parent / 'missions'
START_RESET = PROFILE_DIR.parent / 'sim' / 'start_reset.yaml'
MARGIN = 30.0  # s past the rubric's time limit before the suite gives up
# run_task's exit codes (mhseals_nav tasks/interface.py). Newer: 3, a human override stopped
# it; 4, it refused to start while a human had control. Any other code is a FAILURE.
TASK_EXIT = {0: 'SUCCESS', 1: 'FAILURE', 2: 'TIMEOUT', 3: 'PREEMPTED', 4: 'REFUSED'}
# crane's camera (BoundingBox3D maxDist, sim_backend fov.range_m) sees 20 m: farther from every
# course element, a sim mission searches empty water.
SIM_SIGHT_M = 20.0


def available(directory=MISSION_DIR):
    return sorted(path.stem for path in Path(directory).glob('*.yaml'))


def off_course(profile, history):
    """Return why a sim mission must not start where the boat is, or ''.

    The simulator keeps running across stack sessions, and each session's odometry starts again at
    (0, 0): a boat driven or drifted away in an earlier session believes it is at the start while
    the course is out of its camera's sight. Judged in the ground-truth frame of the course truth,
    so it applies only to a profile with both; without a ground-truth sample it does not block.
    """
    path = getattr(profile, 'course_truth', None)
    if not getattr(profile, 'use_sim_time', False) or not path or not getattr(profile, 'ground_truth', None) or not Path(path).exists():
        return ''
    try:
        elements = [(float(e['x']), float(e['y']))
                    for e in json.loads(Path(path).read_text())['elements'] if e.get('label')]
    except (OSError, ValueError, KeyError, TypeError):
        return ''
    samples = [v for _, v in history() if v.get('x') is not None]
    if not elements or not samples:
        return ''
    x, y = samples[-1]['x'], samples[-1]['y']
    nearest = min(math.hypot(ex - x, ey - y) for ex, ey in elements)
    if nearest <= SIM_SIGHT_M:
        return ''
    return (f'the boat is {nearest:.0f} m from the nearest course element (ground truth '
            f'{x:.1f}, {y:.1f}); restart the simulator so the boat starts on the course')


def sim_stack(profile):
    """True only for a simulator profile with ground truth and a course truth on disk: the
    same evidence off_course needs. Anything else is treated as the real boat."""
    path = getattr(profile, 'course_truth', None)
    return (getattr(profile, 'use_sim_time', False) is True and bool(path)
            and bool(getattr(profile, 'ground_truth', None)) and Path(path).exists())


def reset_to_start(ctx, config_path=START_RESET):
    """Put a sim boat back on the course start and verify it there; return why not, or ''.

    The idle sim boat drifts with the current before a mission (one demo started 99 m off and
    scored 0). Only a verified sim stack (sim_stack) is reset; on anything else this never
    calls the service and returns ''. A sim profile that names a course truth missing on disk is
    refused, not skipped. The pose is checked against the course truth's start from a
    ground-truth sample taken strictly after the reset returned, not from the simulator's own
    reply: a sample from before the teleport, even one received during the call, is not counted.
    """
    profile = ctx.profile
    path = getattr(profile, 'course_truth', None)
    if (getattr(profile, 'use_sim_time', False) is True and getattr(profile, 'ground_truth', None)
            and path and not Path(path).exists()):
        return (f'cannot reset to the course start: the course truth {path} is missing; '
                'restore it or fix the profile')
    if not sim_stack(profile):
        return ''
    config = yaml.safe_load(Path(config_path).read_text())
    service = config['service']
    try:
        truth = json.loads(Path(ctx.profile.course_truth).read_text())
        sx, sy, syaw = float(truth['startX']), float(truth['startY']), float(truth['startYaw'])
    except (OSError, ValueError, KeyError, TypeError) as error:
        return f'cannot reset to the course start: the course truth has no start ({error})'
    reset = ctx.services.get('reset_to_start')
    if reset is None:
        return f'cannot reset to the course start: this session has no {service} client'
    clock = getattr(getattr(ctx, 'monitor', None), 'clock', time.monotonic)
    try:
        ok, message = reset(float(config['timeout_s']))
    except Exception as error:  # noqa: BLE001  (TimeoutError, or rclpy's own)
        return (f'cannot reset to the course start: {service} failed ({error}); '
                'restart the simulator with a crane_ml that has it')
    if not ok:
        return f'cannot reset to the course start: {service} refused ({message})'
    began = clock()   # after the teleport: a sample received during the call predates it
    deadline = time.monotonic() + float(config['settle_s'])
    while True:
        samples = [v for t, v in ctx.history(since=began, name='ground truth')
                   if t > began and v.get('x') is not None]
        if samples or time.monotonic() >= deadline:
            break
        time.sleep(0.05)
    if not samples:
        return (f'{service} answered ({message}) but no ground truth arrived within '
                f'{config["settle_s"]:g}s to confirm the boat is on the start')
    x, y, yaw = samples[-1]['x'], samples[-1]['y'], samples[-1].get('yaw')
    distance = math.hypot(x - sx, y - sy)
    turn = (None if yaw is None else
            abs(math.degrees(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))))
    if distance <= float(config['position_m']) and turn is not None and turn <= float(config['yaw_deg']):
        return ''
    heading = 'unknown' if turn is None else f'{turn:.0f} deg'
    return (f'after {service} the boat is {distance:.1f} m and {heading} off the course start '
            f'(ground truth {x:.1f}, {y:.1f}; start {sx:.1f}, {sy:.1f}); limits '
            f'{config["position_m"]:g} m and {config["yaw_deg"]:g} deg')


def refuse(ctx, task, reason):
    """Record and return the mission's refusal to start."""
    refusal = dict(procedure='mission', task=task, verdict='REFUSED', reason=reason)
    ctx.record(refusal)
    return refusal


def run(ctx):
    task = ctx.services.get('task') or ctx.choose('Mission', tuple(available()))   # an agent's run names it
    rubric = yaml.safe_load((MISSION_DIR / f'{task}.yaml').read_text())
    services = ctx.services
    ctx.prompt(f'Rehearse {task}: Nav2 drives the course from here. '
               'Clear the area and press Enter... ')
    # After the prompt: the idle sim boat drifts while the operator waits.
    unreset = reset_to_start(ctx)
    if unreset:
        return refuse(ctx, task, unreset)
    away = off_course(ctx.profile, lambda: ctx.history(name='ground truth'))
    if away:
        return refuse(ctx, task, away)
    watcher = services['task_watcher']()
    watcher.reset()
    # Run-level fields for the run record (the App merges them into runs/<run_id>.json's
    # ``result``, aborted runs included): the official limit from the mission config.
    fields = services.setdefault('run_fields', {})
    fields['time_limit_s'] = rubric.get('time_limit')
    # run_task directly: ros2 launch exits 0 whatever its node's code.
    command = ['ros2', 'run', 'mhseals_nav', 'run_task', '--task', task, '--ros-args',
               '-p', f'use_sim_time:={str(ctx.profile.use_sim_time).lower()}']
    with ctx.supervise('mission', task=task) as runner:
        paused_before = runner.paused
        # The same instant on both clocks: the track's t and the wall clock (ns) that the
        # session journal's horn, holds and takeovers carry, so the referee maps one onto the other.
        fields['run_start_t'] = runner.clock()   # the clock of ctx.history, so of track.jsonl's t
        fields['run_start_t_ns'] = time.time_ns()
        services['start'](f'task_{task}', command)
        try:
            start, end, _ = runner.supervise(
                rubric.get('time_limit', 300.0) + MARGIN,
                done=lambda: not services['running'](f'task_{task}'),
                label=lambda: f"{task}: {watcher.status.get('tip') or 'starting'}")
        finally:
            # Read before the stop: a SIGINT from here would replace the task's own code.
            code = services.get('returncode', lambda name: None)(f'task_{task}')
            services['stop'](f'task_{task}')  # SIGINT: the task cancels its Nav2 goal
    paused = runner.paused - paused_before
    # The stream's task_finished wins; the exit code speaks only for a stream without one.
    exit_outcome = None if code is None else TASK_EXIT.get(code, 'FAILURE')
    result = dict(score(watcher.events, rubric, outcome=exit_outcome), procedure='mission',
                  task=task, paused_s=paused, exit_code=code, exit_outcome=exit_outcome)
    # The referee reads the wall-clock track; supervise's end excludes the paused time.
    judged = judge(ctx, task, rubric, watcher.events, start, end + paused, statuses=watcher.statuses)
    if judged is not None:
        result['referee'] = judged['verdict']
        result['referee_detail'] = judged
    ctx.record(result)
    return result


def judge(ctx, task, rubric, events, start, end, statuses=()):
    """The referee's view, or an explicit reason it could not judge the run."""
    path = ctx.profile.course_truth
    if not path or not ctx.profile.ground_truth or not Path(path).exists():
        return dict(verdict='N/A', reason='course truth or ground-truth track is unavailable')
    truth = json.loads(Path(path).read_text())
    samples = ctx.history(start, end, name='ground truth')
    track = [(t, v['x'], v['y'], v.get('yaw') or 0.0) for t, v in samples
             if v.get('x') is not None]
    events = [dict(e, time=e.get('received', e.get('time', 0.0))) for e in events]
    statuses = [dict(s, time=s.get('received', 0.0)) for s in statuses]
    # The run record links the run's own files (relative to the session directory).
    ctx.services.setdefault('run_fields', {})['files'] = keep(
        ctx.services.get('session_dir'), task, track, events, statuses, run_id=ctx.services.get('run_id'))
    return referee.judge_task(task, truth, track, events, rubric)


def keep(session, task, track, events, statuses=(), run_id=None):
    """Write what the referee judged into the session, for ``boat_trial analyze referee``.

    ``track.jsonl`` (rows ``{t, x, y, yaw}``), ``task_events.jsonl`` and ``task_status.jsonl``
    (every /task/status message, mhseals.mission_status/1; each row naming ``task``) hold the
    latest mission run; with a ``run_id`` the run's own copies go to ``runs/<run_id>/``, which a
    later run never overwrites. Returns those files' paths relative to the session directory
    (``track``, ``task_events``, ``task_status``; empty without a session directory).
    """
    if session is None:
        return {}
    lines = lambda rows: ''.join(json.dumps(row) + '\n' for row in rows)
    texts = {'track.jsonl': lines(dict(t=t, x=x, y=y, yaw=yaw) for t, x, y, yaw in track),
             'task_events.jsonl': lines(dict(e, task=task) for e in events),
             'task_status.jsonl': lines(dict(s, task=task) for s in statuses)}
    for name, text in texts.items():
        data.atomic_write(Path(session) / name, text)
    where = Path('runs', run_id) if run_id else Path()
    if run_id:
        for name, text in texts.items():
            data.atomic_write(Path(session) / where / name, text)
    return {name.removesuffix('.jsonl'): str(where / name) for name in texts}
