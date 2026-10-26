"""Mission rehearsal: run one of mhseals_nav's task trees and score it.

The task runs as a supervised child (ros2 run mhseals_nav run_task); the suite only watches,
with the interlocks live, and stops the task on any abort. Scoring:
analysis/mission.py against config/missions/<task>.yaml, from what the task
reports. In the simulator the referee (analysis/referee.py) also judges what
the boat really did, from the course truth and the true track.
"""

import json
import math
from pathlib import Path

import yaml

from mhseals_testing import data
from mhseals_testing.analysis import referee
from mhseals_testing.analysis.mission import score
from mhseals_testing.profiles import PROFILE_DIR

MISSION_DIR = PROFILE_DIR.parent / 'missions'
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


def refuse(ctx, task, reason):
    """Record and return the mission's refusal to start."""
    refusal = dict(procedure='mission', task=task, verdict='REFUSED', reason=reason)
    ctx.record(refusal)
    return refusal


def run(ctx):
    task = ctx.services.get('task') or ctx.choose('Mission', tuple(available()))   # an agent's run names it
    rubric = yaml.safe_load((MISSION_DIR / f'{task}.yaml').read_text())
    services = ctx.services
    away = off_course(ctx.profile, lambda: ctx.history(name='ground truth'))
    if away:
        return refuse(ctx, task, away)
    watcher = services['task_watcher']()
    watcher.reset()
    ctx.prompt(f'Rehearse {task}: Nav2 drives the course from here. '
               'Clear the area and press Enter... ')
    # run_task directly: ros2 launch exits 0 whatever its node's code.
    command = ['ros2', 'run', 'mhseals_nav', 'run_task', '--task', task, '--ros-args',
               '-p', f'use_sim_time:={str(ctx.profile.use_sim_time).lower()}']
    with ctx.supervise('mission', task=task) as runner:
        paused_before = runner.paused
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
    judged = judge(ctx, task, rubric, watcher.events, start, end + paused)
    if judged is not None:
        result['referee'] = judged['verdict']
        result['referee_detail'] = judged
    ctx.record(result)
    return result


def judge(ctx, task, rubric, events, start, end):
    """The referee's view, or an explicit reason it could not judge the run."""
    path = ctx.profile.course_truth
    if not path or not ctx.profile.ground_truth or not Path(path).exists():
        return dict(verdict='N/A', reason='course truth or ground-truth track is unavailable')
    truth = json.loads(Path(path).read_text())
    samples = ctx.history(start, end, name='ground truth')
    track = [(t, v['x'], v['y'], v.get('yaw') or 0.0) for t, v in samples
             if v.get('x') is not None]
    events = [dict(e, time=e.get('received', e.get('time', 0.0))) for e in events]
    keep(ctx.services.get('session_dir'), task, track, events)
    return referee.judge_task(task, truth, track, events, rubric)


def keep(session, task, track, events):
    """Write what the referee judged into the session, for ``boat_trial analyze referee``.

    ``track.jsonl`` (rows ``{t, x, y, yaw}``) and ``task_events.jsonl`` (each naming ``task``)
    hold the latest mission run; nothing is written without a session directory.
    """
    if session is None:
        return
    lines = lambda rows: ''.join(json.dumps(row) + '\n' for row in rows)
    data.atomic_write(Path(session) / 'track.jsonl',
                      lines(dict(t=t, x=x, y=y, yaw=yaw) for t, x, y, yaw in track))
    data.atomic_write(Path(session) / 'task_events.jsonl', lines(dict(e, task=task) for e in events))
