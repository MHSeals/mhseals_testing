"""Mission rehearsal: run one of mhseals_nav's task trees and score it.

The task runs as a supervised child (ros2 run mhseals_nav run_task); the suite only watches,
with the interlocks live, and stops the task on any abort. Scoring:
analysis/mission.py against config/missions/<task>.yaml, from what the task
reports. In the simulator the referee (analysis/referee.py) also judges what
the boat really did, from the course truth and the true track.
"""

import json
from pathlib import Path

import yaml

from mhseals_testing.analysis import referee
from mhseals_testing.analysis.mission import score
from mhseals_testing.profiles import PROFILE_DIR

MISSION_DIR = PROFILE_DIR.parent / 'missions'
MARGIN = 30.0  # s past the rubric's time limit before the suite gives up
# run_task's exit codes (mhseals_nav tasks/interface.py); 3, a human override, is newer.
# Any other code is a FAILURE.
TASK_EXIT = {0: 'SUCCESS', 1: 'FAILURE', 2: 'TIMEOUT', 3: 'PREEMPTED'}


def available(directory=MISSION_DIR):
    return sorted(path.stem for path in Path(directory).glob('*.yaml'))


def run(ctx):
    task = ctx.choose('Mission', tuple(available()))
    rubric = yaml.safe_load((MISSION_DIR / f'{task}.yaml').read_text())
    services = ctx.services
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
    return referee.judge_task(task, truth, track, events, rubric)
