"""Mission rehearsal: run one of mhseals_nav's task trees and score it.

The task runs as a supervised child (task.launch.py); the suite only watches,
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
    command = ['ros2', 'launch', 'mhseals_nav', 'task.launch.py', f'task:={task}',
               f'sim:={str(ctx.profile.use_sim_time).lower()}']
    with ctx.supervise('mission', task=task) as runner:
        services['start'](f'task_{task}', command)
        try:
            start, end, _ = runner.supervise(
                rubric.get('time_limit', 300.0) + MARGIN,
                done=lambda: not services['running'](f'task_{task}'),
                label=lambda: f"{task}: {watcher.status.get('tip') or 'starting'}")
        finally:
            services['stop'](f'task_{task}')  # SIGINT: the task cancels its Nav2 goal
    result = dict(score(watcher.events, rubric), procedure='mission', task=task)
    judged = judge(ctx, task, rubric, watcher.events, start, end)
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
    options = rubric.get('referee', {})
    if task == 'mission':
        return referee.judge_mission(truth, track, events, rubric['tasks'], **options)
    return referee.judge(task, truth, track, events, **options)
