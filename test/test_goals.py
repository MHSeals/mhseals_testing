"""Goal gates: each gate refuses with its reason; goals are attributed to the claim."""

import numpy as np
import pytest

from mhseals_testing.harness.goals import REQUIRED, Facts, Goals, Grid, check_goal, gate


def ready(**over):
    base = dict(live=True, owner=True, nav_running=True, lifecycle={s: 'active' for s in REQUIRED},
                unit='velocity', armed=True, driving='', pose_age_s=0.1, fix=4, yaw_quality='ok',
                costmap_age_s=1.0)
    return Facts(**{**base, **over})


@pytest.mark.parametrize('over, reason', [
    (dict(live=False), 'not live'),
    (dict(owner=False), 'does not own'),
    (dict(nav_running=False), 'Navigation is not running'),
    (dict(lifecycle={s: 'active' for s in REQUIRED[1:]}), 'bt_navigator'),
    (dict(lifecycle={**{s: 'active' for s in REQUIRED}, 'velocity_smoother': 'inactive'}), 'velocity_smoother'),
    (dict(unit='effort'), 'velocity controller'),
    (dict(armed=False), 'not armed'),
    (dict(driving='manual'), 'manual is driving'),
    (dict(pose_age_s=0.6), 'pose is stale'),
    (dict(fix=2), 'below 3D'),
    (dict(yaw_quality='fail'), 'yaw quality'),
    (dict(costmap_age_s=3.5), 'costmap is stale'),
])
def test_each_gate_refuses_with_its_reason(over, reason):
    assert reason in gate(ready(**over))


def test_the_costmap_limit_is_the_nav_watchdogs_and_unreadable_fails_closed(monkeypatch):
    from mhseals_testing.harness import goals
    limit = goals.costmap_limit_s()
    if limit is not None:                          # mhseals_nav is installed or in the source tree
        assert gate(ready(costmap_age_s=limit)) is None and 'stale' in gate(ready(costmap_age_s=limit + 0.1))
    monkeypatch.setattr(goals, 'costmap_limit_s', lambda: None)
    assert 'unreadable' in gate(ready(costmap_age_s=0.0))


def test_unit_unset_fails_closed():
    assert 'velocity controller' in gate(ready(unit=''))


def test_all_gates_pass_and_auto_ignores_freshness():
    assert gate(ready()) is None
    assert gate(ready(pose_age_s=9, fix=0, costmap_age_s=9), auto=True) is None


def grid(fill=0):
    return Grid(np.full((100, 100), fill, dtype=np.int8), 0.4, (-20.0, -20.0))  # 40 m square, half 20


@pytest.mark.parametrize('goal, reason', [
    ((30, 0), 'outside the costmap'),
    ((19.5, 0), 'limit'),
    ((0.2, 0), 'under'),
    ((float('nan'), 0), 'finite'),
])
def test_goal_checks(goal, reason):
    assert reason in check_goal(goal, (0, 0), grid())


@pytest.mark.parametrize('value', [99, 100])
def test_lethal_and_inscribed_neighbours_refuse(value):
    g = grid()
    row, col = g.cell(5.0, 0.0)
    g.data[row + 1, col - 1] = value  # a corner of the 3x3 window
    assert 'lethal' in check_goal((5.0, 0.0), (0, 0), g)
    g.data[row + 1, col - 1] = 98
    assert check_goal((5.0, 0.0), (0, 0), g) is None


class FakeGoal:
    done, status = False, 'RUNNING'

    def cancel(self):
        self.canceled = True


class FakeNav2:
    def __init__(self):
        self.sent, self.handle = [], FakeGoal()

    def plan(self, goal, timeout):
        return [(0, 0), (5, 0)]

    def navigate(self, goals):
        self.sent.append(goals)
        return self.handle


def make():
    log, nav2 = [], FakeNav2()
    return Goals(nav2, lambda phase, rec: log.append((phase, rec)), lambda: 7), nav2, log


def test_send_is_attributed_and_rechecks_gates():
    goals, nav2, log = make()
    assert 'not armed' in goals.send((5, 0, 0), (0, 0), grid(), ready(armed=False), 'c3', 5.0)
    assert not nav2.sent
    assert goals.send((5, 0, 0), (0, 0), grid(), ready(), 'c3', 5.0) is None
    assert log[0][0] == 'cursor_goal_start' and log[0][1]['claim'] == 'c3' and log[0][1]['frame'] == 'map'
    assert 'already active' in goals.send((6, 0, 0), (0, 0), grid(), ready(), 'c3', 6.0)


def test_cancel_and_gate_loss_journal_stop():
    goals, nav2, log = make()
    goals.send((5, 0, 0), (0, 0), grid(), ready(), 'c3', 5.0)
    goals.poll(ready(armed=False))  # disarm cancels
    assert nav2.handle.canceled
    stop = log[-1]
    assert stop[0] == 'cursor_goal_stop' and stop[1]['claim'] == 'c3' and 'not armed' in stop[1]['cause']
    assert goals.cancel('x') is False


def test_finished_goal_journals_outcome():
    goals, nav2, log = make()
    goals.send((5, 0, 0), (0, 0), grid(), ready(), 'c3', 5.0)
    nav2.handle.done, nav2.handle.status = True, 'SUCCEEDED'
    goals.poll(ready())
    assert log[-1][1]['outcome'] == 'succeeded' and goals.active is None


def test_preview_refuses_on_gate_and_planner_failure():
    goals, nav2, _ = make()
    assert goals.preview((5, 0), (0, 0), grid(), ready(live=False))[1]
    assert goals.preview((5, 0), (0, 0), grid(), ready())[0] == [(0, 0), (5, 0)]
    nav2.plan = lambda goal, timeout: (_ for _ in ()).throw(RuntimeError('no route'))
    assert 'no route' in goals.preview((5, 0), (0, 0), grid(), ready())[1]
