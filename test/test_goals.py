"""Goal gates: each gate refuses with its reason; goals are attributed to the claim."""

import math

import numpy as np
import pytest

from mhseals_testing.harness import goals as goals_module
from mhseals_testing.harness.goals import REQUIRED, Facts, Goals, Grid, check_fence, check_goal, gate

BOX = [-40.0, -40.0, 40.0, -40.0, 40.0, 40.0, -40.0, 40.0]   # map metres, as fence_watchdog publishes it
FENCE = {'valid': True, 'frame': 'map', 'points': BOX, 'margin_m': 5.0,
         'outward_angle_deg': 60.0, 'outward_speed_mps': 0.1, 'reason': '', 'error': '', 'stamp_ns': 1}


@pytest.fixture(autouse=True)
def fence_topic(monkeypatch):
    """The nav side's fence_topic and 3 periods of fence_rate_hz, whatever mhseals_nav is at hand."""
    monkeypatch.setattr(goals_module, 'fence_config', lambda: ('/fence_watchdog/fence', 3.0))


def ready(**over):
    base = dict(live=True, owner=True, nav_running=True, lifecycle={s: 'active' for s in REQUIRED},
                unit='velocity', armed=True, driving='', pose_age_s=0.1, fix=4, yaw_quality='ok',
                costmap_age_s=1.0, fence=FENCE, fence_age_s=0.5)
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
    ((1e308, 0), 'outside the costmap'),      # would overflow the cell index
    ((0, -1e308), 'outside the costmap'),
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
    assert 'not armed' in goals.send((5, 0, 0), (0, 0, 0), grid(), ready(armed=False), 'c3', 5.0)
    assert not nav2.sent
    assert goals.send((5, 0, 0), (0, 0, 0), grid(), ready(), 'c3', 5.0) is None
    assert log[0][0] == 'cursor_goal_start' and log[0][1]['claim'] == 'c3' and log[0][1]['frame'] == 'map'
    assert 'already active' in goals.send((6, 0, 0), (0, 0, 0), grid(), ready(), 'c3', 6.0)


def test_cancel_and_gate_loss_journal_stop():
    goals, nav2, log = make()
    goals.send((5, 0, 0), (0, 0, 0), grid(), ready(), 'c3', 5.0)
    goals.poll(ready(armed=False))  # disarm cancels
    assert nav2.handle.canceled
    stop = log[-1]
    assert stop[0] == 'cursor_goal_stop' and stop[1]['claim'] == 'c3' and 'not armed' in stop[1]['cause']
    assert goals.cancel('x') is False


def test_finished_goal_journals_outcome():
    goals, nav2, log = make()
    goals.send((5, 0, 0), (0, 0, 0), grid(), ready(), 'c3', 5.0)
    nav2.handle.done, nav2.handle.status = True, 'SUCCEEDED'
    goals.poll(ready())
    assert log[-1][1]['outcome'] == 'succeeded' and goals.active is None


def test_preview_refuses_on_gate_and_planner_failure():
    goals, nav2, _ = make()
    assert goals.preview((5, 0), (0, 0, 0), grid(), ready(live=False))[1]
    assert goals.preview((5, 0), (0, 0, 0), grid(), ready())[0] == [(0, 0), (5, 0)]
    nav2.plan = lambda goal, timeout: (_ for _ in ()).throw(RuntimeError('no route'))
    assert 'no route' in goals.preview((5, 0), (0, 0, 0), grid(), ready())[1]


def test_an_unreadable_costmap_limit_is_read_again_on_the_next_goal(monkeypatch, tmp_path):
    from mhseals_testing.harness import goals
    from mhseals_testing.health import config
    monkeypatch.setattr(goals, '_LIMIT', [])
    monkeypatch.setattr(config, 'share_dir', lambda package: tmp_path / package)
    assert goals.costmap_limit_s() is None and 'unreadable' in gate(ready(costmap_age_s=0.0))
    path = tmp_path / 'mhseals_nav' / 'config' / 'costmap_watchdog.yaml'
    path.parent.mkdir(parents=True)
    path.write_text("/**:\n  ros__parameters:\n    max_age_s: .nan\n")
    assert goals.costmap_limit_s() is None
    path.write_text("/**:\n  ros__parameters:\n    max_age_s: 1.0\n")
    assert goals.costmap_limit_s() == 1.0 and gate(ready(costmap_age_s=1.0)) is None
    assert 'stale' in gate(ready(costmap_age_s=1.1))


# --- the fence: fence_watchdog's own fence and margin rules (C9, C11) --------------------------

@pytest.mark.parametrize('goal, reason', [
    ((45.0, 0.0), 'outside the fence'),                 # past the fence
    ((37.0, 0.0), 'within the 5 m fence margin'),       # inside it, but in the margin
    ((-38.0, 39.0), 'within the 5 m fence margin'),     # a corner: both edges count
])
def test_a_goal_past_the_fence_or_in_its_margin_is_refused(goal, reason):
    assert reason in check_fence(goal, (30.0, 0.0, 0.0), FENCE, 0.5)


def test_preview_and_send_check_the_fence():
    small = dict(FENCE, points=[-8.0, -8.0, 8.0, -8.0, 8.0, 8.0, -8.0, 8.0])
    goals, nav2, _ = make()
    for goal in ((6.0, 0.0, 0.0), (9.0, 0.0, 0.0)):
        assert 'fence' in goals.preview(goal, (0, 0, 0), grid(), ready(fence=small))[1]
        assert 'fence' in goals.send(goal, (0, 0, 0), grid(), ready(fence=small), 'c3', 5.0)
    assert 'no fence' in goals.send((2.0, 0.0, 0.0), (0, 0, 0), grid(), ready(fence=None), 'c3', 2.0)
    assert not nav2.sent
    assert goals.send((2.0, 0.0, 0.0), (0, 0, 0), grid(), ready(fence=small), 'c3', 2.0) is None


def test_a_goal_well_inside_passes():
    assert check_fence((34.9, 0.0), (30.0, 0.0, 0.0), FENCE, 0.5) is None


@pytest.mark.parametrize('fence, age, reason', [
    (None, 0.5, 'no fence from fence_watchdog'),
    (FENCE, 3.1, 'no fence from fence_watchdog'),             # older than three periods
    (dict(FENCE, valid=False, points=None, error='no valid fence configured'), 0.5, 'no valid fence configured'),
    (dict(FENCE, points=BOX[:4]), 0.5, 'no valid fence'),
])
def test_a_missing_stale_or_invalid_fence_refuses(fence, age, reason):
    assert reason in check_fence((5.0, 0.0), (0.0, 0.0, 0.0), fence, age)


def test_the_fence_topic_is_the_nav_watchdogs_and_unreadable_refuses(monkeypatch, tmp_path):
    from mhseals_testing.health import config
    monkeypatch.undo()                                    # the real fence_config
    monkeypatch.setattr(goals_module, '_FENCE', [])
    monkeypatch.setattr(config, 'share_dir', lambda package: tmp_path / package)
    assert goals_module.fence_config() is None
    assert 'fence unknown' in check_fence((5.0, 0.0), (0.0, 0.0, 0.0), FENCE, 0.5)
    path = tmp_path / 'mhseals_nav' / 'config' / 'fence_watchdog.yaml'
    path.parent.mkdir(parents=True)
    path.write_text("/**:\n  ros__parameters:\n    fence_topic: /f\n    fence_rate_hz: 0.0\n")
    assert goals_module.fence_config() is None
    path.write_text("/**:\n  ros__parameters:\n    fence_topic: /f\n    fence_rate_hz: 2.0\n")
    assert goals_module.fence_config() == ('/f', 1.5)


def test_in_the_margin_facing_outward_any_goal_is_refused_with_the_way_out():
    boat = (37.0, 0.0, 0.0)                              # 3 m from the east edge, facing east
    reason = check_fence((33.0, 0.0), boat, FENCE, 0.5)
    assert 'facing outward' in reason and 'U' in reason and 'manual' in reason
    tilted = (37.0, 0.0, 1.10)                           # 63 deg from the outward normal: not outward
    assert check_fence((33.0, 2.0), tilted, FENCE, 0.5) is None


def test_in_the_margin_facing_inward_only_inward_goals_pass():
    boat = (37.0, 0.0, 3.14)                             # facing west, into the fence
    assert check_fence((33.5, 0.0), boat, FENCE, 0.5) is None          # inward, still in the margin
    assert check_fence((34.0, 3.0), boat, FENCE, 0.5) is None
    reason = check_fence((37.0, 3.0), boat, FENCE, 0.5)                # along the edge
    assert 'only goals farther inward' in reason
    assert 'only goals farther inward' in check_fence((38.0, 0.0), boat, FENCE, 0.5)


LAT0, LON0 = 42.0, -71.0
K = math.pi / 180.0 * 6371000.0                       # the watchdog's equirectangular metres per degree


def lat_lon(east, north):
    return LAT0 + north / K, LON0 + east / (K * math.cos(math.radians(LAT0)))


# A lat/lon course 100 m east by 50 m north from (LAT0, LON0), as the boat's fence file gives it.
LL_FENCE = dict(FENCE, frame='latlon', points=[v for p in ((0, 0), (100, 0), (100, 50), (0, 50))
                                               for v in lat_lon(*p)])


def rotated_to_ll(degrees, calls=None):
    """navsat toLL for a map rotated ``degrees`` from ENU, its origin at the course centre (50, 25)."""
    a = math.radians(degrees)

    def to_ll(points):
        if calls is not None:
            calls.append(list(points))
        return [lat_lon(50 + x * math.cos(a) - y * math.sin(a), 25 + x * math.sin(a) + y * math.cos(a))
                for x, y in points]
    return to_ll


def test_a_lat_lon_fence_is_checked_in_lat_lon_through_tol_l_on_a_rotated_map():
    """A goal a translated map copy would allow (map (40, 19): 6 m inside the north edge) lies past
    the edge once the 10 degree map rotation is applied, as toLL does: refused."""
    boat = (0.0, 0.0, 0.0)
    assert check_fence((40.0, 19.0), boat, LL_FENCE, 0.5, rotated_to_ll(0.0)) is None
    assert check_fence((40.0, 19.0), boat, LL_FENCE, 0.5, rotated_to_ll(10.0)) == 'goal is outside the fence'
    calls = []
    assert check_fence((10.0, 0.0), boat, LL_FENCE, 0.5, rotated_to_ll(10.0, calls)) is None
    assert calls == [[(10.0, 0.0), (0.0, 0.0), (1.0, 0.0)]]   # the goal, the boat, a point ahead


def map_of(degrees, east, north):
    """The map point a map rotated ``degrees`` (rotated_to_ll) puts at course ENU (east, north)."""
    a, e, n = math.radians(degrees), east - 50, north - 25
    return e * math.cos(a) + n * math.sin(a), -e * math.sin(a) + n * math.cos(a)


def test_a_lat_lon_fence_reads_the_heading_in_lat_lon_too():
    """Facing map east is facing 45 deg north of east on a map rotated 45 deg: 2.6 m from the
    north edge that is within the 60 deg outward rule, so refused; rotated -45 it faces away."""
    for degrees, reason in ((45.0, 'facing outward'), (-45.0, None)):
        boat = (*map_of(degrees, 47.0, 47.4), 0.0)
        found = check_fence(map_of(degrees, 47.0, 40.0), boat, LL_FENCE, 0.5, rotated_to_ll(degrees))
        assert found is None if reason is None else reason in found


@pytest.mark.parametrize('to_ll, reason', [
    (None, 'no toLL'),
    (lambda points: (_ for _ in ()).throw(RuntimeError('/toLL is not available')), 'toLL failed'),
    (lambda points: [(0.0, 0.0)] * len(points), 'no datum'),
    (lambda points: [(float('nan'), 0.0)] * len(points), 'no datum'),
])
def test_a_lat_lon_fence_without_a_working_tol_l_refuses(to_ll, reason):
    assert reason in check_fence((5.0, 0.0), (0.0, 0.0, 0.0), LL_FENCE, 0.5, to_ll)


def test_a_map_fence_needs_no_tol_l():
    assert check_fence((5.0, 0.0), (0.0, 0.0, 0.0), FENCE, 0.5, None) is None


def test_the_boat_outside_the_fence_refuses_every_goal():
    assert 'boat is outside the fence' in check_fence((30.0, 0.0), (41.0, 0.0, 3.14), FENCE, 0.5)
