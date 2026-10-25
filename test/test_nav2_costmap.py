"""The goal costmap is the costmap node's current grid, judged by its stamp (agent-driving D2).

The local costmap topic sends a full grid only when a subscriber is there as the window moves, so
a late subscriber got the grid latched at startup: every goal more than ~4 m from the start was
"outside the costmap", and the lethal check near the start read a stale grid.
"""

import math
import types

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time as Stamp  # noqa: E402
from nav2_msgs.msg import Costmap  # noqa: E402
from nav2_msgs.srv import GetCostmap  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.task import Future  # noqa: E402

from mhseals_testing.harness import goals  # noqa: E402
from mhseals_testing.ros import nav2 as nav2_module  # noqa: E402
from mhseals_testing.ros.nav2 import Nav2  # noqa: E402
from mhseals_testing.ui import agent_acts  # noqa: E402

NOW_S = 400.0      # the node's (sim) clock


class Clock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=int(NOW_S * 1e9))


class Client:
    """A GetCostmap client that answers at once with ``costmap``, or is not there."""

    def __init__(self, costmap=None):
        self.costmap = costmap

    def wait_for_service(self, timeout_sec):
        return self.costmap is not None

    def call_async(self, request):
        assert isinstance(request, GetCostmap.Request)
        future = Future()
        future.set_result(GetCostmap.Response(map=self.costmap))
        return future


def stamp(seconds):
    return Stamp(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


def served(center, at=NOW_S, lethal=()):
    """A 12 m rolling window around ``center`` as GetCostmap returns it (costs 0-255)."""
    data = np.zeros((240, 240), dtype=np.uint8)
    costmap = Costmap()
    costmap.header.stamp, costmap.header.frame_id = stamp(at), 'odom'
    meta = costmap.metadata
    meta.resolution, meta.size_x, meta.size_y = 0.05, 240, 240
    meta.origin.position.x, meta.origin.position.y = center[0] - 5.95, center[1] - 5.95
    for x, y in lethal:
        data[int((y - meta.origin.position.y) / 0.05), int((x - meta.origin.position.x) / 0.05)] = 254
    costmap.data = data.tobytes()
    return costmap


def nav(client=Client()):
    found = Nav2.__new__(Nav2)
    found.node = types.SimpleNamespace(get_clock=Clock)
    found.costmap_client, found.costmap, found.costmap_time = client, None, None
    found.costmap_topic = '/local_costmap/costmap'
    return found


def check(found, goal, boat):
    """The goal gates' reason (or None) on the grid ``found`` fetches now."""
    cells = agent_acts.grid(found.fetch_costmap(timeout=0.2))
    facts = goals.Facts(live=True, owner=True, nav_running=True, armed=True, unit='velocity',
                        lifecycle={s: 'active' for s in goals.REQUIRED}, pose_age_s=0.0, fix=3,
                        costmap_age_s=agent_acts.costmap_age(found))
    return goals.gate(facts) or goals.check_goal(goal, boat, cells)


@pytest.fixture(autouse=True)
def limit(monkeypatch):
    monkeypatch.setattr(goals, 'costmap_limit_s', lambda: 1.0)


def test_costs_translate_as_nav2_publishes_them():
    assert nav2_module.occupancy(bytes([0, 1, 128, 252, 253, 254, 255])).tolist() == [0, 1, 50, 98, 99, 100, -1]


def test_a_goal_far_from_the_start_is_accepted_on_the_current_grid():
    # the boat drove to (14, 0); the served grid is centred on it, so a goal 3 m ahead is inside
    assert check(nav(Client(served((14.0, 0.0)))), (17.0, 0.0), (14.0, 0.0)) is None


def test_the_lethal_check_reads_the_current_grid():
    found = nav(Client(served((14.0, 0.0), lethal=[(17.0, 0.0)])))
    assert 'lethal' in check(found, (17.0, 0.0), (14.0, 0.0))
    assert check(found, (16.0, 2.0), (14.0, 0.0)) is None


def test_a_stale_served_grid_still_refuses():
    assert 'stale' in check(nav(Client(served((14.0, 0.0), at=NOW_S - 5.0))), (17.0, 0.0), (14.0, 0.0))


def test_a_latched_grid_is_judged_by_its_stamp_not_its_arrival():
    found = nav()                          # no GetCostmap service: the topic's latched grid
    grid = OccupancyGrid()
    grid.header.stamp = stamp(3.0)         # the startup grid, received just now
    found._costmap(grid)
    assert agent_acts.costmap_age(found) == pytest.approx(NOW_S - 3.0, abs=0.5)
    fresh = OccupancyGrid()
    fresh.header.stamp = stamp(NOW_S - 0.2)
    found._costmap(fresh)
    assert agent_acts.costmap_age(found) < 1.0


def test_the_served_grid_keeps_origin_resolution_and_shape():
    grid = nav2_module.grid_of(served((14.0, -3.0)))
    cells = agent_acts.grid(grid)
    assert cells.data.shape == (240, 240) and cells.resolution == pytest.approx(0.05)
    assert cells.origin == pytest.approx((8.05, -8.95)) and math.isclose(cells.half_width(), 6.0, rel_tol=1e-6)


def test_the_fence_message_is_kept_with_its_receipt_age_and_a_bad_one_is_invalid():
    from std_msgs.msg import String
    found = nav()
    found.fence, found.fence_time = None, None
    fence, age = found.fence_state(timeout=0.05)          # none came: waits once, then never again
    assert fence is None and age == math.inf and found.fence_waited
    found._fence(String(data='{"valid": true, "points": [0, 0, 1, 0, 1, 1]}'))
    fence, age = found.fence_state()
    assert fence['valid'] and age < 1.0
    found._fence(String(data='not json'))
    assert found.fence_state()[0] == {'valid': False, 'error': 'unreadable fence message'}
