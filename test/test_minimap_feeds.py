"""Lazy costmap leases, the recorded costmap's crop and gate, the trail file and the rotation fact."""

import math
import types

import numpy as np
import pytest

from mhseals_testing import data
from mhseals_testing.analysis.geo import MapRotation
from mhseals_testing.ros import minimap_feeds as feeds


class FakeNode:
    def __init__(self):
        self.live = []

    def create_subscription(self, kind, topic, callback, qos):
        sub = (topic, callback)
        self.live.append(sub)
        return sub

    def destroy_subscription(self, sub):
        self.live.remove(sub)


def lazy(node):
    return feeds.LazySub(node, object, '/local_costmap/costmap', print, None)


def test_no_subscription_until_leased():
    node = FakeNode()
    sub = lazy(node)
    assert node.live == []
    sub.lease('layers')
    assert len(node.live) == 1


def test_subscription_dropped_when_last_lease_goes():
    node = FakeNode()
    sub = lazy(node)
    sub.lease('layers')
    sub.lease('bringup')
    sub.lease('layers', False)
    assert len(node.live) == 1          # bringup still holds it
    sub.lease('bringup', False)
    assert node.live == []
    sub.lease('layers', False)          # releasing twice is harmless
    assert node.live == []


def test_source_by_zoom_and_choice():
    assert feeds.costmap_source(5) == feeds.LOCAL
    assert feeds.costmap_source(20) == feeds.GLOBAL
    assert feeds.costmap_source(20, 'local') == feeds.LOCAL
    assert feeds.costmap_source(2, 'global') == feeds.GLOBAL


@pytest.mark.parametrize('owner, recording, nav, expected', [
    (True, True, True, True), (False, True, True, False),
    (True, False, True, False), (True, True, False, False)])
def test_recorder_gating_truth_table(owner, recording, nav, expected):
    assert feeds.should_record(owner, recording, nav) is expected


def test_decimate_keeps_lethal_and_crops_around_the_boat():
    grid = np.zeros((500, 500), dtype=np.int8)      # 100 m at 0.2 m, origin (-50, -50)
    grid[260, 301] = 100                            # world x = 10.2, y = 2.0
    grid[0, 0] = -1
    cells, size, origin = feeds.decimate(grid.ravel(), 500, 500, 0.2, (-50.0, -50.0), (0.0, 0.0))
    assert size == 200 and len(cells) == 200 * 200
    out = cells.reshape(size, size)
    assert out.max() == 100
    j, i = np.argwhere(out == 100)[0]
    assert origin[0] + i * 0.4 <= 10.2 < origin[0] + (i + 1) * 0.4
    assert origin[1] + j * 0.4 <= 2.0 < origin[1] + (j + 1) * 0.4
    assert origin == (-40.0, -40.0)


def test_decimate_marks_cells_beyond_the_source_unknown():
    grid = np.zeros(100 * 100, dtype=np.int8)       # 20 m square at 0.2 m
    cells, size, _ = feeds.decimate(grid, 100, 100, 0.2, (0.0, 0.0), (10.0, 10.0))
    out = cells.reshape(size, size)
    assert out[100, 100] == 0 and out[0, 0] == -1 and out[199, 199] == -1


def test_trail_schema_spacing_and_append(tmp_path):
    trail = feeds.TrailWriter(tmp_path)
    assert trail.add(0.0, 1.0, 2.0, fix=6, h_acc_m=0.02, speed_mps=0.3)
    assert not trail.add(0.5, 1.1, 2.0)             # too soon and too near
    assert trail.add(0.6, 2.0, 2.0, speed_mps=math.nan)      # moved 1 m
    assert trail.add(2.0, 2.0, 2.0)                 # 1 s elapsed
    assert not trail.add(9.0, math.nan, 0.0)
    rows = data.read_jsonl(tmp_path / 'minimap' / 'trail.jsonl')
    assert [r['x'] for r in rows] == [1.0, 2.0, 2.0]
    assert rows[0]['schema'] == feeds.TRAIL_SCHEMA and rows[0]['fix'] == 6
    assert rows[1]['speed_mps'] is None
    assert set(rows[0]) >= {'t_ns', 'x', 'y', 'epoch', 'fix', 'h_acc_m', 'speed_mps', 'xte_m'}


def test_rotation_fact_reports_degrees_once_there_is_baseline():
    rotation = MapRotation(settle=0.0)
    assert feeds.rotation_fact(rotation, 0.0) is None
    theta = math.radians(7.0)
    for k in range(30):
        x, y = k * 0.5, 0.2 * k
        rotation.add(float(k), (math.cos(theta) * x - math.sin(theta) * y,
                                math.sin(theta) * x + math.cos(theta) * y), (x, y), (x, y))
    fact = feeds.rotation_fact(rotation, 1.0)
    assert fact.value == pytest.approx(7.0, abs=0.01)


def test_second_fetch_waits_for_a_fresh_costmap():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.nav2 import Nav2

    node = FakeNode()
    nav = types.SimpleNamespace(costmap=None, costmap_time=None)
    nav._costmap_sub = feeds.LazySub(node, object, '/c', lambda m: None, None)
    nav.costmap_lease = types.MethodType(Nav2.costmap_lease, nav)
    with nav.costmap_lease('first'):
        nav.costmap = 'old'
    assert nav.costmap == 'old' and not node.live  # kept after the lease
    assert Nav2.fetch_costmap(nav, timeout=0.1) is None  # stale grid is not returned
    with nav.costmap_lease('a'):
        nav.costmap = 'live'
        with nav.costmap_lease('b'):  # a nested lease keeps the live grid
            assert nav.costmap == 'live'
