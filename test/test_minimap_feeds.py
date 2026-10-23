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


class WiringNode(FakeNode):
    """A node that records subscriptions, publishers and timers without rclpy executing anything."""

    def create_publisher(self, kind, topic, qos):
        return types.SimpleNamespace(publish=lambda m: None)

    def create_timer(self, period, callback):
        return callback


def wired(clock):
    pytest.importorskip('rclpy')
    from mhseals_testing.health.probes import Facts
    node, facts = WiringNode(), Facts()
    return node, facts, feeds.MapFeeds(node, facts=facts, clock=clock)


def odom(x, y):
    return types.SimpleNamespace(pose=types.SimpleNamespace(
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y),
                                   orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))


def feed_pairs(node, count, theta):
    """Drive count fake /gps/fix, /odom/gps, /odom/global triples; /odom/gps is rotated by theta."""
    from mhseals_testing.analysis.geo import geodetic
    topics = {topic: callback for topic, callback in node.live}
    for k in range(count):
        x, y = k * 0.5, 0.2 * k
        lat, lon = geodetic(x, y, (43.0, -70.0))
        topics['/gps/fix'](types.SimpleNamespace(
            status=types.SimpleNamespace(status=2), latitude=lat, longitude=lon))
        topics['/odom/gps'](odom(math.cos(theta) * x - math.sin(theta) * y,
                                 math.sin(theta) * x + math.cos(theta) * y))
        topics['/odom/global'](odom(x, y))


def test_no_costmap_subscription_until_a_layer_lease():
    now = [0.0]
    node, _, map_feeds = wired(lambda: now[0])
    assert sorted(t for t, _ in node.live) == ['/gps/fix', '/odom/global', '/odom/gps']
    map_feeds.want_layers(True, False, 20)
    assert feeds.GLOBAL in [t for t, _ in node.live]
    map_feeds.want_layers(False, False, 20)
    assert feeds.GLOBAL not in [t for t, _ in node.live]


def test_costmap_reaches_map_state_and_record_follows_the_gate():
    node, _, map_feeds = wired(lambda: 0.0)
    map_feeds.want_layers(True, False, 2)
    dict(node.live)[feeds.LOCAL]('grid')
    assert map_feeds.state.latest['costmap'] == 'grid'
    map_feeds.want_layers(False, False, 2)
    assert 'costmap' not in map_feeds.state.latest
    map_feeds.want_record(True, True, True)
    assert feeds.GLOBAL in [t for t, _ in node.live]
    map_feeds.want_record(True, False, True)
    assert feeds.GLOBAL not in [t for t, _ in node.live]


def test_recorder_lease_does_not_leak_global_grid_into_state():
    node, _, map_feeds = wired(lambda: 0.0)
    map_feeds.want_record(True, True, True)
    map_feeds.want_layers(True, False, 2)
    live = dict(node.live)
    live[feeds.LOCAL]('local')
    live[feeds.GLOBAL]('global')
    assert map_feeds.state.latest['costmap'] == 'local'
    map_feeds.want_layers(False, False, 2)
    live[feeds.GLOBAL]('global')
    assert 'costmap' not in map_feeds.state.latest
    assert map_feeds.costmaps[feeds.GLOBAL] == 'global'     # the recorder still gets it


def test_rotation_fact_reaches_health_and_the_check_goes_live():
    from mhseals_testing.health.config import load
    from mhseals_testing.health.engine import Engine

    now = [0.0]

    def clock():
        now[0] += 1.0
        return now[0]

    node, facts, map_feeds = wired(clock)
    spec = next(s for s in load(user=False).specs if s.id == 'nav.map_rotation')
    map_feeds.publish_fact()
    assert 'nav.map_rotation' not in facts.read()[0]
    engine = Engine([spec], clock=lambda: now[0] + 100)
    assert 'nav.map_rotation' not in engine.tick(*facts.read()).results   # missing: skip
    feed_pairs(node, 40, math.radians(12.0))
    map_feeds.publish_fact()
    fact = facts.read()[0]['nav.map_rotation']
    assert abs(fact.data['signed_deg']) == pytest.approx(12.0, abs=0.2)
    engine.tick(*facts.read())
    now[0] += 100       # past the debounce
    assert engine.tick(*facts.read()).results['nav.map_rotation'].status == 'warn'   # severity: warn
