"""The Perception procedure: per-row rate minimums and the costmap it judges."""

from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from mhseals_testing.procedures import perception
from mhseals_testing.profiles import SensorSpec


class Nav:
    def __init__(self, grids):
        self.grids, self.asked = grids, []

    def fetch_costmap(self, timeout=2.0, topic=None):
        self.asked.append(topic)
        return self.grids.get(topic)


def run(grids, sensors=(), rows=None):
    @contextmanager
    def observe(name):
        yield SimpleNamespace(supervise=lambda *a, **k: None)
    nav, recorded = Nav(grids), []
    ctx = SimpleNamespace(observe=observe, settings=SimpleNamespace(observe=1), record=recorded.append,
                          monitor=SimpleNamespace(snapshot=lambda: {'sensors': rows or {}}),
                          profile=SimpleNamespace(sensors=list(sensors)), services={'nav2': lambda: nav})
    return {r['check']: r for r in perception.run(ctx)}, nav


def test_costmap_prefers_the_perception_grid_and_falls_back_to_nav2():
    grid = SimpleNamespace(data=[0, 100, 99])
    results, nav = run({perception.PERCEPTION_COSTMAP: grid})
    assert results['costmap']['verdict'] == 'PASS' and nav.asked == [perception.PERCEPTION_COSTMAP]
    assert results['costmap']['detail'].startswith('/perception/costmap: 2 obstacle cells')
    results, nav = run({None: grid})
    assert results['costmap']['detail'].startswith('local costmap: 2') and nav.asked == ['/perception/costmap', None]
    assert run({})[0]['costmap']['verdict'] == 'N/A'


def test_a_row_rate_sets_the_minimum_else_its_kind():
    row = dict(status='LIVE', hz=1.5, age=0.1, issue='', values={})
    sensors = [SensorSpec('preview', '/front_camera/preview/compressed', 'compressed', hz=2),
               SensorSpec('camera', '/cam', 'compressed'), SensorSpec('compass', '/mag', 'mag')]
    results, _ = run({}, sensors, {'preview': row, 'camera': row, 'compass': row})
    assert results['preview']['verdict'] == 'PASS' and results['camera']['verdict'] == 'FAIL'
    assert 'compass' not in results                      # kinds without a minimum are not judged


def test_nav2_fetches_another_topic_once_without_its_lease():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.nav2 import Nav2

    class Node:
        def __init__(self):
            self.live = []

        def create_subscription(self, kind, topic, callback, qos):
            self.live.append(topic)
            callback(f'grid on {topic}')
            return topic

        def destroy_subscription(self, sub):
            self.live.remove(sub)
    nav = SimpleNamespace(node=Node(), costmap_topic='/local_costmap/costmap')
    assert Nav2.fetch_costmap(nav, topic='/perception/costmap') == 'grid on /perception/costmap'
    assert nav.node.live == []
