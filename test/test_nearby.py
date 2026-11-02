"""No raw clouds or images in the TUI's always-on subscriptions (D-20261101-26): the LiDAR row and the points-nearby
check read mhseals_nav's /perception/nearby; a raw image is only looked for on the graph."""

import json
from types import SimpleNamespace as NS

import pytest

from mhseals_testing.health import config as health_config
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import SensorSpec
from mhseals_testing.ros import nearby
from mhseals_testing.ui.feeds.conditions import nearby_point
from mhseals_testing.ui.panels.status import sensor_rows

CLOUD = {'sectors': [None] * 15 + [2.4], 'nearest': {'range': 2.4, 'bearing': 351.0}, 'count': 812, 'rate': 9.8,
         'latency': 0.06, 'frame': 'base_link'}
SPECS = (SensorSpec('LiDAR', '/points', 'points'), SensorSpec('camera', '/front_camera/rgb/image', 'image'))


def test_a_nearby_message_is_the_lidar_row_with_the_nodes_rate_and_latency():
    now = [10.0]
    monitor, facts = BoatMonitor(2.0, clock=lambda: now[0]), {}
    assert nearby.parse('not json') is None and nearby.parse('{"sectors": 3}') is None
    found = nearby.parse(json.dumps(CLOUD))
    for _ in range(3):
        nearby.observe(monitor, ['LiDAR'], found, lambda probe, f: facts.update({probe: f}), Fact)
        now[0] += 0.2                                           # the summary comes at 5 Hz, the cloud at 9.8
    row = sensor_rows(monitor.snapshot(), SPECS)[0]
    assert row['state'] == 'LIVE' and row['value'] == '10 Hz' and 'nearest 2.4 m at 351 deg' in row['detail']
    assert facts['nearby']['perc.nearby.range'].value == 2.4 and facts['nearby']['perc.nearby.range'].label
    nearby.observe(monitor, ['LiDAR'], dict(CLOUD, nearest=None), lambda probe, f: facts.update({probe: f}), Fact)
    assert facts['nearby'] == {}                                # nothing near: no fact, no warning


def test_a_raw_image_is_up_from_the_graph_and_never_shows_a_rate():
    monitor = BoatMonitor(2.0, clock=lambda: 5.0)
    nearby.published(monitor, 'camera', 0)
    assert sensor_rows(monitor.snapshot(), SPECS)[1]['state'] == 'OFF'
    nearby.published(monitor, 'camera', 1)
    row = sensor_rows(monitor.snapshot(), SPECS)[1]
    assert row['state'] == 'LIVE' and row['value'] == 'up'


def test_points_inside_the_configured_range_warn():
    spec = next(s for s in health_config.load(None, profile=None).specs if s.id == 'perc.nearby')
    engine = Engine([spec], clock=lambda: 100.0, periods={'nearby': 1.0})

    def status(value):
        facts = {'perc.nearby.range': Fact(value, 'at 10 deg', time=100.0)}
        return engine.tick(facts, {'nearby': (100.0, None)})
    assert status(spec.params.warn + 1).results['perc.nearby'].status == 'ok'
    assert status(spec.params.warn - 1).results['perc.nearby'].status == 'warn'


def test_the_conditions_view_gets_the_nearest_bearing_while_fresh():
    ros = NS(nearby=(50.0, CLOUD))
    assert nearby_point(ros, now=51.0) == {'range': 2.4, 'bearing': 351.0}
    assert nearby_point(ros, now=60.0) is None and nearby_point(NS(), now=1.0) is None


def test_rosio_subscribes_raw_clouds_only_while_tapped():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.node import RosIO
    made, gone = [], []
    node = NS(create_subscription=lambda *a, **k: made.append((a[1], k.get('raw'))) or len(made),
              destroy_subscription=gone.append)
    io = NS(node=node, taps={}, _tap_subs={}, profile=NS(sensors=SPECS), _tapped=lambda kind, m: None)
    RosIO.tap(io, 'points', print)
    RosIO.tap(io, 'points', print)
    assert made == [('/points', True)]
    RosIO.tap(io, 'points', None)
    assert gone == [1] and io.taps == {}


pytestmark = pytest.mark.smoke   # tools/smoke
