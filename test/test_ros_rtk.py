"""RTCM sink behaviour against a fake node; needs mavros_msgs but no running ROS graph."""

from types import SimpleNamespace

import pytest

pytest.importorskip('mavros_msgs')

from builtin_interfaces.msg import Time
from rclpy.qos import HistoryPolicy, ReliabilityPolicy

from mhseals_testing.ros.rtk import MAX_AGE, MAX_FRAME, MavrosSink


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, text):
        self.warnings.append(text)


class FakeNode:
    def __init__(self, others=()):
        self.logger, self.published, self.qos = FakeLogger(), [], None
        self.others = [SimpleNamespace(node_name=n, node_namespace='/') for n in others]
        self.checks = 0

    def count_publishers(self, topic):
        return 1 + len(self.others)

    def create_publisher(self, kind, topic, qos):
        self.qos = qos
        return SimpleNamespace(publish=self.published.append)

    def get_publishers_info_by_topic(self, topic):
        self.checks += 1
        return [SimpleNamespace(node_name='rtk_base', node_namespace='/')] + self.others

    def get_name(self):
        return 'rtk_base'

    def get_namespace(self):
        return '/'

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))


def sink(node, now=100.):
    return MavrosSink(node, clock=lambda: now)


def test_one_publish_per_frame_without_splitting():
    node = FakeNode()
    send = sink(node)
    send(bytes(500), 100.)
    send(b'\xd3' * 10, 99.9)
    assert [len(m.data) for m in node.published] == [500, 10]
    assert node.qos.depth == 16
    assert node.qos.reliability == ReliabilityPolicy.RELIABLE
    assert node.qos.history == HistoryPolicy.KEEP_LAST
    assert node.qos.lifespan.nanoseconds == 0
    assert send.counts['sent'] == 2


def test_exact_multiples_of_a_fragment_are_split_so_the_fcu_sees_the_end():
    node = FakeNode()
    send = sink(node)
    for size in (180, 360, 540, 720, 361):
        send(bytes(size), 100.)
    assert [len(m.data) for m in node.published] == [180, 359, 1, 539, 1, 720, 361]
    assert send.counts['sent'] == 5 and send.counts['split'] == 2


def test_stale_frame_is_dropped_and_counted():
    node = FakeNode()
    send = sink(node)
    send(bytes(8), 100. - MAX_AGE - .1)
    assert not node.published and send.counts['stale'] == 1


def test_oversize_frame_is_dropped_with_warning():
    node = FakeNode()
    send = sink(node)
    send(bytes(MAX_FRAME + 1), 100.)
    assert not node.published and send.counts['oversize'] == 1 and node.logger.warnings


def test_second_publisher_warns_once_and_frames_still_flow():
    node = FakeNode(others=['mission_planner_bridge'])
    send = sink(node)
    send(bytes(8), 100.)
    send(bytes(8), 100.)
    assert len(node.logger.warnings) == 1 and 'mission_planner_bridge' in node.logger.warnings[0]
    assert node.checks == 1 and len(node.published) == 2


def test_publisher_appearing_later_warns_once():
    node = FakeNode()
    send = sink(node)
    send(bytes(8), 100.)
    assert not node.logger.warnings
    node.others.append(SimpleNamespace(node_name='boat_test', node_namespace='/'))
    send(bytes(8), 100.)
    send(bytes(8), 100.)
    assert len(node.logger.warnings) == 1 and 'boat_test' in node.logger.warnings[0]
    assert node.checks == 1 and len(node.published) == 3


def test_other_publisher_with_our_node_name_still_warns():
    node = FakeNode(others=['rtk_base'])
    send = sink(node)
    send(bytes(8), 100.)
    assert len(node.logger.warnings) == 1 and 'rtk_base' in node.logger.warnings[0]


def test_replacement_publisher_is_warned_after_count_changes():
    node = FakeNode(others=['first'])
    send = sink(node)
    send(bytes(8), 100.)
    node.others = []
    send(bytes(8), 100.)
    node.others = [SimpleNamespace(node_name='second', node_namespace='/')]
    send(bytes(8), 100.)
    assert len(node.logger.warnings) == 2 and 'second' in node.logger.warnings[1]


def test_status_publisher_is_latched_json_and_the_entry_point_imports():
    import json
    from mhseals_testing.ros.rtk import StatusPublisher, serve
    from mhseals_testing.rtk import main
    node = FakeNode()
    node.create_timer = lambda period, callback: SimpleNamespace(period=period, callback=callback)
    node.create_publisher = lambda kind, topic, qos: (setattr(node, 'qos', qos), setattr(node, 'topic', topic),
                                                      SimpleNamespace(publish=node.published.append))[2]
    status = StatusPublisher(node, SimpleNamespace(status=lambda: dict(schema='mhseals.rtk_status/1', age=0.4)))
    assert node.topic == '/mhseals_testing/rtk/status' and status.timer.period == 1.0
    assert node.qos.durability.name == 'TRANSIENT_LOCAL'
    status.timer.callback()
    assert json.loads(node.published[0].data) == dict(schema='mhseals.rtk_status/1', age=0.4)
    assert callable(serve) and callable(main)


def test_sink_tracks_bytes_and_the_last_frame_time():
    send = sink(FakeNode(), now=100.)
    send(bytes(100), 99.9)
    assert send.bytes == 100 and send.last == 100.
