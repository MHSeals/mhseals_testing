"""Two suite instances must not share a node name, or the command-topic interlock is blind."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from mhseals_testing.ros.node import RosIO, node_name


def test_node_name_is_per_process():
    assert node_name(1) != node_name(2)


def test_second_instance_is_reported_as_a_publisher():
    infos = [SimpleNamespace(node_name=node_name(n), node_namespace='/') for n in (1, 2)]
    fake = SimpleNamespace(node=SimpleNamespace(
        get_name=lambda: node_name(1), get_publishers_info_by_topic=lambda topic: infos))
    assert RosIO.publishers(fake, '/cmd_vel') == ['/' + node_name(2)]
