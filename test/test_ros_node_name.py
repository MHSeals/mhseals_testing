"""Two suite instances must not share a node name, or the command-topic interlock is blind."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from mhseals_testing.ros.node import RosIO, node_name


def test_same_pid_on_two_hosts_gets_two_names():
    assert node_name(1, 'odroid') != node_name(1, 'jetson.local')


def test_node_name_is_per_process():
    assert node_name(1) != node_name(2)


def test_second_instance_is_reported_as_a_publisher():
    infos = [SimpleNamespace(node_name=node_name(n), node_namespace='/') for n in (1, 2)]
    fake = SimpleNamespace(node=SimpleNamespace(
        get_name=lambda: node_name(1), get_publishers_info_by_topic=lambda topic: infos))
    assert RosIO.publishers(fake, '/cmd_vel') == ['/' + node_name(2)]


def test_instances_lists_other_suite_nodes_only():
    names = [(node_name(1), '/'), (node_name(2), '/'), ('thruster_pwm_node', '/')]
    fake = SimpleNamespace(node=SimpleNamespace(
        get_name=lambda: node_name(1), get_node_names_and_namespaces=lambda: names))
    assert RosIO.instances(fake) == [node_name(2)]
