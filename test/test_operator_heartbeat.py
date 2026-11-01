"""operator_heartbeat publishes this host's name; link_watchdog does the host filtering. No ROS."""

import socket
import sys
from types import ModuleType

from mhseals_testing.ros import operator_heartbeat as heartbeat


def test_main_publishes_the_host_name(monkeypatch):
    monkeypatch.setattr(socket, 'gethostname', lambda: 'tranquility')
    seen = []
    rclpy = ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.create_node = lambda name: seen.append(name) or 'node'
    rclpy.spin = lambda node: (_ for _ in ()).throw(KeyboardInterrupt)
    rclpy.try_shutdown = lambda: seen.append('shutdown')
    executors = ModuleType('rclpy.executors')
    executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', executors)
    monkeypatch.setattr(heartbeat, 'bind', lambda node: seen.append(node))
    assert heartbeat.main() == 0
    assert seen == ['operator_heartbeat', 'node', 'shutdown']


def _beat(monkeypatch):
    """The String bind() publishes each period, with a fake node and std_msgs."""
    class String:
        def __init__(self, data):
            self.data = data

    class Node:
        def create_publisher(self, kind, topic, depth):
            self.topic = topic
            return self

        def create_timer(self, period, tick):
            self.period, self.tick = period, tick

        def publish(self, msg):
            self.sent = msg

    std_msgs, msgs = ModuleType('std_msgs'), ModuleType('std_msgs.msg')
    msgs.String = String
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', msgs)
    node = Node()
    heartbeat.bind(node)
    node.tick()
    assert (node.topic, node.period) == ('/operator/heartbeat', 0.5)
    return node.sent.data


def test_the_beat_names_this_machine(monkeypatch):
    monkeypatch.delenv('ASTRO_HOSTNAME', raising=False)
    monkeypatch.setattr(socket, 'gethostname', lambda: 'dingus.local')
    assert _beat(monkeypatch) == 'dingus'


def test_astro_hostname_names_a_container_named_roboboat(monkeypatch):
    # A container from before prebuild named containers after their machine is 'roboboat'.
    monkeypatch.setattr(socket, 'gethostname', lambda: 'roboboat')
    monkeypatch.setenv('ASTRO_HOSTNAME', 'dingus')
    assert _beat(monkeypatch) == 'dingus'
    monkeypatch.delenv('ASTRO_HOSTNAME')
    assert _beat(monkeypatch) == 'roboboat'
