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
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setattr(heartbeat, 'bind', lambda node: seen.append(node))
    assert heartbeat.main() == 0
    assert seen == ['operator_heartbeat', 'node', 'shutdown']
