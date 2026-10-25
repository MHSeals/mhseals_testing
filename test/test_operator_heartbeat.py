"""operator_heartbeat refuses to start on a boat host. No ROS."""

import socket
import sys
from types import ModuleType

import pytest

from mhseals_testing.ros import operator_heartbeat as heartbeat


def test_shipped_config_names_the_boat_computers():
    config = heartbeat.settings()
    assert heartbeat.on_boat_host('blastoise-odroid', config) and heartbeat.on_boat_host('blastoise-jetson', config)
    assert not heartbeat.on_boat_host('tranquility', config)


@pytest.mark.parametrize('config', [{}, {'boat_hosts': 'blastoise*'}, {'boat_hosts': [1]}])
def test_an_unreadable_boat_list_refuses(config):
    assert heartbeat.on_boat_host('tranquility', config)


def test_main_refuses_on_a_boat_host_before_touching_ros(monkeypatch, capsys):
    monkeypatch.setattr(socket, 'gethostname', lambda: 'blastoise-odroid')
    monkeypatch.setitem(sys.modules, 'rclpy', None)          # an import would raise
    assert heartbeat.main() == 2
    assert 'refusing' in capsys.readouterr().err


def test_main_publishes_the_host_name_on_an_operator_host(monkeypatch):
    monkeypatch.setattr(socket, 'gethostname', lambda: 'tranquility')
    seen = []
    rclpy = ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.create_node = lambda name: seen.append(name) or 'node'
    rclpy.spin = lambda node: (_ for _ in ()).throw(KeyboardInterrupt)
    rclpy.try_shutdown = lambda: seen.append('shutdown')
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setattr(heartbeat, 'bind', lambda node, period: seen.append(period))
    assert heartbeat.main() == 0
    assert seen == ['operator_heartbeat', 0.5, 'shutdown']
