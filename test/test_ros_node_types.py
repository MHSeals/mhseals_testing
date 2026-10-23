"""gpsraw resolves lazily, so a host without mavros_msgs still builds the node."""

import sys

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav_msgs')

from mhseals_testing.ros.node import message_type


def test_gpsraw_missing_mavros_is_none(monkeypatch):
    monkeypatch.setitem(sys.modules, 'mavros_msgs', None)
    monkeypatch.setitem(sys.modules, 'mavros_msgs.msg', None)
    assert message_type('gpsraw') is None


def test_other_kinds_do_not_need_mavros(monkeypatch):
    monkeypatch.setitem(sys.modules, 'mavros_msgs', None)
    monkeypatch.setitem(sys.modules, 'mavros_msgs.msg', None)
    assert message_type('imu') is not None
