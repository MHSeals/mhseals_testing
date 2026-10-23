"""Events are latched: a recorder that starts after session_start must still get it."""

from types import SimpleNamespace
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.node import Node
from std_msgs.msg import String

from mhseals_testing.ros.node import EVENTS_QOS, EVENTS_TOPIC, RosIO


def test_late_subscriber_receives_earlier_event():
    rclpy.init()
    io = reader = None
    try:
        profile = SimpleNamespace(sensors=[], use_sim_time=False, base_frame='base_link')
        io = RosIO(profile, SimpleNamespace(event=lambda text: None))
        io.emit('session_start')
        reader, got = Node('latched_reader'), []
        reader.create_subscription(String, EVENTS_TOPIC, got.append, EVENTS_QOS)
        deadline = time.time() + 5
        while not got and time.time() < deadline:
            rclpy.spin_once(reader, timeout_sec=0.1)
        assert got and 'session_start' in got[0].data
    finally:
        if reader:
            reader.destroy_node()
        if io:
            io.close()
        rclpy.shutdown()
