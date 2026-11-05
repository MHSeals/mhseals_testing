"""The raw MAVLink fallback for a gpsraw topic is subscribed only while nothing publishes that topic."""

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')

from mhseals_testing.ros.node import RosIO


class Node:
    def __init__(self):
        self.publishers, self.subs, self.destroyed = 0, [], []

    def count_publishers(self, topic):
        return self.publishers

    def create_subscription(self, kind, topic, callback, qos):
        self.subs.append(topic)
        return len(self.subs)

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


def test_the_fallback_follows_whether_the_gpsraw_topic_publishes():
    ros = RosIO.__new__(RosIO)
    ros.node = Node()
    ros.sensor_node = ros.node
    ros._mavlink, ros._mavlink_source = {'/gps/raw': None}, '/uas1/mavlink_source'
    ros._watch_mavlink()
    assert ros.node.subs == ['/uas1/mavlink_source'] and ros._mavlink['/gps/raw'] == 1   # silent: the fallback
    ros._watch_mavlink()
    assert len(ros.node.subs) == 1                                                       # once
    ros.node.publishers = 1
    ros._watch_mavlink()
    assert ros.node.destroyed == [1] and ros._mavlink['/gps/raw'] is None                # it publishes: dropped
    ros.node.publishers = 0
    ros._watch_mavlink()
    assert len(ros.node.subs) == 2                                                       # silent again: back
