"""Odometry sources beyond the profile's sensors, validated and kept for the Odometry popup.

GPS raw and the compass are profile sensors (``gpsraw`` and ``mag`` kinds). The
estimates below are not shown as sensors: only their latest observation is held.
"""

from functools import partial

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from mhseals_testing.validation import validate

SOURCE_TOPICS = {'global': '/odom/global', 'gps': '/odom/gps', 'mavros': '/odom/mavros'}


def subscribe_sources(ros, topics=None):
    """Subscribe the odometry estimates; ``ros.sources[name]`` holds the newest Observation."""
    ros.sources = {}
    for name, topic in (topics or SOURCE_TOPICS).items():
        ros.node.create_subscription(
            Odometry, topic, partial(_store, ros, name), qos_profile_sensor_data)
    return ros.sources


def _store(ros, name, message):
    ros.sources[name] = validate('odometry', message, ros.now(), stale_after=ros.stale_after)
