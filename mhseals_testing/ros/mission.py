"""Watch a task run: collect /task/events and the latched /task/status."""

import json

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class TaskWatcher:
    def __init__(self, ros):
        self.events, self.status = [], {}
        ros.node.create_subscription(String, '/task/events', self._event, 50)
        ros.node.create_subscription(
            String, '/task/status', self._status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.monitor = ros.monitor

    def _event(self, message):
        event = json.loads(message.data)
        event['received'] = self.monitor.clock()  # on the track's clock, for the referee
        self.events.append(event)
        self.monitor.event(f"task {event.get('event')}")

    def _status(self, message):
        self.status = json.loads(message.data)

    def reset(self):
        self.events.clear()
        self.status = {}
