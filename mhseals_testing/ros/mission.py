"""Watch a task run: collect /task/events and the latched /task/status.

``TaskWatcher`` is the mission procedure's lazy reader; ``MissionFeed`` is the always-on one
that parses the three mission topics into a ``ui.feeds.mission.MissionState`` for the TUI.
"""

import json

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MISSION_TOPICS = (('/task/status', 'status', 1), ('/task/events', 'event', 100),
                  ('/task/hypotheses', 'hypotheses', 1))   # (topic, MissionState kind, depth)


class MissionFeed:
    """Subscribes the mission topics (RELIABLE, TRANSIENT_LOCAL) and hands each parsed message on."""

    def __init__(self, node, state):
        self.state, self.log = state, node.get_logger()
        self.drops = {}     # kind -> messages dropped; the first of each kind is logged
        for topic, kind, depth in MISSION_TOPICS:
            node.create_subscription(
                String, topic, lambda m, k=kind: self._take(k, m.data),
                QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _take(self, kind, data):
        try:
            self.state.ingest(kind, json.loads(data))
        except Exception as error:   # any bad message is dropped: raising here would end RosIO's spin
            self.drops[kind] = self.drops.get(kind, 0) + 1
            if self.drops[kind] == 1:
                self.log.warning(f'mission {kind} dropped ({type(error).__name__}: {error}); '
                                 'later drops are only counted')


class TaskWatcher:
    def __init__(self, ros):
        self.events, self.status = [], {}
        ros.node.create_subscription(  # transient local: the run's first events too
            String, '/task/events', self._event,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
