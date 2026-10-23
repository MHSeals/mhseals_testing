"""Command output for the profile's plant: Twist or TwistStamped on its topic.

The runner calls ``publish`` every tick and ``neutral`` on every exit; this
module only knows how to turn (surge, sway, yaw) into the right message.
"""

import json

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, UInt16MultiArray

from mhseals_testing.health.probes.graph import topic_fact_name

NEUTRAL_REPEATS = 3  # a few copies so one dropped message can't leave thrust on
# thruster_pwm_node's raw-pulse input: physical outputs 1-4 in µs, for bench tests.
TEST_PULSES_TOPIC = '/thruster_pwm_node/test_pulses'
MUX_REQUEST_TOPIC = '/command_mux/request'  # command_mux ~/request: {op, cause, by}


class Plant:
    def __init__(self, ros, profile):
        self.ros = ros
        self.profile = profile
        self.stamped = profile.command_type == 'twist_stamped'
        self.publisher = None  # exists only while armed, so an idle suite is no conflict
        self.manual_publisher = None  # keyboard drive and Space only
        self.pulse_publisher = None
        # Created up front: the mux's reliable subscription needs discovery time, so a
        # publisher made at the first Space would lose that first (safety) request.
        self.request_publisher = (ros.node.create_publisher(String, MUX_REQUEST_TOPIC, 10)
                                  if profile.muxed else None)

    def _create(self, topic):
        return self.ros.node.create_publisher(TwistStamped if self.stamped else Twist, topic, 10)

    def engage(self):
        """Start publishing on the command topic (on arm, or the first command)."""
        if self.publisher is None:
            self.publisher = self._create(self.profile.command_topic)

    def _manual(self):
        """The keyboard's publisher: the mux's manual input, shared with the command topic if unmuxed."""
        if self.profile.manual_topic == self.profile.command_topic:
            self.engage()
            return self.publisher
        if self.manual_publisher is None:
            self.manual_publisher = self._create(self.profile.manual_topic)
        return self.manual_publisher

    def release(self):
        """Stop being a publisher of the command topics (on disarm)."""
        for name in ('publisher', 'manual_publisher'):
            if getattr(self, name) is not None:
                self.ros.node.destroy_publisher(getattr(self, name))
                setattr(self, name, None)

    def clamp(self, command):
        return tuple(max(-limit, min(limit, float(value)))
                     for value, limit in zip(command, self.profile.command_limit))

    def message(self, command):
        surge, sway, yaw = self.clamp(command)
        if self.stamped:
            message = TwistStamped()
            message.header.stamp = self.ros.node.get_clock().now().to_msg()
            message.header.frame_id = self.profile.base_frame
            twist = message.twist
        else:
            message = twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = surge, sway, yaw
        return message

    def publish(self, command, manual=False):
        """Procedures publish on the command topic (auto); keyboard drive passes manual=True."""
        if manual:
            publisher = self._manual()
        else:
            self.engage()
            publisher = self.publisher
        publisher.publish(self.message(command))

    def neutral(self):
        for publisher in (self.publisher, self.manual_publisher):  # never commanded: silent
            if publisher is not None:
                for _ in range(NEUTRAL_REPEATS):
                    publisher.publish(self.message((0.0, 0.0, 0.0)))

    def hold(self, cause='keys'):
        """Space: neutral on the manual input, then ask the mux to hold (zeros, sticky) so no
        auto command resumes behind the stop."""
        if self.request_publisher is None:
            return
        manual = self._manual()
        for _ in range(NEUTRAL_REPEATS):
            manual.publish(self.message((0.0, 0.0, 0.0)))
        self.request_publisher.publish(String(data=json.dumps(
            {'op': 'hold', 'cause': cause, 'by': 'tui'})))

    def test_pulses(self, pulses):
        if self.pulse_publisher is None:
            self.pulse_publisher = self.ros.node.create_publisher(
                UInt16MultiArray, TEST_PULSES_TOPIC, 10)
        self.pulse_publisher.publish(UInt16MultiArray(data=[int(p) for p in pulses]))

    def conflicts(self):
        """Another node commanding the same topic would fight every trial."""
        topic = self.profile.command_topic
        pubs = self.ros.graph_facts([topic])[topic_fact_name(topic)].value['pub']
        return [f'{topic} also published by {ep.node}' for ep in pubs if not ep.own]

    def close(self):
        self.neutral()
        self.release()
