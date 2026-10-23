"""Command output for the profile's plant: Twist or TwistStamped on its topic.

The runner calls ``publish`` every tick and ``neutral`` on every exit; this
module only knows how to turn (surge, sway, yaw) into the right message.
"""

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import UInt16MultiArray

NEUTRAL_REPEATS = 3  # a few copies so one dropped message can't leave thrust on
# thruster_pwm_node's raw-pulse input: physical outputs 1-4 in µs, for bench tests.
TEST_PULSES_TOPIC = '/thruster_pwm_node/test_pulses'


class Plant:
    def __init__(self, ros, profile):
        self.ros = ros
        self.profile = profile
        self.stamped = profile.command_type == 'twist_stamped'
        self.publisher = None  # exists only while armed, so an idle suite is no conflict
        self.pulse_publisher = None

    def engage(self):
        """Start publishing on the command topic (on arm, or the first command)."""
        if self.publisher is None:
            self.publisher = self.ros.node.create_publisher(
                TwistStamped if self.stamped else Twist, self.profile.command_topic, 10)

    def release(self):
        """Stop being a publisher of the command topic (on disarm)."""
        if self.publisher is not None:
            self.ros.node.destroy_publisher(self.publisher)
            self.publisher = None

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

    def publish(self, command):
        self.engage()
        self.publisher.publish(self.message(command))

    def neutral(self):
        if self.publisher is None:  # never commanded, so nothing to zero
            return
        for _ in range(NEUTRAL_REPEATS):
            self.publisher.publish(self.message((0.0, 0.0, 0.0)))

    def test_pulses(self, pulses):
        if self.pulse_publisher is None:
            self.pulse_publisher = self.ros.node.create_publisher(
                UInt16MultiArray, TEST_PULSES_TOPIC, 10)
        self.pulse_publisher.publish(UInt16MultiArray(data=[int(p) for p in pulses]))

    def conflicts(self):
        """Another node commanding the same topic would fight every trial."""
        return [f'{self.profile.command_topic} also published by {name}'
                for name in self.ros.publishers(self.profile.command_topic)]

    def close(self):
        self.neutral()
        self.release()
