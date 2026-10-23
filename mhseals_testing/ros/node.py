"""The suite's one ROS node: sensor and TF subscriptions feeding the monitor.

Callbacks only validate and record; they never command anything. The node
spins on its own thread so the TUI loop can block on keys.
"""

from functools import partial
import os
import json
import threading

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

from mhseals_testing import events
from mhseals_testing.validation import finite, validate

MESSAGE_TYPES = {
    'odometry': Odometry, 'imu': Imu, 'gps': NavSatFix, 'points': PointCloud2,
    'image': Image, 'compressed': CompressedImage, 'detections': Detection3DArray,
}
EVENTS_TOPIC = '/mhseals_testing/events'
DRIFT_TOPIC = '/calibrated_velocity_controller/drift'


def node_name(pid=None):
    """Per-process node name, so publishers() can tell a second suite from ourselves."""
    return f'mhseals_testing_{os.getpid() if pid is None else pid}'


class RosIO:
    """Owns the node, its executor thread and the publishers the suite needs."""

    def __init__(self, profile, monitor, stale_after=2.0):
        self.profile = profile
        self.monitor = monitor
        self.stale_after = stale_after
        self.fault = ''
        self.node = Node(node_name(), parameter_overrides=[
            Parameter('use_sim_time', value=profile.use_sim_time)])
        self.events = self.node.create_publisher(String, EVENTS_TOPIC, 10)
        for sensor in profile.sensors:
            self.node.create_subscription(
                MESSAGE_TYPES[sensor.kind], sensor.topic,
                partial(self._sensor, sensor.name, sensor.kind), qos_profile_sensor_data)
        self.node.create_subscription(
            Vector3Stamped, DRIFT_TOPIC,
            lambda message: monitor.estimate_drift(message.vector.x, message.vector.y), 10)
        self.node.create_subscription(TFMessage, '/tf', self._tf, qos_profile_sensor_data)
        self.node.create_subscription(
            TFMessage, '/tf_static', partial(self._tf, static=True),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spinning = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            while self._spinning and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as error:  # surfaced on the dashboard; motion is then refused
            self.fault = str(error)
            self.monitor.event('ROS fault: ' + self.fault)

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def _sensor(self, name, kind, message):
        base = self.profile.base_frame if name == 'odometry' else None
        result = validate(kind, message, self.now(), base, self.stale_after)
        self.monitor.observe(name, result.values, result.issue, result.stamp_age)

    def _tf(self, message, info=None, static=False):
        # The publisher GID tells two publishers of the same edge apart.
        gid = getattr(info, 'publisher_gid', None) or (info or {}).get('publisher_gid', b'')
        authority = bytes(getattr(gid, 'data', gid)).hex()
        now = self.now()
        for transform in message.transforms:
            q, t = transform.transform.rotation, transform.transform.translation
            if (not finite(q.x, q.y, q.z, q.w, t.x, t.y, t.z)
                    or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > .1):
                self.monitor.event('invalid TF transform: ' + transform.child_frame_id)
                continue
            stamp = transform.header.stamp
            self.monitor.transform(transform.header.frame_id, transform.child_frame_id,
                                   now - stamp.sec - stamp.nanosec / 1e9, static, authority)

    def emit(self, phase, **details):
        """Publish an event into the bag and show it on the dashboard."""
        self.events.publish(String(data=events.line(
            events.payload(phase, self.now(), details, node=self.node.get_name()))))
        self.monitor.event(phase + (' ' + json.dumps(details, sort_keys=True, default=str)
                                    if details else ''))

    def publishers(self, topic):
        """Other nodes publishing ``topic`` (e.g. a second /cmd_vel source)."""
        return [f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in self.node.get_publishers_info_by_topic(topic)
                if info.node_name != self.node.get_name()]

    def subscribers(self, topic):
        return [f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in self.node.get_subscriptions_info_by_topic(topic)
                if info.node_name != self.node.get_name()]

    def nodes(self):
        return [f'{namespace.rstrip("/")}/{name}'
                for name, namespace in self.node.get_node_names_and_namespaces()]

    def parameter_service(self, name):
        """Whether node ``name`` serves parameters (a dead service hides the node's config)."""
        return any(service.endswith(f'/{name}/get_parameters')
                   for service, _ in self.node.get_service_names_and_types())

    def close(self):
        self._spinning = False
        self.thread.join(timeout=1)
        self.executor.shutdown(timeout_sec=1)
        self.node.destroy_node()

