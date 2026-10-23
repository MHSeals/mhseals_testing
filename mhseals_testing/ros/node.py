"""The suite's one ROS node: sensor and TF subscriptions feeding the monitor.

Callbacks only validate and record; they never command anything. The node
spins on its own thread so the TUI loop can block on keys.
"""

from functools import partial
import os
import re
import socket
import threading
import time

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image, Imu, MagneticField, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

from mhseals_testing import events
from mhseals_testing.harness import journal
from mhseals_testing.harness.simsession import ClockWatch
from mhseals_testing.health.probes.graph import collect
from mhseals_testing.ros.graph import RclpyGraph
from mhseals_testing.ros.ingest import RTK_TOPIC, ingest
from mhseals_testing.ros.minimap_feeds import MapFeeds
from mhseals_testing.ros.sources import subscribe_sources
from mhseals_testing.validation import RAW_KINDS

MESSAGE_TYPES = {
    'odometry': Odometry, 'imu': Imu, 'gps': NavSatFix,
    'mag': MagneticField, 'points': PointCloud2,
    'image': Image, 'compressed': CompressedImage, 'detections': Detection3DArray,
}


def message_type(kind):
    """Message class for a sensor kind; None when gpsraw's mavros_msgs is not installed."""
    if kind == 'gpsraw':
        try:
            from mavros_msgs.msg import GPSRAW
        except ImportError:
            return None
        return GPSRAW
    return MESSAGE_TYPES[kind]


EVENTS_TOPIC = '/mhseals_testing/events'
GUID_PREFIX = 12             # bytes of an endpoint gid that name its DDS participant
DRIFT_TOPIC = '/calibrated_velocity_controller/drift'
EVENTS_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def node_name(pid=None, host=None):
    """Per-process, per-host node name, so publishers() can tell a second suite from ourselves.

    Pids repeat across machines and pid-namespaced containers on one domain.
    """
    host = re.sub(r'\W', '_', socket.gethostname() if host is None else host)
    return f'mhseals_testing_{host}_{os.getpid() if pid is None else pid}'


class RosIO:
    """Owns the node, its executor thread and the publishers the suite needs."""

    def __init__(self, profile, monitor, stale_after=2.0, journal_path=None, facts=None):
        self.profile = profile
        self.monitor = monitor
        self.stale_after = stale_after
        self.fault = ''
        self.power_facts = None   # analysis.power facts of /mavros/battery; None without a voltage source
        self.journal_path = journal_path
        self.recording = True  # stamped on each event as ``rec``
        self._prefix = None
        self.node = Node(node_name(), parameter_overrides=[
            Parameter('use_sim_time', value=profile.use_sim_time)])
        # Latched, so a recorder that starts (or is discovered) late still gets session_start.
        self.events = self.node.create_publisher(String, EVENTS_TOPIC, EVENTS_QOS)
        for sensor in profile.sensors:
            message = message_type(sensor.kind)
            if message is None:
                monitor.advise(sensor.name, 'N/A', 'mavros_msgs not installed')
                continue
            raw = sensor.kind in RAW_KINDS
            self.node.create_subscription(
                message, sensor.topic, partial(self._ingest, sensor.topic, sensor.kind),
                qos_profile_sensor_data, raw=raw)
        self.node.create_subscription(
            Vector3Stamped, DRIFT_TOPIC, partial(self._ingest, DRIFT_TOPIC, 'drift'), 10)
        if profile.use_sim_time:
            self.clock_watch = ClockWatch()
            self.node.create_subscription(Clock, '/clock', self._clock, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, '/tf', self._tf, qos_profile_sensor_data)
        self.node.create_subscription(
            TFMessage, '/tf_static', partial(self._tf, static=True),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Latched JSON from the RTK base; a legacy DiagnosticArray is read by ingest.parse_rtk from bags
        self.node.create_subscription(
            String, RTK_TOPIC, partial(self._ingest, RTK_TOPIC, 'rtk'), EVENTS_QOS)
        subscribe_sources(self)
        self.map = MapFeeds(self.node, facts=facts, on_fix=partial(self.emit, 'gps_fix'))  # rotation always, costmaps on lease
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

    def _clock(self, message):
        # An episode reset rewinds /clock; one system marker per rewind (bags.md B14).
        if self.clock_watch.feed(message.clock.sec + message.clock.nanosec / 1e9):
            self.emit('marker', kind='system', note='sim reset')

    def _ingest(self, topic, kind, message, authority=''):
        ingest(self.monitor, self.profile, topic, kind, message, self.now(),
               self.stale_after, authority)

    def _tf(self, message, info=None, static=False):
        # The publisher GID tells two publishers of the same edge apart.
        gid = getattr(info, 'publisher_gid', None) or (info or {}).get('publisher_gid', b'')
        self._ingest('/tf_static' if static else '/tf', 'tf_static' if static else 'tf',
                     message, bytes(getattr(gid, 'data', gid)).hex())

    def emit(self, phase, **details):
        """Journal an event, publish it into the bag and show it on the dashboard."""
        rec = details.get('rec', self.recording)
        event = events.payload(phase, self.now(), dict(details, rec=rec), node=self.node.get_name())
        if self.journal_path:
            journal.append(self.journal_path, event)  # first: it survives a dead publisher
        self.events.publish(String(data=events.line(event)))
        self._ingest(EVENTS_TOPIC, 'event', event)

    def endpoints(self, topic, subscribers=False):
        """Every publisher (or subscriber) of ``topic`` as ``{node, gid, self}``.

        Our own endpoints are tagged ``self`` by GUID prefix, never dropped by node name, so a
        second suite (or any node that shares our name) still shows up. The prefix is shared by
        every endpoint of one DDS participant; it is learned from our events publisher. If the
        graph cannot tell which participant is ours, nothing is tagged ``self`` (fail visible).
        """
        get = (self.node.get_subscriptions_info_by_topic if subscribers
               else self.node.get_publishers_info_by_topic)
        own = self._own_prefix()
        out = []
        for info in get(topic):
            gid = bytes(getattr(info, 'endpoint_gid', ()) or ())
            out.append({'node': f'{info.node_namespace.rstrip("/")}/{info.node_name}',
                        'gid': gid.hex(), 'self': own is not None and gid[:GUID_PREFIX] == own})
        return out

    def _own_prefix(self):
        """GUID prefix of this process's DDS participant, or None while ambiguous."""
        if self._prefix is None:
            prefixes = {bytes(getattr(i, 'endpoint_gid', ()) or ())[:GUID_PREFIX]
                        for i in self.node.get_publishers_info_by_topic(EVENTS_TOPIC)
                        if i.node_name == self.node.get_name()}
            if len(prefixes) == 1 and b'' not in prefixes:
                self._prefix = prefixes.pop()
        return self._prefix

    def publishers(self, topic):
        """Other nodes publishing ``topic`` (e.g. a second /cmd_vel source)."""
        return [e['node'] for e in self.endpoints(topic) if not e['self']]

    def graph_facts(self, topics, full=False):
        """The graph probe's facts for ``topics`` (see health/probes/graph.py)."""
        return collect(RclpyGraph(self.node), topics, time.monotonic(), full)

    def subscribers(self, topic):
        return [e['node'] for e in self.endpoints(topic, True) if not e['self']]

    def nodes(self):
        return [f'{namespace.rstrip("/")}/{name}'
                for name, namespace in self.node.get_node_names_and_namespaces()]

    def instances(self):
        """Other suite nodes on the graph (names only); a crashed one lingers until its DDS lease ends."""
        return sorted({name for name, _ in self.node.get_node_names_and_namespaces()
                       if name.startswith('mhseals_testing_') and name != self.node.get_name()})

    def parameter_service(self, name):
        """Whether node ``name`` serves parameters (a dead service hides the node's config)."""
        return any(service.endswith(f'/{name}/get_parameters')
                   for service, _ in self.node.get_service_names_and_types())

    def driver_parameters(self, name, keys, timeout=0.5):
        """Node ``name``'s live values for parameters ``keys`` (the config readback), or None."""
        client = self.node.create_client(GetParameters, f'/{name}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            done = threading.Event()
            future = client.call_async(GetParameters.Request(names=list(keys)))
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout) or future.result() is None:
                return None
            values = [parameter_value_to_python(v) for v in future.result().values]
            return dict(zip(keys, (list(v) if isinstance(v, (list, tuple)) or hasattr(v, 'tolist')
                                   else v for v in values)))
        finally:
            self.node.destroy_client(client)

    def close(self):
        self._spinning = False
        self.thread.join(timeout=1)
        self.executor.shutdown(timeout_sec=1)
        self.node.destroy_node()

