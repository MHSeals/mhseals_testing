"""The suite's one ROS node: sensor and TF subscriptions feeding the monitor.

Callbacks only validate and record; they never command anything. The node
spins on its own thread so the TUI loop can block on keys.
"""

from functools import partial
import json
import os
import re
import socket
import tempfile
import threading
import time
import traceback

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image, Imu, MagneticField, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

from mhseals_testing import events
from mhseals_testing.harness import journal
from mhseals_testing.harness.horn import TOPIC as HORN_TOPIC, HornJournal
from mhseals_testing.harness.simsession import ClockWatch
from mhseals_testing.health.probes.graph import collect
from mhseals_testing.readiness import BEAT_STALE_S
from mhseals_testing.ros.graph import RclpyGraph
from mhseals_testing.ros.ingest import RTK_TOPIC, ingest
from mhseals_testing.health.model import Fact
from mhseals_testing.profiles import default_topics, topics_of
from mhseals_testing.ros import nearby
from mhseals_testing.ros.minimap_feeds import MapFeeds
from mhseals_testing.ros.mission import MissionFeed
from mhseals_testing.model.mission_status import MissionState
from mhseals_testing.ros.sources import subscribe_sources
from mhseals_testing.validation import GPS_RAW_INT, RAW_KINDS, GpsRawInt

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


EVENTS_TOPIC = events.TOPIC
GRAPH_S = 1.0          # raw image sensors: how often the graph is asked whether they publish
FAULT_LOG = os.path.join(tempfile.gettempdir(), 'boat_test_faults.log')  # callback tracebacks
MAVLINK_TOPIC = default_topics()['mavlink_source']   # MAVROS's every received frame; a profile's topics: overrides
AUTOPILOT = 1                            # MAV_COMP_ID_AUTOPILOT1, the FCU's component id
GUID_PREFIX = 12             # bytes of an endpoint gid that name its DDS participant
DRIFT_TOPIC = default_topics()['drift']             # the velocity controller's drift; likewise
EVENTS_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def node_name(pid=None, host=None):
    """Per-process, per-host node name, so publishers() can tell a second suite from ourselves.

    Pids repeat across machines and pid-namespaced containers on one domain.
    """
    host = re.sub(r'\W', '_', socket.gethostname() if host is None else host)
    return f'mhseals_testing_{host}_{os.getpid() if pid is None else pid}'


def make_executor(name=None):
    """The node's executor (config/perf.yaml ``tune.executor``): ``events`` is rclpy's C++ EventsExecutor, a
    fraction of SingleThreadedExecutor's per-message cost, which rebuilds its wait set in Python for every
    message; ``single``, or a rclpy without the EventsExecutor, is SingleThreadedExecutor."""
    if name is None:
        from mhseals_testing.tunables import tune
        name = tune('perf')['executor']
    if name == 'events':
        try:
            from rclpy.experimental import EventsExecutor
        except ImportError:
            return SingleThreadedExecutor()
        return EventsExecutor()
    return SingleThreadedExecutor()


class RosIO:
    """Owns the node, its executor thread and the publishers the suite needs."""

    def __init__(self, profile, monitor, stale_after=2.0, journal_path=None, facts=None, executor=None):
        self.profile = profile
        topics = topics_of(profile)
        self.monitor = monitor
        self.stale_after = stale_after
        self.fault = ''
        self._faults = set()    # callback faults already surfaced
        self.ghosts = set    # () -> node names to list once (the App sets Stack.ghosts)
        self.power_facts = None   # analysis.power facts of /mavros/battery; None without a voltage source
        self.journal_path = journal_path
        self.taps = {}         # kind -> callable(message): views that read a session subscription (Fusion tab)
        self.recording = True  # stamped on each event as ``rec``
        self._prefix = None
        self.beat_at, self.beat_runs, self._beat_subs = {}, {}, {}   # watchdog ~/status (watch_beats)
        self.beat_state = {}
        self.node = Node(node_name(), parameter_overrides=[
            Parameter('use_sim_time', value=profile.use_sim_time)])
        # Latched, so a recorder that starts (or is discovered) late still gets session_start.
        self.events = self.node.create_publisher(String, EVENTS_TOPIC, EVENTS_QOS)
        for sensor in profile.sensors:
            message = message_type(sensor.kind)
            if message is None:
                monitor.advise(sensor.name, 'N/A', 'mavros_msgs not installed')
                continue
            if sensor.kind in RAW_KINDS:      # never always-on (D-20261101-26): a summary, the graph, or tap()
                continue
            self.node.create_subscription(
                message, sensor.topic, partial(self._ingest, sensor.topic, sensor.kind), qos_profile_sensor_data)
            if sensor.kind == 'gpsraw':
                from mavros_msgs.msg import Mavlink
                self.node.create_subscription(Mavlink, topics['mavlink_source'],
                                              partial(self._gps_raw_int, sensor.topic), qos_profile_sensor_data)
        self.node.create_subscription(
            Vector3Stamped, topics['drift'], partial(self._ingest, topics['drift'], 'drift'), 10)
        self.facts, self.nearby, self._tap_subs = facts, None, {}
        clouds = [s.name for s in profile.sensors if s.kind == 'points']
        if clouds:
            self.node.create_subscription(String, nearby.TOPIC, partial(self._nearby, clouds), 10)
        images = [s for s in profile.sensors if s.kind in RAW_KINDS and s.kind != 'points']
        if images:
            self.node.create_timer(GRAPH_S, lambda: [nearby.published(
                monitor, s.name, self.node.count_publishers(s.topic)) for s in images])
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
        self.driver_status = None   # latest thruster driver ~/status (latched JSON)
        self.node.create_subscription(
            String, '/thruster_pwm_node/status', self._driver_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # The driver's ~/output (10 Hz): the pulses its outputs hold, read back; with its receive time.
        self.driver_output, self.driver_output_at = None, None
        self.node.create_subscription(String, '/thruster_pwm_node/output', self._driver_output, 10)
        if profile.use_sim_time:   # the sim's harbor-alert horn, journaled for the referee
            hear = HornJournal(self.emit, getattr(profile, 'course_truth', None))
            self.node.create_subscription(String, HORN_TOPIC, lambda m: hear(m.data), 10)
        self.map = MapFeeds(self.node, facts=facts, topics=topics, on_fix=partial(self.emit, 'gps_fix'))  # rotation always, costmaps on lease
        self.mission = MissionState(self.map.state)   # a ui feed; tasks and the goal go on into the map
        MissionFeed(self.node, self.mission)
        self.executor = make_executor(executor)
        self.spin_cpu = 0.0     # this node's executor thread's CPU time so far (ui/perf.py splits the TUI's CPU)
        self.executor.add_node(self.node)
        self._spinning = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while self._spinning and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as error:  # a bad sample must not stop every other feed
                self._callback_fault(error)
            self.spin_cpu = time.thread_time()

    def _callback_fault(self, error):
        """Surface a callback error once per kind and keep spinning; motion stays refused.

        The full traceback goes to FAULT_LOG, the dashboard gets its last frame.
        """
        frame = traceback.extract_tb(error.__traceback__)[-1:] or [None]
        where = f' at {os.path.basename(frame[0].filename)}:{frame[0].lineno}' if frame[0] else ''
        fault = f'{error}{where}'
        if fault in self._faults:
            return
        self._faults.add(fault)
        self.fault = fault
        try:
            with open(FAULT_LOG, 'a') as log:
                log.write(f'{time.strftime("%F %T")} {self.node.get_name()}\n'
                          + ''.join(traceback.format_exception(error)))
        except OSError:
            pass
        self.monitor.event('ROS fault: ' + fault)

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def _clock(self, message):
        # An episode reset rewinds /clock; one system marker per rewind (bags.md B14).
        if self.clock_watch.feed(message.clock.sec + message.clock.nanosec / 1e9):
            self.emit('marker', kind='system', note='sim reset')

    def _ingest(self, topic, kind, message, authority=''):
        tap = self.taps.get(kind)
        if tap is not None:
            tap(message)
        ingest(self.monitor, self.profile, topic, kind, message, self.now(),
               self.stale_after, authority)

    def _nearby(self, clouds, message):
        found = nearby.parse(message.data)
        if found is None:
            return
        self.nearby = (time.monotonic(), found)
        publish = getattr(self.facts, 'publish', None)
        nearby.observe(self.monitor, clouds, found, publish, lambda value, label: Fact(value, label))

    def tap(self, kind, callback):
        """A view's raw ``kind`` messages (points, image) while it is open: the subscription lives only as long as
        the tap; None ends it."""
        if callback is None:
            self.taps.pop(kind, None)
            for sub in self._tap_subs.pop(kind, ()):
                self.node.destroy_subscription(sub)
            return
        self.taps[kind] = callback
        if kind not in self._tap_subs:
            self._tap_subs[kind] = [self.node.create_subscription(
                MESSAGE_TYPES[kind], s.topic, partial(self._tapped, kind), qos_profile_sensor_data, raw=True)
                for s in self.profile.sensors if s.kind == kind]

    def _tapped(self, kind, message):
        tap = self.taps.get(kind)      # the view only: the sensor row keeps reading the summary
        if tap is not None:
            tap(message)

    def _gps_raw_int(self, topic, message):
        """GPS_RAW_INT from MAVROS's raw MAVLink stream as the gpsraw sensor, only while nothing
        publishes ``topic`` (no mavros_extras gpsstatus plugin), so RTK FLOAT/FIXED still shows."""
        # Only the autopilot's (MAV_COMP_ID_AUTOPILOT1): a GCS or another component must not supply the fix.
        if message.msgid != GPS_RAW_INT or message.compid != AUTOPILOT or self.node.count_publishers(topic):
            return
        raw = GpsRawInt(message)
        if not (raw.header.stamp.sec or raw.header.stamp.nanosec):
            raw.header.stamp = self.node.get_clock().now().to_msg()
        self._ingest(topic, 'gpsraw', raw)

    def _tf(self, message, info=None, static=False):
        # The publisher GID tells two publishers of the same edge apart.
        gid = getattr(info, 'publisher_gid', None) or (info or {}).get('publisher_gid', b'')
        self._ingest('/tf_static' if static else '/tf', 'tf_static' if static else 'tf',
                     message, bytes(getattr(gid, 'data', gid)).hex())

    def _driver_status(self, message):
        try:
            self.driver_status = json.loads(message.data)
        except ValueError:
            pass

    def _driver_output(self, message):
        try:
            self.driver_output = json.loads(message.data)
        except ValueError:
            return
        self.driver_output_at = time.monotonic()

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
        return collect(RclpyGraph(self.node), topics, time.monotonic(), full, self.ghosts())

    def subscribers(self, topic):
        return [e['node'] for e in self.endpoints(topic, True) if not e['self']]

    def watch_beats(self, nodes):
        """Subscribe once to each node's ``~/status`` (best effort, over the radio): its liveness beat. Returns the
        nodes subscribed now."""
        new = [node for node in nodes if node not in self._beat_subs]
        for node in new:
            self._beat_subs[node] = self.node.create_subscription(
                String, f'{node}/status', partial(self._beat, node), qos_profile_sensor_data)
        return new

    def _beat(self, node, message):
        now = time.monotonic()                     # the App's clock
        last = self.beat_at.get(node)
        if last is None or now - last > BEAT_STALE_S:
            self.beat_runs[node] = now             # a gap: its unbroken run starts again
        self.beat_at[node] = now
        try:                                       # a hold watchdog's {ok, reason}: whether its fault cleared
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.beat_state[node] = payload if isinstance(payload, dict) else None

    def beat_status(self):
        """``{node: its last ~/status payload}`` (a hold watchdog's ``{ok, reason}``), for the beats ever heard."""
        return dict(self.beat_state)

    def beats(self):
        """``{node: when its ~/status was last received}`` (monotonic), for the beats ever heard."""
        return dict(self.beat_at)

    def runs(self):
        """``{node: when its beats last began to arrive without a gap over BEAT_STALE_S}``."""
        return dict(self.beat_runs)

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

    def get_parameters(self, name, timeout=1.0):
        """Every parameter of node ``name`` as {name: value} (the Pose popup), or None when it does not answer."""
        from mhseals_testing.ros.params import Params
        self._params = getattr(self, '_params', None) or Params(self)
        try:
            return self._params.get(name, sorted(self._params.list_parameters(name, timeout)), timeout)
        except (TimeoutError, RuntimeError, ValueError):
            return None

    def close(self):
        self._spinning = False
        self.thread.join(timeout=1)
        self.executor.shutdown(timeout_sec=1)
        self.node.destroy_node()

