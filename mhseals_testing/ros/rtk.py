"""RTCM sink: one mavros_msgs/RTCM per base frame, published once with no acks."""

import json
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import RTCM
from std_msgs.msg import String

STATUS_TOPIC = '/mhseals_testing/rtk/status'

# MAVROS fragments up to 4 x 180 B; a longer message is dropped on its side with an error.
MAX_FRAME = 720

# MAVROS fragments in 180 B pieces and sends no empty terminator, so the FCU cannot tell where a
# 360 or 540 B frame ends and drops it. 720 B is ended by the full fourth fragment.
FRAGMENT = 180

# A frame older than this when it reaches us is worth less than the next epoch.
MAX_AGE = 2.0


class MavrosSink:
    """BaseSession sink: sink(frame, stamp) publishes the frame whole to MAVROS.

    No lifespan, ack wait or back-off: pacing and loss are MAVROS's and the
    FCU's. Staleness is judged here from the frame's read stamp instead.
    """

    def __init__(self, node, namespace='/mavros', clock=time.monotonic):
        self.node, self.clock = node, clock
        self.topic = namespace.rstrip('/') + '/gps_rtk/send_rtcm'
        self.publisher = node.create_publisher(
            RTCM, self.topic,
            QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE))
        self.counts = dict(sent=0, stale=0, oversize=0, split=0)
        self.warned, self.count = None, 0
        self.bytes, self.last = 0, None     # published payload bytes; clock stamp of the last frame

    def __call__(self, frame, stamp):
        n = self.node.count_publishers(self.topic)
        if self.warned is None or n != self.count:
            self.check_owner(n)
        if len(frame) > MAX_FRAME:
            self.counts['oversize'] += 1
            self.node.get_logger().warning(f'RTCM frame of {len(frame)} B dropped; MAVROS accepts {MAX_FRAME}')
            return
        if self.clock() - stamp > MAX_AGE:
            self.counts['stale'] += 1
            return
        frame = bytes(frame)
        if FRAGMENT < len(frame) < MAX_FRAME and len(frame) % FRAGMENT == 0:
            # The FCU gets a byte stream, so a 1 B tail ends the last fragment short and in order
            self.counts['split'] += 1
            parts = [frame[:-1], frame[-1:]]
        else:
            parts = [frame]
        for part in parts:
            message = RTCM()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.data = list(part)
            self.publisher.publish(message)
        self.counts['sent'] += 1
        self.bytes += len(frame)
        self.last = self.clock()

    def check_owner(self, n):
        """Warn once per other live node publishing corrections; two senders interleave frames."""
        self.count, self.warned = n, self.warned or set()
        if n <= 1:
            return
        # We own exactly one endpoint, so drop one entry matching our own name (others may share it)
        others = [(i.node_name, i.node_namespace)
                  for i in self.node.get_publishers_info_by_topic(self.topic)]
        me = (self.node.get_name(), self.node.get_namespace())
        if me in others:
            others.remove(me)
        new = set(others) - self.warned
        self.warned |= new
        if new:
            names = ', '.join(sorted(name for name, _ in new))
            self.node.get_logger().warning(
                f'another RTCM publisher is active ({names}); stop it to avoid mixed corrections')

    def close(self):
        self.node.destroy_publisher(self.publisher)


class StatusPublisher:
    """Latched std_msgs/String JSON of StatusTracker.status() at 1 Hz."""

    def __init__(self, node, tracker, topic=STATUS_TOPIC):
        self.tracker = tracker
        self.publisher = node.create_publisher(
            String, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(1.0, self.publish)

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.tracker.status())))


# The rover's fix for NTRIP picking and the GGA uplink: stock MAVROS publishes it, no extras needed.
ROVER_FIX = '/mavros/global_position/raw/fix'


def serve(config, survey=False, position=None):
    """Own the node: local base and/or NTRIP -> Corrections -> MavrosSink (+ Radio), status at 1 Hz."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from mhseals_testing import ntrip
    from mhseals_testing.corrections import Corrections
    from mhseals_testing.rtk import BaseSession, Radio, StatusTracker
    rclpy.init()
    node = rclpy.create_node('rtk_base')
    sink = MavrosSink(node)
    sinks = [sink] + ([Radio(config['radio'], config['radio_baud'])] if config['radio'] else [])
    settings = ntrip.load_settings(config)
    mux = Corrections(sinks, config['priority'], settings['near_km'], config=config)
    rough = ntrip.Position(settings['position_cache'], settings['fallback'])
    base = BaseSession([mux.gate('base')], config) if config['device'] else None
    try:
        net = (ntrip.NtripSource(settings, mux.gate('ntrip'), rough)
               if settings['enabled'] and settings['casters'] else None)
    except PermissionError as error:
        node.get_logger().warning(f'NTRIP disabled: {error}')
        net = None
    mux.attach(base, net, rough)

    def on_fix(msg):
        if msg.status.status >= 0:      # STATUS_NO_FIX is -1
            rough.offer('rover', msg.latitude, msg.longitude, msg.altitude)

    node.create_subscription(NavSatFix, ROVER_FIX, on_fix, qos_profile_sensor_data)
    node.create_timer(1.0, mux.evaluate)
    StatusPublisher(node, StatusTracker(mux, sink, lambda: node.count_publishers(sink.topic)))
    if base:
        base.start(survey=survey, position=position, auto=config['auto_site'])
    if net:
        net.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for part in (base, net):
            if part:
                part.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
