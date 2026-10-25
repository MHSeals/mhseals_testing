"""Subscribe the pre-flight sources on a node and publish the ``preflight`` and ``graph`` facts.

Read only and cheap: small latched or low-rate topics on this host's graph (the driver
status, /mavros/rc/in, the GPS raw status, the camera host's ``host:`` diagnostic) plus graph
queries and a sysfs PWM read. The TUI and ``boat_trial check`` share it, so both show the
same facts. See health/probes/preflight.py for the fact names.
"""

import copy
import json
import socket
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from mhseals_testing.health import probes
from mhseals_testing.health.probes import graph, preflight
from mhseals_testing.health.probes.fcu import FcuProbe
from mhseals_testing.health.registry import KINDS
from mhseals_testing.ros.graph import RclpyGraph

DRIVER = 'thruster_pwm_node'
STATUS_TOPIC = f'/{DRIVER}/status'
RC_TOPIC = '/mavros/rc/in'
GPS_RAW_TOPIC = '/mavros/gpsstatus/gps1/raw'
DIAGNOSTICS = '/diagnostics'
HOST_SILENCE_S = 6.0            # three host_diagnostic periods; older camera-host readings drop


def watched_topics(specs):
    """The topics the graph probe must sample for ``specs`` (each kind's ``watch``)."""
    return sorted({t for s in specs for t in KINDS[s.kind].watch(s.params).get('graph', ())})


class Preflight:
    """Callbacks store the latest values; ``facts(now)`` turns them into facts."""

    def __init__(self, profile, hardware, registry=None, clock=time.monotonic, local=None, status=None):
        self.profile, self.registry, self.clock = profile, registry, clock
        self._hardware = hardware if callable(hardware) else (lambda: hardware)   # the TUI edits it live
        self._status = status                    # the TUI's RosIO.driver_status; None: on_status fills it
        self.status = None
        self._mux = None
        self._rc = None
        self.gps = FcuProbe()
        self.host = None                         # (receipt, hostname, values)
        self.local = local or socket.gethostname()

    @property
    def hardware(self):
        return self._hardware() or {}

    @property
    def rc(self):
        """The RC link judged by the live ``command_mux``; rebuilt (frames dropped) when it changes."""
        if not self.profile.uses_fcu:
            return None
        mux = self.hardware.get('command_mux')
        if self._rc is None or mux != self._mux:
            self._mux, self._rc = copy.deepcopy(mux), preflight.RcLink(mux)
        return self._rc

    @property
    def driver_status(self):
        return self._status() if self._status else self.status

    def on_status(self, message):
        try:
            self.status = json.loads(message.data)
        except ValueError:
            self.status = None

    def on_rc(self, message):
        self.rc.frame(message.channels, self.clock())

    def on_gps(self, message):
        h_acc = message.h_acc / 1000 if getattr(message, 'h_acc', 0) else None    # mm
        self.gps.on_gps_raw(message.fix_type, message.satellites_visible, h_acc, self.clock())

    def on_diagnostics(self, message):
        for status in message.status:
            host = status.hardware_id[len(preflight.HOST_PREFIX):]
            if status.hardware_id.startswith(preflight.HOST_PREFIX) and host != self.local:
                self.host = (self.clock(), host, {kv.key: kv.value for kv in status.values})

    def facts(self, now, nodes):
        """Every fact; the driver's only while its node is on the graph (its status is latched)."""
        out = {}
        if any(n.rsplit('/', 1)[-1] == DRIVER for n in nodes):
            out.update(preflight.driver_facts(self.driver_status, self.hardware))
        rc = self.rc
        if rc is not None:
            out.update(rc.facts(now))
        out.update({k: v for k, v in self.gps.facts(now).items() if k.startswith('gps.')})
        if self.host and now - self.host[0] <= HOST_SILENCE_S:
            out.update(preflight.orin_facts(self.host[2], self.host[1]))
        if self.registry is not None:
            from mhseals_testing.harness.registry import alive
            out.update(preflight.bag_facts(self.registry.children(), alive))
        if self.profile.outputs == 'odroid' and self.hardware.get('pwm_chips'):
            from mhseals_testing.health.legacy import pwm_readback
            armed = any(n.rsplit('/', 1)[-1] == DRIVER for n in nodes)    # the driver runs only armed
            level, detail, _ = pwm_readback(self.hardware['pwm_chips'], self.hardware['pwm_channels'],
                                            armed, self.hardware.get('frequency', 50.0))
            out['pwm.readback'] = probes.Fact(level, data={'detail': detail})
        return out


def subscribe_preflight(node, facts, profile, hardware, specs, periods, registry=None,
                        ghosts=lambda: (), status=None):
    """Subscribe on ``node`` and publish ``preflight`` and ``graph`` facts on timers.

    ``specs`` set the topics the graph probe samples; ``ghosts()`` names stopped children whose
    nodes may linger for a DDS lease (see probes/graph.collapse). ``status()`` returns the
    driver's latest ~/status dict when the caller already subscribes it (the TUI's RosIO);
    without it this subscribes.
    """
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String
    probe = Preflight(profile, hardware, registry, status=status)
    if status is None:
        node.create_subscription(String, STATUS_TOPIC, probe.on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(DiagnosticArray, DIAGNOSTICS, probe.on_diagnostics, 10)
    if profile.uses_fcu:
        try:
            from mavros_msgs.msg import GPSRAW, RCIn
            node.create_subscription(RCIn, RC_TOPIC, probe.on_rc, qos_profile_sensor_data)
            node.create_subscription(GPSRAW, GPS_RAW_TOPIC, probe.on_gps, qos_profile_sensor_data)
        except ImportError:
            pass                                 # no mavros_msgs: the RC and GPS facts stay unknown
    source = RclpyGraph(node)
    topics = watched_topics(specs)

    def tick():
        try:
            facts.publish('preflight', probe.facts(time.monotonic(), source.node_names()))
        except Exception as error:               # an odd message must not stop the timer
            facts.publish('preflight', {}, error=repr(error))

    def sample_graph():
        try:
            facts.publish('graph', graph.collect(source, topics, time.monotonic(), True, ghosts()))
        except Exception as error:
            facts.publish('graph', {}, error=repr(error))

    node.create_timer(float(periods.get('preflight', 1)), tick)
    node.create_timer(float(periods.get('graph', 2)), sample_graph)
    return probe


def subscribe_tui(ros, facts, profile, hardware, registry):
    """The TUI's wiring: the Preflight group's graph topics, and the session's own registry."""
    from mhseals_testing.health import config
    loaded = config.load(profile=profile)
    periods = {n: s['period'] for n, s in loaded.probes.items() if isinstance(s, dict) and 'period' in s}
    specs = [s for s in loaded.specs if s.group == 'Preflight']
    return subscribe_preflight(ros.node, facts, profile, hardware, specs, periods, registry,
                               lambda: ros.ghosts(), lambda: ros.driver_status)


def cli_collector(profile, config, specs, host_collect):
    """(collect, shutdown) for ``boat_trial check``: the TUI's ROS probes on a short-lived node.

    ``host_collect`` is the ROS-free collector (host facts); its store gets the FCU, battery,
    perception, preflight, graph and calibration facts too, so one read serves the engine.
    """
    from types import SimpleNamespace

    from mhseals_hardware.configuration import load_config

    from mhseals_testing.harness.registry import Registry
    from mhseals_testing.health.advisories import Health
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.nav2_live import cli_node
    from mhseals_testing.ros.perception import subscribe_detections

    hardware = load_config()
    store = host_collect.store
    periods = {n: s['period'] for n, s in config.probes.items() if isinstance(s, dict) and 'period' in s}
    node, shutdown = cli_node('boat_trial_check')
    ros = SimpleNamespace(node=node, power_facts=None)
    subscribe_detections(ros, store, profile)
    if profile.uses_fcu:
        subscribe_fcu(ros, SimpleNamespace(advise=lambda *a: None), store, profile.battery)
    subscribe_preflight(node, store, profile, hardware, specs, periods, Registry())
    owner = SimpleNamespace(hardware=lambda: hardware)

    def collect():
        try:
            store.publish('calibration', Health.calibration(owner))     # the TUI's own probe
        except Exception as error:
            store.publish('calibration', {}, error=repr(error))
        return host_collect()
    return collect, shutdown
