"""The ROS side of the live Nav2 apply (harness/nav2_live.py).

Parameter clients, mux and authority state. ``RosPorts`` works on any spinning
rclpy node: the TUI passes its RosIO node, ``boat_trial config apply nav2``
gets its own through ``cli_node``. It subscribes once, when it is built, so the
latched authority, mux and goal-status samples and the /rosout buffer are in
place by the time a write is asked for (build it when the preview opens).
"""

import json
import re
import threading
import time
from collections import deque
from types import SimpleNamespace

from mhseals_testing.harness import journal as journal_file
from mhseals_testing.harness.authority import is_unknown
from mhseals_testing.harness.nav2_live import Ports
from mhseals_testing.ros.authority import AUTHORITY_TOPIC, MUX_STATE_TOPIC
from mhseals_testing.ros.params import Params

MUX_REQUEST_TOPIC = '/command_mux/request'
GOAL_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
LIVE_GOAL = (1, 2, 3)           # GoalStatus ACCEPTED, EXECUTING, CANCELING
BAD_LOG = ('not found', 'invalid')
ERROR_LEVEL = 40                # rcl_interfaces/Log ERROR; FATAL is 50
FIRST_SAMPLE_S = 4.0
# The MPPI parameter handler is called for every parameter of
# controller_server and warns "Parameter <name> not found" for each one it
# does not own. Measured on Nav2 1.3.13 (parameters_handler.hpp registers no
# callback for static parameters, so for MPPI's own keys the line is the only
# sign a key took no effect). The window runs only for live keys, all
# FollowPath.*; only the two rotation-shim names among them are MPPI's noise.
SHIM_NAMES = frozenset(('max_angular_accel', 'rotate_to_heading_angular_vel'))
MPPI_NOISE = re.compile(r'Parameter (\S+) not found')


class Rosout:
    """The /rosout lines seen since ``start`` that say a parameter failed."""

    def __init__(self, buffer, start):
        self.buffer, self.start = buffer, start

    @staticmethod
    def exempt(name, text, params):
        """Say whether ``text`` is MPPI's not-found noise for a key it cannot own."""
        noise = MPPI_NOISE.fullmatch(text)
        if name != 'controller_server' or not noise or noise[1] not in params:
            return False
        return noise[1].removeprefix('FollowPath.') in SHIM_NAMES

    def errors(self, nodes, params):
        """Return the ERROR/FATAL lines of any node that logs to /rosout and
        the ``not found``/``Invalid`` lines about what was set.

        A library logger such as nav2_costmap_2d never reaches /rosout (see
        harness.nav2_live.footprint_problem for the one that matters).
        """
        found = []
        for stamp, name, text, level in list(self.buffer):
            low = text.lower()
            if stamp < self.start:
                continue
            if level >= ERROR_LEVEL:
                found.append(f'{name}: {text}')
                continue
            if not any(b in low for b in BAD_LOG):
                continue
            about = (any(name.startswith(node) for node in nodes)
                     or any(param in text for param in params))
            if not about or self.exempt(name, text, params):
                continue
            found.append(f'{name}: {text}')
        return found

    def close(self):
        """Stop collecting (the buffer is shared and keeps running)."""


class RosPorts(Ports):
    """The live world, as seen from one rclpy node."""

    def __init__(self, node, registry, journal, mux_expected=lambda: False,
                 clock=time.monotonic):
        from action_msgs.msg import GoalStatusArray
        from rcl_interfaces.msg import Log
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)
        from std_msgs.msg import String
        self.node, self.registry, self.clock = node, registry, clock
        self.journal_to, self.expected = journal, mux_expected
        self.params = Params(SimpleNamespace(node=node))
        self.samples, self.status = {}, {}
        self.log = deque(maxlen=2000)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, topic in (('authority', AUTHORITY_TOPIC),
                           ('mux', MUX_STATE_TOPIC)):
            node.create_subscription(
                String, topic, lambda m, k=key: self._sample(k, m), latched)
        for action in GOAL_ACTIONS:
            node.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                lambda m, a=action: self.status.__setitem__(a, m), latched)
        node.create_subscription(
            Log, '/rosout',
            lambda m: self.log.append(
                (self.clock(), m.name, m.msg, m.level)),
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.publisher = node.create_publisher(String, MUX_REQUEST_TOPIC, 10)

    def _sample(self, key, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict):
            self.samples[key] = (value, self.clock())

    def _fresh(self, key):
        """Return the newest latched sample if recent; wait for the first."""
        deadline = self.clock() + FIRST_SAMPLE_S
        while key not in self.samples and self.clock() < deadline:
            time.sleep(0.05)
        if key not in self.samples:
            return None
        value, at = self.samples[key]
        stale = is_unknown(value, time.time(), self.clock() - at)
        return None if stale else value

    def owner(self):
        """Return the registry owner while its process is alive."""
        owner = self.registry.owner() if self.registry else None
        alive = owner and self.registry.alive(owner['pid'])
        return owner if alive else None

    def authority(self):
        """Return the authority record published by the session owner."""
        return self._fresh('authority')

    def mux(self):
        """Return the command_mux state."""
        return self._fresh('mux')

    def mux_expected(self):
        """Say whether the mux must be up (muxed profile, armed)."""
        return bool(self.expected())

    def mux_request(self, request):
        """Publish a request on the mux request topic."""
        from std_msgs.msg import String
        self.publisher.publish(String(data=json.dumps(request)))

    def goals(self):
        """Return the navigation goals that are running."""
        time.sleep(0.2)           # a just-built subscription takes its sample
        live = []
        for action in GOAL_ACTIONS:
            if not self.node.count_publishers(f'/{action}/_action/status'):
                continue          # no server: the kept sample is not a goal
            statuses = getattr(self.status.get(action), 'status_list', ())
            for status in statuses:
                if status.status in LIVE_GOAL:
                    uuid = bytes(status.goal_info.goal_id.uuid).hex()
                    live.append(f'{action}:{uuid[:8]}')
        return live

    def read(self, node):
        """Return every parameter of ``node`` and the names it declares."""
        names = self.params.list_parameters(node)
        return self.params.get(node, sorted(names)), names

    def lifecycle(self, command):
        """Run ``reset`` or ``startup`` on every managed Navigation server."""
        return self.params.manage_nodes(command)

    def get(self, node, names):
        """Return the values of ``names`` on ``node``."""
        return self.params.get(node, names)

    def put(self, node, values):
        """Set ``values`` on ``node`` in one atomic request."""
        return self.params.set_atomically(node, values)

    def rosout_open(self):
        """Return a view of the /rosout lines from now on."""
        return Rosout(self.log, self.clock())

    def journal(self, phase, **fields):
        """Write one event to the session journal."""
        self.journal_to(phase, **fields)


def session_journal(registry, node_name='boat_trial'):
    """Return a journal writer for a guest: the owner's events.jsonl.

    It raises OSError when the registry names no session.
    """
    from mhseals_testing import events

    def write(phase, **fields):
        session = (registry.state() or {}).get('session')
        if not session:
            raise OSError('the registry names no session journal')
        journal_file.append(f'{session}/events.jsonl',
                            events.payload(phase, 0, fields, node=node_name))
    return write


def cli_node(name='boat_trial_config'):
    """Return (node, shutdown): an rclpy node spun on its own thread."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = rclpy.create_node(name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def shutdown():
        executor.shutdown()
        thread.join(timeout=2.0)   # a live executor thread aborts at exit
        node.destroy_node()
        rclpy.try_shutdown()
    return node, shutdown
