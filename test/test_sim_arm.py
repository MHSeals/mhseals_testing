"""Sim-only auto-arm (harness/sim_arm.py): every check refuses alone, a boat refuses, a real sim graph passes.

The desk side (run --sim-arm through the desk on fakes) is in test_agent_desk.py.
"""

from dataclasses import replace
import os
import threading
import time
from types import SimpleNamespace

import pytest

from mhseals_testing.harness import agent_config, sim_arm

ENV = {'ROS_DOMAIN_ID': '177', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
ON = dict(sim_arm.DEFAULTS, enabled=True)


def passing(directory):
    """Facts every check passes: the crane sim in a private-IPC container, domain 177."""
    return sim_arm.Facts(
        use_sim_time=True, outputs='topic', uses_fcu=False, muxed=True, remote=False, ground_truth='/crane/odom',
        env=dict(ENV), container=True, ipc='ipc:[4026533104]', clock_publishers=('/ros_tcp_endpoint',),
        sim_now=12.5, truth_publishers=('/ros_tcp_endpoint',),
        nodes=('/ros_tcp_endpoint', '/ekf_local', '/link_watchdog', '/fence_watchdog', '/costmap_watchdog'),
        hold_file=str(directory / sim_arm.HOLD_FILE), session=str(directory), holds=(), stop_file=False)


def failed(facts, config=ON):
    return [name for name, ok, _ in sim_arm.checks(facts, config) if not ok]


def test_the_crane_sim_passes_every_check(tmp_path):
    results = sim_arm.checks(passing(tmp_path), ON)
    assert [name for name, _, _ in results] == list(sim_arm.CHECKS) and not failed(passing(tmp_path))
    assert sim_arm.refusal(results) == '' and sim_arm.passed(results) == list(sim_arm.CHECKS)


ALONE = [  # (check, what changes); each alone must refuse with that check named first
    ('profile', dict(use_sim_time=False)), ('profile', dict(use_sim_time='true')),
    ('profile', dict(outputs='odroid')), ('profile', dict(outputs='none')), ('profile', dict(uses_fcu=True)),
    ('profile', dict(muxed=False)), ('profile', dict(remote=True)),
    ('domain', dict(env={**ENV, 'ROS_DOMAIN_ID': '42'})), ('domain', dict(env={**ENV, 'ROS_DOMAIN_ID': '0'})),
    ('domain', dict(env={**ENV, 'ROS_DOMAIN_ID': '233'})), ('domain', dict(env={**ENV, 'ROS_DOMAIN_ID': 'x'})),
    ('domain', dict(env={'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'})),
    ('discovery', dict(env={**ENV, 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'})),
    ('discovery', dict(env={'ROS_DOMAIN_ID': '177'})),
    ('discovery', dict(env={**ENV, 'ROS_STATIC_PEERS': '192.168.0.10'})),
    ('container', dict(container=False)),
    ('private_ipc', dict(ipc=sim_arm.INIT_IPC)), ('private_ipc', dict(ipc='')),
    ('clock', dict(clock_publishers=())), ('clock', dict(sim_now=0.0)),
    ('player', dict(truth_publishers=())), ('player', dict(ground_truth='/odom/local')),
    ('player', dict(ground_truth='')),
    ('no_hardware', dict(nodes=('/mavros',))), ('no_hardware', dict(nodes=('/boat/mavros_router',))),
    ('no_hardware', dict(nodes=('/zed/zed_node',))), ('no_hardware', dict(nodes=('/fcu_watchdog',))),
    ('no_hardware', dict(nodes=('/thruster_pwm_node',))), ('no_hardware', dict(nodes=('/command_mux',))),
    ('holds', dict(holds=('costmap_stale',))), ('holds', dict(hold_file='/tmp/mhseals/command_mux_177.json')),
    ('holds', dict(hold_file='')), ('holds', dict(session='')),
    ('stop_file', dict(stop_file=True))]


@pytest.mark.parametrize('check, change', ALONE, ids=[f'{c}-{i}' for i, (c, _) in enumerate(ALONE)])
def test_each_check_refuses_alone(tmp_path, check, change):
    facts = replace(passing(tmp_path), **change)
    assert failed(facts) == [check]
    assert sim_arm.refusal(sim_arm.checks(facts, ON)).startswith(f'sim-arm refused: {check}: ')


def test_off_in_agent_yaml_refuses(tmp_path):
    assert failed(passing(tmp_path), dict(ON, enabled=False)) == ['enabled']
    assert failed(passing(tmp_path), {}) == ['enabled']                 # the default is off


def test_no_config_makes_the_fleet_domain_a_sim(tmp_path):
    facts = replace(passing(tmp_path), env={**ENV, 'ROS_DOMAIN_ID': '42'})
    assert failed(facts, dict(ON, domains=[1, 232])) == ['domain']


def test_a_boat_like_environment_refuses():
    """Domain 42, MAVROS and the boat's drivers on the graph, no sim clock, the boat profile, host IPC."""
    boat = sim_arm.Facts(use_sim_time=False, outputs='odroid', uses_fcu=True, muxed=True, remote=False,
                         ground_truth='', env={'ROS_DOMAIN_ID': '42'}, container=True, ipc=sim_arm.INIT_IPC,
                         nodes=('/mavros', '/fcu_watchdog', '/zed_node', '/ekf_local'),
                         hold_file='/run/user/1000/mhseals/command_mux_42.json', session='/bags/sessions/s1',
                         stop_file=False)
    assert failed(boat) == ['profile', 'domain', 'discovery', 'private_ipc', 'clock', 'player', 'no_hardware',
                            'holds']
    assert sim_arm.refusal(sim_arm.checks(boat, ON)).startswith('sim-arm refused: profile: use_sim_time is off')


class Graph:
    def __init__(self, nodes=(), publishers=None, now=0.0):
        self._nodes, self._pubs, self._now = list(nodes), publishers or {}, now

    def nodes(self):
        return self._nodes

    def publishers(self, topic):
        return self._pubs.get(topic, [])

    def now(self):
        return self._now


def app_of(tmp_path, profile, ros, remote=None, hardware=None):
    config_path = tmp_path / 'sessions' / 's1' / 'config' / 'hardware.yaml'
    arming = SimpleNamespace(remote=remote, config_path=config_path)
    return SimpleNamespace(profile=profile, arming=arming, ros=ros,
                           args=SimpleNamespace(hardware_config=hardware or {'command_mux': {}}))


SIM = SimpleNamespace(use_sim_time=True, outputs='topic', uses_fcu=False, muxed=True, ground_truth='/crane/odom')


def test_gather_reads_the_live_app(tmp_path, monkeypatch):
    from mhseals_hardware import hold_store
    monkeypatch.setattr(sim_arm, 'in_container', lambda: True)
    monkeypatch.setattr(sim_arm, 'ipc_namespace', lambda: 'ipc:[4026533104]')
    ros = Graph(['/ros_tcp_endpoint'], {'/clock': ['/ros_tcp_endpoint'], '/crane/odom': ['/ros_tcp_endpoint']}, 3.0)
    app = app_of(tmp_path, SIM, ros)
    facts = sim_arm.gather(app, env=ENV)
    assert facts.hold_file == str(tmp_path / 'sessions' / 's1' / sim_arm.HOLD_FILE)
    assert not failed(facts)
    hold_store.save(facts.hold_file, {'keys': {'cause': 'keys', 'by': 'tui', 'since': 1.0, 'token': None}})
    assert failed(sim_arm.gather(app, env=ENV)) == ['holds']
    sim_arm.stop_path().parent.mkdir(parents=True, exist_ok=True)
    sim_arm.stop_path().touch()
    assert failed(sim_arm.gather(app, env=ENV)) == ['holds', 'stop_file']


def test_gather_on_a_boat_refuses(tmp_path, monkeypatch):
    """The boat's profile and graph through gather: pins, FCU, MAVROS, no /clock, the shared hold file."""
    monkeypatch.setattr(sim_arm, 'in_container', lambda: True)
    monkeypatch.setattr(sim_arm, 'ipc_namespace', lambda: sim_arm.INIT_IPC)
    boat = SimpleNamespace(use_sim_time=False, outputs='odroid', uses_fcu=True, muxed=True, ground_truth=None)
    ros = Graph(['/mavros', '/ekf_local', '/fcu_watchdog'], {'/odom/local': ['/ekf_local']}, 1.7e9)
    facts = sim_arm.gather(app_of(tmp_path, boat, ros), env={'ROS_DOMAIN_ID': '42'})
    assert {'profile', 'domain', 'private_ipc', 'clock', 'player', 'no_hardware', 'holds'} <= set(failed(facts))
    remote = sim_arm.gather(app_of(tmp_path, SIM, ros, remote=object()), env=ENV)
    assert {'profile', 'holds'} <= set(failed(remote))           # a remote (ssh) driver keeps the boat's hold file


def test_agent_yaml_ships_sim_arm_on_and_validates_it():
    shipped = agent_config.load()['sim_arm']
    assert shipped['enabled'] is True and shipped['domains'] == [101, 232] and 'mavros\\w*' in shipped['real_nodes']
    assert agent_config.sim_arm({}) == sim_arm.DEFAULTS and sim_arm.DEFAULTS['enabled'] is False
    for bad in ({'enabled': 'yes'}, {'max_s': 0}, {'domains': [1]}, {'domains': [100, 300]}, {'domains': [True, 5]},
                {'real_nodes': []}, {'real_nodes': ['(']}, {'release_holds': True}, []):
        with pytest.raises(ValueError, match='sim_arm'):
            agent_config.sim_arm(bad)


# --- App.agent_arm: the arm panel's path without its popup --------------------------------------

class ArmApp:
    """The App surface agent_arm reads; the mux reports ``mux`` once the driver starts."""

    def __init__(self, mux=None, refusal=''):
        self.review = self.watching = None
        self.source = SimpleNamespace(live=True)
        self.profile = SimpleNamespace(name='crane_full_stack', outputs='topic', muxed=True)
        self.paths = SimpleNamespace(hardware='/s/config/hardware.yaml')
        self.arming = object()
        self.calls, self.events, self.notice, self.refusal = [], [], '', refusal
        self.arbiter = SimpleNamespace(lock=threading.Lock(), mux=None, clock=lambda: 0.0)
        self.after = mux or {'input': 'auto'}
        self.plant = SimpleNamespace(engage=lambda: self.calls.append('engage'))
        self.ros = SimpleNamespace(emit=lambda phase, **f: self.events.append((phase, f)))
        self.armed = False

    def thruster_refusal(self):
        return ''

    def pin_refusal(self):
        return ''

    def watchdog_refusal(self):
        return self.refusal

    def wait_for_driver(self):
        self.calls.append('driver')
        self.armed, self.arbiter.mux = True, self.after

    def await_mux(self, arbiter, since, done):
        return arbiter.mux is not None

    def override_cause(self):
        return '' if self.arbiter.mux['input'] == 'auto' else 'hold'

    def release_hold(self):
        self.calls.append('release_hold')
        return True

    def disarm(self, reason='disarmed', by='operator'):
        self.calls.append(('disarm', reason, by))
        self.armed = False


@pytest.fixture
def App():
    return pytest.importorskip('mhseals_testing.ui.app').App


def test_agent_arm_starts_the_driver_and_journals_who(App):
    app = ArmApp()
    assert App.agent_arm(app, 'claude-autoarm', ['profile', 'domain']) == '' and app.armed
    assert app.calls == ['driver', 'engage']
    phase, fields = app.events[0]
    assert phase == 'armed' and fields['by'] == 'agent' and fields['who'] == 'claude-autoarm'
    assert fields['sim_arm'] == ['profile', 'domain']


def test_agent_arm_never_releases_a_restored_hold(App):
    app = ArmApp(mux={'input': 'hold', 'holds': [{'cause': 'costmap_stale', 'by': 'costmap_watchdog'}]})
    why = App.agent_arm(app, 'claude-autoarm', [])
    assert 'costmap_stale' in why and 'U' in why and not app.armed
    assert 'release_hold' not in app.calls and app.calls[-1][0] == 'disarm' and app.calls[-1][2] == 'agent'


def test_agent_arm_keeps_the_panels_refusals(App):
    app = ArmApp(refusal='costmap_watchdog is off the graph')
    assert App.agent_arm(app, 'claude-autoarm', []) == 'costmap_watchdog is off the graph'
    assert app.calls == [] and not app.armed
    app = ArmApp()
    app.watching = 4242
    assert App.agent_arm(app, 'claude-autoarm', []) == 'this session cannot arm' and app.calls == []


# --- the hook still blocks every direct arm ---------------------------------------------------

@pytest.mark.parametrize('command', [
    'ros2 run mhseals_hardware thruster_pwm_node --ros-args -p output_backend:=topic',
    'ros2 run mhseals_hardware command_mux', 'ros2 run mhseals_hardware calibrated_velocity_controller',
    'ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"',
    'ros2 run mavros mavsafety arm'])
def test_the_hook_blocks_direct_arming(command):
    from mhseals_testing import agent_hook as hook
    hit = hook.denied(command, hook.DEFAULTS)
    assert hit is not None
    if 'mhseals_hardware' in command:
        assert 'run KEY --sim-arm --claim ID' in hook.deny_reason(*hit, config=hook.DEFAULTS)


def test_the_playbook_names_sim_arm_as_sim_only():
    from mhseals_testing import agent_hook as hook
    text = hook.start_text('claude-autoarm', config=hook.DEFAULTS)
    assert '`run KEY --sim-arm`' in text and 'Sim only' in text and 'release a hold' in text


# --- a real ROS graph in the private-IPC test container -----------------------------------------

def in_private_container():
    return sim_arm.in_container() and sim_arm.ipc_namespace() not in ('', sim_arm.INIT_IPC)


class RclpyGraph:
    """publishers/nodes/now over one rclpy node, as ros/node.Ros answers them (own endpoints dropped)."""

    def __init__(self, node):
        self.node = node

    def publishers(self, topic):
        return [f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in self.node.get_publishers_info_by_topic(topic)
                if i.node_name != self.node.get_name()]

    def nodes(self):
        return [f'{ns.rstrip("/")}/{name}' for name, ns in self.node.get_node_names_and_namespaces()]

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9


@pytest.mark.skipif(not in_private_container(), reason='needs the private-IPC test container (ctest)')
def test_a_live_sim_graph_in_the_private_ipc_container_passes_and_mavros_refuses(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'     # test/ros_isolation, via conftest
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    try:
        player = rclpy.create_node('ros_tcp_endpoint', context=context)   # the crane player's endpoint
        clock, odom = player.create_publisher(Clock, '/clock', 10), player.create_publisher(Odometry, '/crane/odom', 10)
        tui = rclpy.create_node('mhseals_testing_probe', context=context,
                                parameter_overrides=[Parameter('use_sim_time', value=True)])
        for node in (player, tui):
            executor.add_node(node)
        graph, sim_t, deadline = RclpyGraph(tui), 10.0, time.monotonic() + 20

        def facts():
            pubs, now, truth, nodes = sim_arm.graph_facts(graph, '/crane/odom')
            return replace(passing(tmp_path), env=dict(os.environ), container=sim_arm.in_container(),
                           ipc=sim_arm.ipc_namespace(), clock_publishers=pubs, sim_now=now,
                           truth_publishers=truth, nodes=nodes)
        while time.monotonic() < deadline and failed(facts()):
            sim_t += 0.1
            clock.publish(Clock(clock=rclpy.time.Time(seconds=sim_t).to_msg()))
            odom.publish(Odometry())
            executor.spin_once(timeout_sec=0.1)
        results = sim_arm.checks(facts(), ON)
        assert sim_arm.refusal(results) == '', results
        assert dict((n, d) for n, _, d in results)['clock'] == '/clock from /ros_tcp_endpoint'
        executor.add_node(rclpy.create_node('mavros', context=context))      # a real FCU joins the domain
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not failed(facts()):
            executor.spin_once(timeout_sec=0.1)
        assert failed(facts()) == ['no_hardware']
        assert failed(replace(facts(), env={**facts().env, 'ROS_DOMAIN_ID': '42'})) == ['domain', 'no_hardware']
    finally:
        executor.shutdown()
        rclpy.shutdown(context=context)
