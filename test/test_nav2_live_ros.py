"""CFG-NAV-LIVE1 against a real throwaway Nav2 stack (sim).

controller_server (rotation shim + MPPI), planner_server and velocity_smoother run on
mhseals_nav's own nav2_params.yaml, with the real command_mux and the real authority arbiter.
Skipped without ROS, Nav2 and mhseals_hardware. It starts its own processes on a private ROS
domain (``MHSEALS_TEST_DOMAIN``, else the caller's ``ROS_DOMAIN_ID``, else a pid-derived one in
120-219) restricted to localhost, and refuses to start on a domain where other nodes already run
(their mux or arbiter would answer on the same topics), and locks the domain against a
concurrent run. It stops only the process groups it started. The independent evidence is
``ros2 param get`` in a separate process.
"""

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
pytest.importorskip('mhseals_hardware.command_mux')

from mhseals_testing.harness.nav2_live import Nav2Live  # noqa: E402
from mhseals_testing.ros.nav2_live import RosPorts  # noqa: E402

from nav_paths import find_nav  # noqa: E402

FOLLOW = 'controller_server.FollowPath.'
SMOOTHER = '/velocity_smoother'
CONTROLLER = '/controller_server'
OWN_ENV = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'XDG_RUNTIME_DIR')


class Stack:
    """The processes, the in-process node and the helpers the tests share."""

    def __init__(self, directory, domain, nav):
        self.dir, self.domain, self.nav = directory, domain, nav
        self.env = {**os.environ, 'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                    'XDG_RUNTIME_DIR': str(directory / 'run'),
                    'MHSEALS_DATA': str(directory / 'data'),
                    'MHSEALS_NAV2': str(directory / 'nav2.yaml'),
                    'XDG_CONFIG_HOME': str(directory / 'config')}
        (directory / 'run').mkdir()
        self.procs = {}
        self.journal = []

    def start(self, name, *command):
        log = open(self.dir / f'{name}.log', 'w')
        self.procs[name] = subprocess.Popen(command, env=self.env, stdout=log,
                                            stderr=subprocess.STDOUT, start_new_session=True)

    def child(self, name):
        """Return the pid of the real binary under a ``ros2 run`` wrapper."""
        wrapper = str(self.procs[name].pid)
        found = subprocess.run(['pgrep', '-P', wrapper], capture_output=True, text=True)
        out = found.stdout.split()
        return int(out[0])

    def stop(self):
        for proc in self.procs.values():
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in self.procs.values():
            try:
                proc.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(5)

    def get(self, node, name):
        """Return a parameter read by a separate ``ros2 param get`` process."""
        command = ['ros2', 'param', 'get', '--spin-time', '3', node, name]
        done = subprocess.run(command, env=self.env, capture_output=True, text=True, timeout=30)
        assert done.stdout.strip(), f'ros2 param get {node} {name}: {done.stderr}'
        out = done.stdout.strip().splitlines()[-1]
        value = out.partition(': ')[2]
        if 'values are' in out:
            return [float(x) for x in re.findall(r'-?\d+(?:\.\d+)?(?:e-?\d+)?', value)]
        return float(value) if re.fullmatch(r'-?[\d.e-]+', value) else value

    def mux_holds(self):
        return [h['cause'] for h in (self.ports.mux() or {}).get('holds', [])]


class Ports(RosPorts):
    """RosPorts that can corrupt, observe or freeze what is sent, so the real nodes fail."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rewrite, self.log_after, self.freeze = {}, {}, None
        self.no_mux, self.rosout = False, None
        self.sets, self.held_during = [], []
        self.commands, self.startup_fails = [], 0
        self.order = []                      # every set and lifecycle command, in the order sent

    def put(self, node, values):
        self.sets.append(node)
        self.order.append(('set', node))
        self.held_during.append([h['cause'] for h in (self.mux() or {}).get('holds', [])])
        if self.freeze and node == SMOOTHER:
            os.kill(self.freeze, signal.SIGSTOP)    # silent between the preview and the write
        result = super().put(node, self.rewrite.pop(node, values))
        if node in self.log_after:
            line = self.log_after[node]
            if isinstance(line, tuple):
                self.say(*line)
            else:
                self.node.get_logger().error(line)
        return result

    def lifecycle(self, command):
        self.commands.append(command)
        self.order.append((command, None))
        done = super().lifecycle(command)
        if command == 'startup' and self.startup_fails:
            self.startup_fails -= 1
            return False                         # as if the manager reported failure
        return done

    def say(self, name, text):
        """Publish a /rosout line as if node ``name`` had logged it."""
        from rcl_interfaces.msg import Log
        if self.rosout is None:
            self.rosout = self.node.create_publisher(Log, '/rosout', 10)
            time.sleep(0.5)
        self.rosout.publish(Log(name=name, msg=text, level=30))
        time.sleep(0.3)

    def mux(self):
        return None if self.no_mux else super().mux()


def start_nav2(run, params):
    """Start the throwaway Nav2 servers, their lifecycle manager, the TFs and the real mux."""
    ros2 = ['ros2', 'run']
    run.start('tf_odom', *ros2, 'tf2_ros', 'static_transform_publisher', '--frame-id', 'odom',
              '--child-frame-id', 'base_link')
    run.start('tf_map', *ros2, 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map',
              '--child-frame-id', 'odom')
    run.start('controller', *ros2, 'nav2_controller', 'controller_server', '--ros-args',
              '--params-file', params, '-r', 'cmd_vel:=cmd_vel_nav')
    run.start('smoother', *ros2, 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args',
              '--params-file', params, '-r', 'cmd_vel:=cmd_vel_nav',
              '-r', 'cmd_vel_smoothed:=cmd_vel')
    run.start('planner', *ros2, 'nav2_planner', 'planner_server', '--ros-args',
              '--params-file', params)
    run.start('manager', *ros2, 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
              '-r', '__node:=lifecycle_manager_navigation', '-p', 'autostart:=true',
              '-p', 'bond_timeout:=0.0',
              '-p', 'node_names:=[controller_server, planner_server, velocity_smoother]')
    run.start('mux', sys.executable, '-m', 'mhseals_hardware.command_mux')


def claim_domain(first):
    """Lock a ROS domain for this module; a concurrent run in the container never shares it.

    The caller's domain is tried first; a locked one is skipped for the next free one in 120-219.
    """
    import fcntl
    for domain in (first, *range(120, 220)):
        handle = open(f'/tmp/mhseals-nav2-live-domain-{domain}.lock', 'w')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return domain, handle
        except OSError:
            handle.close()
    pytest.skip('every ROS test domain is locked by another run')


@pytest.fixture(scope='module')
def stack(tmp_path_factory):
    nav = find_nav()
    params = nav / 'config' / 'nav2_params.yaml' if nav else None
    if not (shutil.which('ros2') and params and params.is_file()):
        pytest.skip('needs ros2 and mhseals_nav')
    domain, lock = claim_domain(int(os.environ.get('MHSEALS_TEST_DOMAIN')
                                    or os.environ.get('ROS_DOMAIN_ID') or 120 + os.getpid() % 100))
    run = Stack(tmp_path_factory.mktemp('nav2live'), domain, nav)
    probe = subprocess.run(['ros2', 'pkg', 'prefix', 'nav2_controller'], env=run.env,
                           capture_output=True)
    if probe.returncode:
        lock.close()
        pytest.skip('nav2_controller is not installed')
    saved = {key: os.environ.get(key) for key in run.env}
    os.environ.update({key: run.env[key] for key in OWN_ENV})
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('nav2_live_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        time.sleep(1.5)                      # graph discovery: who else is on this domain
        others = sorted(n for n in set(node.get_node_names())
                        if n != 'nav2_live_test' and not n.startswith('_ros2cli'))
        assert not others, f'ROS domain {domain} is in use by {others}; set MHSEALS_TEST_DOMAIN'
        start_nav2(run, str(params))
        from mhseals_testing.harness.registry import Registry
        from mhseals_testing.ros import authority as authority_module
        registry = Registry()
        registry.claim({'pid': os.getpid(), 'node': 'nav2_live_test', 'host': 'test'},
                       session=str(run.dir))
        run.registry = registry
        run.state = {'procedure': None}
        graph = {'cmd_vel': {'age_s': None, 'nonzero': False}, 'publishers': [], 'own_gids': []}
        run.arbiter = authority_module.Arbiter(
            lambda record: None, lambda phase, **fields: None, registry=None, tui_owner=True,
            sources=lambda: {'procedure': run.state['procedure'], 'task': None, 'nav': None,
                             'graph': graph, 'mux_expected': True, 'muxed': True})
        authority_module.bind(SimpleNamespace(node=node), run.arbiter)
        run.ports = Ports(node, registry,
                          lambda phase, **fields: run.journal.append((phase, fields)),
                          mux_expected=lambda: True)
        run.node, run.context = node, context
        deadline = time.monotonic() + 120
        while True:
            try:
                if run.ports.params.is_active(timeout=2.0):
                    break
            except TimeoutError:
                pass
            if time.monotonic() > deadline:
                log = (run.dir / 'manager.log').read_text()[-1500:]
                pytest.fail('the throwaway Nav2 did not become active:\n' + log)
            time.sleep(1.0)
        deadline = time.monotonic() + 30
        # The arbiter says 'unknown' until the first mux state reaches it.
        while time.monotonic() < deadline:
            record = run.ports.authority()
            if record and run.ports.mux() and record['owner'] != 'unknown':
                break
            time.sleep(0.5)
        assert run.ports.mux() is not None, 'the real command_mux published no state'
        assert run.ports.authority()['owner'] == 'none'
        yield run
    finally:
        run.stop()
        executor.shutdown()
        spin.join(3)
        node.destroy_node()
        rclpy.shutdown(context=context)
        lock.close()
        for key, value in saved.items():
            if key in OWN_ENV:
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


@pytest.fixture
def live(stack):
    """Yield a Nav2Live on the real ports; the original values are put back afterwards."""
    ports = stack.ports
    for scratch in (ports.rewrite, ports.log_after, ports.sets, ports.held_during, ports.commands,
                    ports.order):
        scratch.clear()
    ports.freeze, ports.no_mux, ports.startup_fails = None, False, 0
    stack.journal.clear()
    stack.state['procedure'] = None
    keys = ['FollowPath.vx_max', 'FollowPath.wz_max', 'failure_tolerance']
    original = {CONTROLLER: ports.get(CONTROLLER, keys),
                SMOOTHER: ports.get(SMOOTHER, ['max_velocity'])}
    yield Nav2Live(ports, me=os.getpid())
    for node, values in original.items():
        stack.ports.params.set_atomically(node, values)


def ask(**keys):
    return {FOLLOW + name: value for name, value in keys.items()}


def test_a_live_set_reaches_the_real_nodes_under_a_tokened_hold(stack, live):
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'verified', receipt
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.25
    assert stack.get(SMOOTHER, 'max_velocity')[2] == 0.25           # the derived limit moved too
    assert stack.ports.sets == [SMOOTHER, CONTROLLER]               # smoother first, restrictive
    assert stack.ports.held_during == [['config'], ['config']]      # held before each write
    assert stack.mux_holds() == [] and receipt['hold']['released'] and receipt['hold']['needed']
    assert [phase for phase, _ in stack.journal] == ['nav2_live_intent', 'nav2_live_outcome']
    mux = stack.ports.mux()
    assert mux['last_release']['tokenless'] is False and mux['last_release']['cause'] == 'config'


def test_a_cost_weight_and_inflation_apply_live_with_readback_and_no_reset(stack, live):
    node = '/global_costmap/global_costmap'
    weight, radius = 'FollowPath.CostCritic.cost_weight', 'inflation_layer.inflation_radius'
    before = {weight: stack.get(CONTROLLER, weight), radius: stack.get(node, radius)}
    starts = activations(stack)
    try:
        receipt = live.apply({FOLLOW + 'CostCritic.cost_weight': 4.5,
                              'global_costmap.global_costmap.' + radius: 1.2})
        assert receipt['status'] == 'verified' and not receipt['reset'], receipt
        assert stack.ports.commands == [] and activations(stack) == starts
        assert stack.ports.sets == [CONTROLLER, node]
        assert stack.get(CONTROLLER, weight) == 4.5 and stack.get(node, radius) == 1.2
        assert stack.mux_holds() == [] and receipt['hold']['released']
    finally:
        stack.ports.params.set_atomically(CONTROLLER, {weight: before[weight]})
        stack.ports.params.set_atomically(node, {radius: before[radius]})


def test_a_live_footprint_reaches_the_costmap_with_no_reset(stack, live):
    node, key = '/global_costmap/global_costmap', 'footprint'
    before = stack.get(node, key)
    shape = '[[0.6, 0.45], [0.6, -0.45], [-0.6, -0.45], [-0.6, 0.45]]'
    try:
        receipt = live.apply({'global_costmap.footprint': shape})
        assert receipt['status'] == 'verified' and not receipt['reset'], receipt
        assert stack.ports.commands == [] and stack.get(node, key) == shape
    finally:
        stack.ports.params.set_atomically(node, {key: before})


def test_a_reset_path_key_survives_the_mppi_parameter_warning(stack, live):
    receipt = live.apply({'controller_server.failure_tolerance': 0.25})
    assert receipt['status'] == 'verified', receipt
    assert stack.get(CONTROLLER, 'failure_tolerance') == 0.25


def test_an_integer_reaches_a_double_parameter_after_coercion(stack, live):
    assert stack.ports.params.set_atomically(CONTROLLER, {'failure_tolerance': 1}) == (True, '')
    assert stack.get(CONTROLLER, 'failure_tolerance') == 1.0
    with pytest.raises(ValueError):
        stack.ports.params.set_atomically(CONTROLLER, {'failure_tolerance': 'fast'})


def test_a_costmap_key_the_real_node_rejects_stops_before_the_reset(stack, live):
    stack.ports.rewrite['/global_costmap/global_costmap'] = {'width': -5}   # Nav2 says no
    receipt = live.apply({**ask(wz_max=0.25), 'global_costmap.global_costmap.width': 50})
    assert receipt['status'] == 'rolled_back' and 'rejected the set' in receipt['error'], receipt
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3
    assert stack.get('/global_costmap/global_costmap', 'width') == 100.0
    assert stack.ports.commands == [] and stack.mux_holds() == []


def test_a_costmap_key_is_set_before_the_reset_that_stops_serving_it(stack, live):
    node = '/global_costmap/global_costmap'
    receipt = live.apply({**ask(wz_max=0.25), 'global_costmap.global_costmap.width': 50})
    try:
        assert receipt['status'] == 'verified' and receipt['reset'], receipt
        assert stack.ports.order[0] == ('set', node) and stack.ports.order[1] == ('reset', None)
        assert stack.get(node, 'width') == 50.0
        assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.25
    finally:
        stack.ports.params.set_atomically(node, {'width': 100})
        live.apply({'global_costmap.global_costmap.width': 100, **ask(wz_max=0.3)})


def test_a_bad_rosout_line_rolls_back_the_real_nodes(stack, live):
    stack.ports.log_after[CONTROLLER] = 'Invalid FollowPath.wz_max'
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'rolled_back' and '/rosout' in receipt['error'], receipt
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3
    assert stack.get(SMOOTHER, 'max_velocity')[2] == 0.3


def test_a_malformed_footprint_never_reaches_the_costmap(stack, live):
    """The real costmap takes any string and only complains on its own stderr, never on /rosout."""
    node, key, bad = '/global_costmap/global_costmap', 'footprint', '[[0.1, 0.1], [0.1'
    before = stack.get(node, key)
    assert stack.ports.params.set_atomically(node, {key: bad}) == (True, '')    # Nav2 says yes
    try:
        time.sleep(0.5)
        assert 'Error parsing footprint' in (stack.dir / 'planner.log').read_text()
    finally:
        stack.ports.params.set_atomically(node, {key: before})
    receipt = live.apply({'global_costmap.global_costmap.footprint': bad})
    assert receipt['status'] == 'refused' and 'footprint must be' in receipt['refused'], receipt
    assert stack.ports.sets == [] and stack.get(node, key) == before


def test_any_error_line_in_the_window_rolls_back_whatever_it_says(stack, live):
    stack.ports.log_after[CONTROLLER] = 'something broke'          # logged at ERROR, no keyword
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'rolled_back' and 'something broke' in receipt['error'], receipt
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3


def test_a_static_mppi_key_the_handler_does_not_own_rolls_back(stack, live):
    stack.ports.log_after[CONTROLLER] = ('controller_server',
                                         'Parameter FollowPath.wz_max not found')
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'rolled_back' and '/rosout' in receipt['error'], receipt
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3
    assert stack.get(SMOOTHER, 'max_velocity')[2] == 0.3


def test_the_shim_not_found_line_stays_exempt_on_the_live_path(stack, live):
    stack.ports.log_after[CONTROLLER] = (
        'controller_server', 'Parameter FollowPath.rotate_to_heading_angular_vel not found')
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'verified', receipt


def test_a_frozen_node_gets_one_try_and_what_went_before_is_restored(stack, live):
    pid = stack.ports.freeze = stack.child('smoother')
    try:
        receipt = live.apply(ask(wz_max=0.25))
    finally:
        os.kill(pid, signal.SIGCONT)
    assert receipt['status'] == 'rolled_back' and 'not retried' in receipt['error'], receipt
    assert stack.ports.sets == [SMOOTHER]            # one try; nothing went before it to restore
    assert stack.mux_holds() == [] and receipt['hold']['released']
    time.sleep(1.0)
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3


def test_refusals_with_the_real_authority_state(stack, live):
    stack.state['procedure'] = {'name': 'drive_to', 'run': 'r1'}
    stack.arbiter.tick(force=True)
    receipt = live.apply(ask(wz_max=0.25))
    assert receipt['status'] == 'refused' and 'procedure running' in receipt['refused']
    assert stack.ports.sets == [] and stack.mux_holds() == []
    stack.state['procedure'] = None
    stack.arbiter.tick(force=True)
    ack = stack.arbiter.handle({'op': 'claim', 'who': 'agent-x', 'reason': 'test'})
    assert ack['ok']
    time.sleep(1.3)
    assert 'claim held' in live.apply(ask(wz_max=0.25))['refused']
    stack.arbiter.handle({'op': 'release', 'who': 'agent-x', 'id': ack['id']})
    time.sleep(1.3)
    assert live.preview(ask(wz_max=0.25)).refusals == []
    dead = subprocess.Popen([sys.executable, '-c', 'pass'])
    dead.wait()
    stack.registry.update(owner={'pid': dead.pid, 'node': 'gone'})
    try:
        assert 'no live session owner' in live.apply(ask(wz_max=0.25))['refused']
    finally:
        stack.registry.update(owner={'pid': os.getpid(), 'node': 'nav2_live_test', 'host': 'test'})


def test_an_active_goal_refuses_and_now_allows_a_limits_change(stack, live):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = stack.node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                                         latched)
    entry = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    entry.goal_info.goal_id.uuid = list(range(16))
    status.publish(GoalStatusArray(status_list=[entry]))
    time.sleep(1.0)
    try:
        assert 'a goal is active' in live.apply(ask(wz_max=0.25))['refused']
        other = live.apply({'controller_server.failure_tolerance': 0.2}, now=True)
        assert 'a goal is active' in other['refused']
        assert stack.ports.sets == []
        assert live.apply(ask(wz_max=0.25), now=True)['status'] == 'verified'
        assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.25
    finally:
        entry.status = GoalStatus.STATUS_SUCCEEDED
        status.publish(GoalStatusArray(status_list=[entry]))
        time.sleep(0.5)
        stack.node.destroy_publisher(status)


def test_now_with_a_goal_and_no_mux_refuses_with_no_sets(stack, live):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = stack.node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                                         latched)
    entry = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    entry.goal_info.goal_id.uuid = list(range(16))
    status.publish(GoalStatusArray(status_list=[entry]))
    time.sleep(1.0)
    stack.ports.no_mux, stack.ports.expected = True, lambda: False
    try:
        receipt = live.apply(ask(wz_max=0.25), now=True)
        assert receipt['status'] == 'refused' and 'no mux to hold' in receipt['refused'], receipt
        assert stack.ports.sets == [] and stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3
    finally:
        stack.ports.expected = lambda: True
        entry.status = GoalStatus.STATUS_SUCCEEDED
        status.publish(GoalStatusArray(status_list=[entry]))
        time.sleep(0.5)
        stack.node.destroy_publisher(status)


def test_applying_under_a_human_hold_keeps_the_human_hold(stack, live):
    from std_msgs.msg import String
    request = stack.node.create_publisher(String, '/command_mux/request', 10)
    time.sleep(0.5)
    request.publish(String(data=json.dumps({'op': 'hold', 'cause': 'keys', 'by': 'tui'})))
    time.sleep(1.5)
    try:
        assert stack.mux_holds() == ['keys']
        receipt = live.apply(ask(wz_max=0.25))
        assert receipt['status'] == 'verified', receipt
        assert stack.mux_holds() == ['keys']
    finally:
        request.publish(String(data=json.dumps({'op': 'release', 'by': 'tui'})))
        time.sleep(1.0)
    assert stack.mux_holds() == []


def test_boat_trial_config_apply_nav2_end_to_end(stack, live):
    (stack.dir / 'nav2.yaml').write_text(
        'controller_server:\n  ros__parameters:\n    FollowPath:\n      wz_max: 0.25\n'
        '    failure_tolerance: 0.2\n')
    (stack.dir / 'events.jsonl').write_text('')
    # The CLI's mux hold must confirm within HOLD_CONFIRM_S (2 s); under full-suite load the
    # mux can answer later, so this subprocess alone waits longer. The live window is unchanged.
    patched = ('import runpy, mhseals_testing.harness.nav2_live as n; n.HOLD_CONFIRM_S = 15.0; '
               'runpy.run_module("mhseals_testing.harness.cli", run_name="__main__")')
    command = [sys.executable, '-c', patched, 'config', 'apply', 'nav2', '--json']
    options = {'env': stack.env, 'capture_output': True, 'text': True, 'timeout': 120}
    preview = subprocess.run(command, **options)
    assert preview.returncode == 2, preview.stdout + preview.stderr
    assert json.loads(preview.stdout)['outcome'] == 'preview'
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.3
    done = subprocess.run([*command, '--yes'], **options)
    assert done.returncode == 0, done.stdout + done.stderr
    doc = json.loads(done.stdout)
    assert doc['outcome'] == 'applied' and doc['live'] == 'until Navigation restarts'
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.25
    assert stack.get(CONTROLLER, 'failure_tolerance') == 0.2
    assert stack.get(SMOOTHER, 'max_velocity')[2] == 0.25
    lines = (stack.dir / 'events.jsonl').read_text().splitlines()
    events = [json.loads(line)['phase'] for line in lines]
    assert events == ['nav2_live_intent', 'nav2_live_outcome']
    from mhseals_testing.history import History
    changes = History(stack.dir / 'data' / 'config-history').changes('nav2')
    assert [(c['applied'], c['source']) for c in changes] == [('live-session', 'cli:apply-nav2')]
    again = subprocess.run([*command, '--yes'], **options)
    assert again.returncode == 1 and json.loads(again.stdout)['outcome'] == 'nothing to do'


def activations(stack):
    return (stack.dir / 'controller.log').read_text().count('Activating')


def test_a_non_live_key_is_set_and_navigation_resets_and_starts_again(stack, live):
    original = stack.get(CONTROLLER, 'FollowPath.batch_size')
    starts = activations(stack)
    receipt = live.apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'verified' and receipt['reset'], receipt
    assert stack.get(CONTROLLER, 'FollowPath.batch_size') == 500
    assert activations(stack) > starts                   # the controller really restarted
    assert stack.ports.params.is_active(timeout=10.0)
    assert stack.mux_holds() == [] and receipt['hold']['released']
    again = live.apply({FOLLOW + 'batch_size': original})
    assert again['status'] == 'verified', again
    assert stack.get(CONTROLLER, 'FollowPath.batch_size') == original


def test_nothing_is_set_on_the_active_controller_before_the_reset(stack, live):
    """MPPI keeps dynamic callbacks on dead locals (PathAngleCritic.mode, motion_model, ...)."""
    mode = FOLLOW + 'PathAngleCritic.mode'
    try:
        receipt = live.apply({mode: 1, FOLLOW + 'vx_max': 0.35})
        assert receipt['status'] == 'verified' and receipt['reset'], receipt
        kinds = [kind for kind, _ in stack.ports.order]
        assert kinds[0] == 'reset' and kinds[-1] == 'startup', stack.ports.order
        assert kinds.count('reset') == kinds.count('startup') == 1
        assert {n for k, n in stack.ports.order if k == 'set'} >= {CONTROLLER}
        assert stack.get(CONTROLLER, 'FollowPath.PathAngleCritic.mode') == 1
        assert stack.get(CONTROLLER, 'FollowPath.vx_max') == 0.35
    finally:    # the overlay cannot say mode 0, so put it back the safe way
        assert stack.ports.lifecycle('reset')
        stack.ports.params.set_atomically(CONTROLLER, {'FollowPath.PathAngleCritic.mode': 0,
                                                       'FollowPath.vx_max': 0.4})
        assert stack.ports.lifecycle('startup')
    assert stack.get(CONTROLLER, 'FollowPath.PathAngleCritic.mode') == 0


def test_a_rollback_after_a_reset_resets_before_it_restores(stack, live):
    stack.ports.startup_fails = 1
    receipt = live.apply({FOLLOW + 'PathAngleCritic.mode': 1})
    assert receipt['status'] == 'rolled_back', receipt
    kinds = [kind for kind, _ in stack.ports.order]
    assert kinds[0] == 'reset' and kinds.index('set') == 1
    assert kinds[-4:] == ['startup', 'reset', 'set', 'startup'], stack.ports.order
    assert stack.get(CONTROLLER, 'FollowPath.PathAngleCritic.mode') == 0


def test_a_failed_startup_restores_the_value_and_resets_again(stack, live):
    original = stack.get(CONTROLLER, 'FollowPath.batch_size')
    stack.ports.startup_fails = 1                                  # the first startup only
    starts = activations(stack)
    receipt = live.apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'rolled_back' and 'startup failed' in receipt['error'], receipt
    assert stack.ports.commands == ['reset', 'startup', 'reset', 'startup']
    assert stack.get(CONTROLLER, 'FollowPath.batch_size') == original
    assert stack.ports.params.is_active(timeout=10.0) and activations(stack) >= starts + 1
    assert stack.mux_holds() == []


def test_a_reset_is_refused_while_a_goal_runs_even_with_now(stack, live):
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = stack.node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                                         latched)
    entry = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    entry.goal_info.goal_id.uuid = list(range(16))
    status.publish(GoalStatusArray(status_list=[entry]))
    time.sleep(1.0)
    try:
        receipt = live.apply({FOLLOW + 'batch_size': 500}, now=True)
        assert receipt['status'] == 'refused' and 'a goal is active' in receipt['refused']
        assert stack.ports.sets == [] and stack.ports.commands == []
    finally:
        entry.status = GoalStatus.STATUS_SUCCEEDED
        status.publish(GoalStatusArray(status_list=[entry]))
        time.sleep(0.5)
        stack.node.destroy_publisher(status)


def test_the_tui_wiring_applies_only_as_the_session_owner(stack, live, tmp_path):
    from mhseals_testing import nav2_limits
    from mhseals_testing.harness.nav2_edit import Nav2File
    from mhseals_testing.history import History
    from mhseals_testing.profiles import load_profile
    from mhseals_testing.ui.app import build_nav2_live
    events = []
    ros = SimpleNamespace(node=stack.node, emit=lambda phase, **fields: events.append(phase))
    app = SimpleNamespace(armed=False, history=History(tmp_path / 'history'), warn_history=print)
    nav2_limits.path().parent.mkdir(parents=True, exist_ok=True)
    nav2_limits.path().write_text(
        'controller_server:\n  ros__parameters:\n    FollowPath:\n      wz_max: 0.25\n')
    tui = build_nav2_live(ros, stack.registry, load_profile('boat'), lambda: app)
    app.nav2_file = Nav2File(nav2_limits.path(), tui.applier)
    assert tui.me == os.getpid()
    assert app.nav2_file.apply() == 'Nav2 applied 1 key(s) live'
    assert stack.get(CONTROLLER, 'FollowPath.wz_max') == 0.25
    assert events == ['nav2_live_intent', 'nav2_live_outcome']
    change = app.history.head('nav2')
    assert (change['applied'], change['source']) == ('live-session', 'tui:nav2-live')
    stack.registry.update(owner={'pid': os.getppid(), 'node': 'someone-else'})
    try:
        other = tui.apply(ask(wz_max=0.25))
    finally:
        stack.registry.update(owner={'pid': os.getpid(), 'node': 'nav2_live_test', 'host': 'test'})
    assert other['status'] == 'refused' and 'does not own the session' in other['refused']
