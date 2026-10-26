import pytest
import yaml

from mhseals_hardware.configuration import load_defaults
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.arming import Arming, driver_commands


def test_driver_argv_for_effort_and_velocity_control(tmp_path):
    config = tmp_path / 'hardware.yaml'
    [(name, argv), mux] = driver_commands(load_profile('boat'), config)
    assert name == 'hardware'
    assert argv == ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node', '--ros-args',
                    '-p', f'config_file:={config}', '-p', 'use_sim_time:=false',
                    '-p', 'output_backend:=odroid', '-r', 'cmd_vel:=/cmd_vel/applied']
    assert mux[0] == 'mux' and mux[1][:5] == ['ros2', 'run', 'mhseals_hardware',
                                              'command_mux', '--ros-args']
    children = dict(driver_commands(load_profile('crane_full_stack').with_velocity_control(),
                                    config))
    assert children['hardware'][-1] == 'cmd_vel:=/cmd_vel_effort'
    assert 'output_backend:=topic' in children['hardware']
    assert 'use_sim_time:=true' in children['velocity']
    assert 'input_topic:=/cmd_vel/applied' in children['velocity']
    assert 'odom_topic:=/odom/local' in children['velocity']


class Ros:
    def __init__(self, subscribers=(), nodes=(), parameters=True):
        self.names = list(subscribers)
        self.node_names, self.parameters = list(nodes), parameters

    def subscribers(self, topic):
        if isinstance(self.names, dict):  # per-topic subscribers
            return self.names.get(topic, [])
        return self.names

    def publishers(self, topic):
        return []

    def nodes(self):
        return self.node_names

    def parameter_service(self, name):
        return self.parameters


class Processes:
    def __init__(self, ros, die=False):
        self.ros, self.die, self.started, self.stopped = ros, die, [], []

    def start(self, name, argv):
        self.started.append(name)
        if name == 'hardware' and not self.die:
            self.ros.names.append('/thruster_pwm_node')

    def last_line(self, name):
        return 'Traceback: no pwm chip'

    def running(self, name):
        return name in self.started and name not in self.stopped and not self.die

    def stop(self, name):
        self.stopped.append(name)
        if name == 'hardware' and '/thruster_pwm_node' in self.ros.names:
            self.ros.names.remove('/thruster_pwm_node')   # a stopped driver leaves the graph


def arming(profile='boat', die=False, subscribers=(), tmp_path=None, nodes=()):
    ros = Ros(subscribers, nodes)
    clock = [0.0]
    processes = Processes(ros, die)

    def sleep(seconds):
        clock[0] += seconds

    return Arming(load_profile(profile), ros, processes, load_defaults(),
                  tmp_path / 'hardware.yaml', clock=lambda: clock[0], sleep=sleep), processes


def test_arm_snapshots_config_and_waits_for_the_driver(tmp_path):
    armed, processes = arming('crane_full_stack', tmp_path=tmp_path)
    armed.arm()
    assert armed.armed and processes.started == ['hardware', 'mux'] and armed.fault() == ''
    assert yaml.safe_load((tmp_path / 'hardware.yaml').read_text())['output_backend'] == 'topic'
    processes.stopped.append('hardware')  # the driver dies while armed
    assert 'hardware exited' in armed.fault()
    armed.disarm()
    assert not armed.armed and armed.fault() == ''


def test_restart_driver_runs_only_the_driver_for_the_readback_then_stops_it(tmp_path):
    armed, processes = arming('crane_full_stack', tmp_path=tmp_path)
    during = []
    assert armed.restart_driver(dict(load_defaults(), mosfet_line=27),
                                lambda: during.append(processes.running('hardware')) or 'seen')
    assert during == [True] and processes.started == ['hardware'] == processes.stopped
    assert not processes.running('hardware') and armed.rival() == ''
    assert yaml.safe_load((tmp_path / 'hardware.yaml').read_text())['mosfet_line'] == 27
    armed.arm()   # the next arm is not blocked by a leftover driver
    assert armed.armed and processes.started == ['hardware', 'hardware', 'mux']


def test_restart_driver_refuses_beside_another_driver(tmp_path):
    armed, processes = arming(nodes=['/sim/thruster_pwm_node'], tmp_path=tmp_path)
    with pytest.raises(RuntimeError, match='another thruster_pwm_node'):
        armed.restart_driver(load_defaults(), lambda: pytest.fail('no readback'))
    assert processes.started == []


def test_arm_refuses_a_second_driver_and_cleans_up_a_dead_one(tmp_path):
    busy, processes = arming(subscribers=['/thruster_pwm_node'], tmp_path=tmp_path)
    with pytest.raises(RuntimeError, match='already drives'):
        busy.arm()
    assert processes.started == []
    dead, processes = arming(die=True, tmp_path=tmp_path)
    with pytest.raises(RuntimeError, match='exited during startup: Traceback: no pwm chip'):
        dead.arm()
    assert processes.stopped == ['mux', 'hardware'] and not dead.armed


def test_plants_without_outputs_arm_without_children(tmp_path):
    direct, processes = arming('crane_direct', tmp_path=tmp_path)
    direct.arm()
    assert direct.armed and processes.started == []
    direct.disarm()
    assert processes.stopped == []


def test_begin_polls_until_the_driver_listens_and_times_out_after_ten_seconds(tmp_path):
    slow, processes = arming(tmp_path=tmp_path)
    slow.ros.names.clear()
    processes.start = lambda name, argv: processes.started.append(name)
    slow.begin()
    slow.ros.node_names.append('/thruster_pwm_node')  # discovered, yet never subscribes
    assert slow.poll() == 'starting' and not slow.armed
    slow.sleep(10.5)
    with pytest.raises(RuntimeError, match='did not subscribe'):
        slow.poll()
    assert processes.stopped == ['mux', 'hardware'] and slow.pending is None


def test_cancel_stops_a_start_in_progress(tmp_path):
    slow, processes = arming(tmp_path=tmp_path)
    slow.begin()
    slow.cancel()
    assert slow.pending is None and not slow.armed and processes.stopped == ['mux', 'hardware']


def test_an_invalid_velocity_model_fails_at_once_without_starting_anything(tmp_path):
    bad, processes = arming('crane_full_stack', tmp_path=tmp_path)
    bad.profile = bad.profile.with_velocity_control()
    bad.hardware_config = {}  # no characterization.velocity_gain
    with pytest.raises(RuntimeError, match='velocity model invalid: .*velocity_gain'):
        bad.begin()
    assert processes.started == [] and bad.pending is None
    assert not (tmp_path / 'hardware.yaml').exists()
    bad.hardware_config = {'characterization': {'velocity_gain': {'surge': {'surge': 1.0}}}}
    with pytest.raises(RuntimeError, match='velocity model invalid'):  # partial row
        bad.begin()
    assert processes.started == []


def test_velocity_arming_waits_for_the_controller_subscription(tmp_path):
    waiting, processes = arming('crane_full_stack', tmp_path=tmp_path)
    waiting.profile = waiting.profile.with_velocity_control()
    waiting.hardware_config = dict(load_defaults(), characterization={'velocity_gain': {
        row: {column: float(row == column) for column in ('surge', 'sway', 'yaw')}
        for row in ('surge', 'sway', 'yaw')},
        'motion_limits': {'velocity': {'surge': 1.0, 'sway': 1.0, 'yaw': 1.0}}})
    waiting.begin()
    assert waiting.listening() == ['calibrated_velocity_controller']
    assert waiting.poll() == 'starting'
    waiting.ros.names.append('/calibrated_velocity_controller')
    assert waiting.poll() == 'armed'


def startup_failure(tmp_path, **ros):
    slow, processes = arming(tmp_path=tmp_path)
    slow.ros.names.clear()
    slow.ros.parameters = ros.get('parameters', True)
    processes.start = lambda name, argv: processes.started.append(name)
    slow.begin()
    slow.ros.node_names = ros.get('nodes', [])  # discovered only after our driver started
    slow.sleep(10.5)
    with pytest.raises(RuntimeError) as error:
        slow.poll()
    return str(error.value)


def test_a_timeout_says_whether_the_driver_is_undiscovered_or_its_parameters_are_down(tmp_path):
    assert 'not discovered; check ROS_DOMAIN_ID' in startup_failure(tmp_path)
    assert 'parameter service is down' in startup_failure(
        tmp_path, nodes=['/thruster_pwm_node'], parameters=False)
    assert 'did not subscribe' in startup_failure(tmp_path, nodes=['/thruster_pwm_node'])


def test_a_controller_timeout_names_the_topic_it_subscribes_to(tmp_path):
    slow, processes = arming('crane_full_stack', tmp_path=tmp_path)
    slow.profile = slow.profile.with_velocity_control()
    slow.hardware_config = dict(load_defaults(), characterization={'velocity_gain': {
        row: {column: float(row == column) for column in ('surge', 'sway', 'yaw')}
        for row in ('surge', 'sway', 'yaw')},
        'motion_limits': {'velocity': {'surge': 1.0, 'sway': 1.0, 'yaw': 1.0}}})
    slow.begin()
    slow.ros.names = {slow.driver_topic: ['/thruster_pwm_node']}
    slow.ros.node_names = ['/thruster_pwm_node', '/calibrated_velocity_controller']
    slow.sleep(10.5)
    with pytest.raises(RuntimeError, match=f'did not subscribe to {slow.profile.command_topic}'):
        slow.poll()


def test_a_boat_velocity_controller_listens_on_the_mux_output(tmp_path):
    profile = load_profile('boat').with_velocity_control()
    children = dict(driver_commands(profile, tmp_path / 'hardware.yaml'))
    assert 'input_topic:=/cmd_vel/applied' in children['velocity']
    assert 'cmd_vel:=/cmd_vel_effort' in children['hardware']
    assert list(children) == ['hardware', 'velocity', 'mux']


def test_arm_refuses_a_driver_node_listening_on_another_topic(tmp_path):
    """rv1-47: the sim rig's own thruster_pwm_node subscribes elsewhere; arming must still refuse."""
    busy, processes = arming(nodes=['/thruster_pwm_node'], tmp_path=tmp_path)
    assert 'owns the outputs' in busy.rival()
    with pytest.raises(RuntimeError, match='owns the outputs'):
        busy.begin()
    assert processes.started == []


def test_arm_refuses_a_foreign_publisher_on_applied(tmp_path):
    from mhseals_testing.ros.arming import APPLIED_TOPIC
    armed, processes = arming(tmp_path=tmp_path)
    armed.ros.publishers = lambda topic: ['/command_mux'] if topic == APPLIED_TOPIC else []
    with pytest.raises(RuntimeError, match='already has a publisher'):
        armed.arm()
    assert processes.started == []


def test_profile_manual_topic_follows_the_mux():
    assert load_profile('boat').manual_topic == '/cmd_vel/manual'
    assert load_profile('crane_direct').manual_topic == '/crane/cmd_vel_stamped'


def test_controller_restart_rebuilds_snapshot_and_topology_from_the_profile(tmp_path):
    from dataclasses import replace
    from mhseals_testing.ros.arming import controller_topics

    profile = replace(load_profile('crane_full_stack').with_velocity_control(), components={})
    desired = dict(load_defaults(), characterization={'velocity_gain': {
        row: {column: float(row == column) for column in ('surge', 'sway', 'yaw')}
        for row in ('surge', 'sway', 'yaw')},
        'motion_limits': {'velocity': {'surge': 1., 'sway': 1., 'yaw': 1.}}})
    desired['controller'].update(kp=[2., 3., 4.], ki=[.1, .2, .3])
    topics = controller_topics(profile)
    ros = Ros()
    processes = Processes(ros)
    invocations = []
    def start(name, argv):
        invocations.append((name, argv))
        processes.started.append(name)
        if name == 'hardware':
            ros.names.append('/thruster_pwm_node')
        if name == 'velocity':
            ros.names.append('/calibrated_velocity_controller')
    processes.start = start
    armed = Arming(profile, ros, processes, desired, tmp_path / 'snapshot.yaml')
    for attempt in range(2):
        invocations.clear()
        armed.begin()
        assert armed.poll() == 'armed'
        assert armed.expected()['calibrated_velocity_controller'] == topics['input_topic']
        snapshot = yaml.safe_load(armed.config_path.read_text())
        assert snapshot['controller']['kp'] == desired['controller']['kp']
        assert snapshot['characterization'] == desired['characterization']
        argv = dict(invocations)['velocity']
        for key, value in topics.items():
            assert f'{key}:={value}' in argv
        assert not any(value.startswith(('kp:=', 'ki:=')) for value in argv)
        assert f"cmd_vel:={topics['output_topic']}" in dict(invocations)['hardware']
        armed.disarm()
        ros.names.clear()
        processes.stopped.clear()
        desired['controller']['kp'] = [5., 5., 5.]
        armed.hardware_config = desired
    assert yaml.safe_load(armed.config_path.read_text())['controller']['kp'] == [5., 5., 5.]


def test_session_hardware_gives_only_a_sim_session_its_own_mux_hold_file(tmp_path):
    from types import SimpleNamespace
    from mhseals_testing.ros.arming import session_hardware
    path = tmp_path / 'snapshots' / 'hardware.yaml'
    config = {'command_mux': {'state_file': '/run/boat.json', 'rate': 1}}
    sim = session_hardware(load_profile('crane_full_stack'), config, path)
    assert sim['output_backend'] == 'topic' and sim['command_mux']['rate'] == 1
    assert sim['command_mux']['state_file'] == str(tmp_path / 'command_mux_holds.json')
    assert config['command_mux']['state_file'] == '/run/boat.json'
    boat = session_hardware(load_profile('boat'), config, path)
    assert boat['command_mux'] == config['command_mux'] and boat['output_backend'] == 'odroid'
    remote = session_hardware(load_profile('crane_full_stack'), config, path, remote=object())
    assert remote['command_mux'] == config['command_mux']
    assert 'command_mux' not in session_hardware(SimpleNamespace(
        use_sim_time=False, outputs='topic'), {}, path)


def test_all_three_arm_time_snapshots_use_the_helper():
    import inspect
    from mhseals_testing.ros import arming
    from mhseals_testing.ui import app
    for source in (inspect.getsource(arming.Arming.begin), inspect.getsource(arming.Arming.restart_driver),
                   inspect.getsource(app.App.apply_hardware)):
        assert 'session_hardware(' in source and 'output_backend=' not in source
