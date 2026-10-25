"""Remote arming end to end on one machine playing two hosts: the TUI side (RosLink, RemoteDriver)
and the boat side (unit CLI, keeper, arm_link and its children) talk over DDS, and the "ssh" call
runs the unit command in a local shell with the boat's own runtime directory.

The boat side refuses to start beside any other thruster driver or mux on the computer, so these
skip where one runs (a simulator in the dev container); run them in a private pid namespace:
``docker exec -u root -e HOME=/tmp/h roboboat_dev unshare --pid --fork --mount-proc bash -c
'source <overlay>/install/setup.bash && python3 -m pytest test/test_arm_link_ros.py'``.
"""

from dataclasses import replace
import base64
import json
from pathlib import Path
import os
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt16MultiArray
from std_srvs.srv import Trigger

from mhseals_hardware.configuration import load_defaults, save_config
from mhseals_testing.devices import Host
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros import arm_link
from mhseals_testing.ros.arm_link import LINK_S, RemoteDriver, RosLink
from mhseals_testing.ros.arming import driver_commands

BOAT = Host('boat', capabilities=frozenset({'thrusters'}), address='127.0.0.1')
pytestmark = pytest.mark.skipif(bool(arm_link.rivals()),
                                reason='a thruster driver or mux runs here; use a pid namespace')


def wait_for(condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def boat(tmp_path):
    """The boat's environment and its ssh stand-in."""
    environment = dict(os.environ, XDG_RUNTIME_DIR=str(tmp_path / 'boat-run'),
                       MHSEALS_DATA=str(tmp_path / 'boat-data'))
    os.makedirs(environment['XDG_RUNTIME_DIR'], mode=0o700)
    state = {'down': False}

    def call(host, command, jump, timeout):
        if state['down']:
            return [], 'ssh: timeout'
        done = subprocess.run(['bash', '-c', command], capture_output=True, text=True,
                              timeout=timeout, env=environment)
        lines = [json.loads(line) for line in done.stdout.splitlines() if line.startswith('{')]
        return lines, (None if lines else f'exit {done.returncode}: {done.stderr[-300:]}')

    def status():
        return call(None, 'python3 -m mhseals_testing.harness.unit status mhseals-arming', None, 10)[0][0]

    yield state, call, status, environment
    state['down'] = False
    call(None, 'python3 -m mhseals_testing.harness.unit stop mhseals-arming', None, 30)


@pytest.fixture
def tui():
    rclpy.init()
    node = rclpy.create_node('tui_side')
    estops = []
    server = rclpy.create_node('thruster_pwm_node')    # stands in for the driver's e-stop
    server.create_service(Trigger, '~/estop',
                          lambda request, response: estops.append(time.monotonic()) or response)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node, executor, server, estops
    executor.shutdown()
    node.destroy_node()
    server.destroy_node()
    rclpy.try_shutdown()


def tree(status):
    record = status()
    return [record['pid'], *arm_link.descendants(record['pid'])] if record['state'] == 'running' else []


def arm(node, call, children, config=None):
    config = config or config_text()
    driver = RemoteDriver(BOAT, RosLink(node), by={'host': 'tui'}, call=call)
    driver.start(children, config)
    assert wait_for(driver.confirmed, 30), driver.error or driver.last_line('hardware')
    return driver


TOPIC = f'/arm_link_test_{os.getpid()}/pwm'      # never a simulator's on the same domain
# The unit's --driver gate starts only arming's own children: the real driver and mux, on a topic.
FAKE = driver_commands(replace(load_profile('boat'), outputs='topic', velocity_controller=False),
                       arm_link.CONFIG, {})

def config_text(tmp_path_factory=None):
    import tempfile
    path = Path(tempfile.mkdtemp()) / 'hardware.yaml'
    save_config(dict(load_defaults(), output_backend='topic', pwm_topic=TOPIC), path)
    return path.read_text()


def test_link_loss_stops_every_child_within_the_bound(boat, tui):
    _, call, status, _ = boat
    node, *_ = tui
    driver = arm(node, call, FAKE)
    pids = tree(status)
    assert len(pids) >= 3       # arm_link and the driver and mux nodes themselves
    with driver.link.lock:      # the radio drops: no beat and no stop message reach the boat
        driver.link.armed = False
        driver.link.stop_until = 0.0
    dropped = time.monotonic()
    assert wait_for(lambda: not any(arm_link._alive(pid) for pid in pids), LINK_S + 2.0)
    assert time.monotonic() - dropped < LINK_S + 1.0
    assert wait_for(lambda: status()['state'] == 'exited', 5) and status()['code'] == 3
    assert not driver.running('hardware')         # the TUI sees it from the boat's last state
    assert 'operator link lost' in driver.last_line('hardware')


def test_disarm_and_estop_act_over_dds_while_ssh_is_down(boat, tui):
    state, call, status, _ = boat
    node, _, _, estops = tui
    driver = arm(node, call, FAKE)
    pids = tree(status)
    state['down'] = True
    started = time.monotonic()
    driver.stop()
    assert wait_for(lambda: not any(arm_link._alive(pid) for pid in pids), 1.0)
    assert wait_for(lambda: estops, 2.0) and estops[0] - started < 1.0
    state['down'] = False
    assert wait_for(lambda: status()['state'] == 'exited', 5) and status()['code'] == 0


def test_a_second_tuis_beats_never_keep_it_and_no_beat_starts_nothing(boat, tui, tmp_path):
    _, call, status, environment = boat
    node, *_ = tui
    other = RosLink(node)
    other.set('other-token', True)
    argv = [*arm_link.MODULE, 'run', '--token', 'mine', '--unit', arm_link.UNIT,
            '--first-beat-s', '2', '--config-b64', base64.b64encode(config_text().encode()).decode(), '--', *arm_link.join_children(FAKE)]
    from mhseals_testing.harness.remote import unit_command
    [started], error = call(None, unit_command('start', arm_link.UNIT, argv, lease_s=3.0, driver=True),
                            None, 20)
    assert error is None and started['state'] == 'running'
    assert wait_for(lambda: status()['state'] == 'exited', 10)
    log = open(status()['log']).read()
    assert 'operator heard' not in log and 'no operator beat' in log
    other.set('other-token', False)


def test_the_real_driver_and_mux_reach_neutral_and_stop_on_link_loss(boat, tui, tmp_path):
    _, call, status, _ = boat
    node, executor, *_ = tui
    profile = replace(load_profile('boat'), outputs='topic', velocity_controller=False)
    config = tmp_path / 'hardware.yaml'
    save_config(dict(load_defaults(), output_backend='topic', pwm_topic=TOPIC), config)
    pulses = []
    node.create_subscription(UInt16MultiArray, TOPIC,
                             lambda message: pulses.append(list(message.data)), 10)
    mux_state = []
    node.create_subscription(String, '/command_mux/state', mux_state.append,
                             rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    driver = arm(node, call, driver_commands(profile, arm_link.CONFIG, {}), config.read_text())
    names = lambda: {name for name, _ in node.get_node_names_and_namespaces()}
    assert wait_for(lambda: {'thruster_pwm_node', 'command_mux'} <= names() and mux_state, 20)
    pids = tree(status)
    with driver.link.lock:
        driver.link.armed = False
        driver.link.stop_until = 0.0
    dropped = time.monotonic()
    assert wait_for(lambda: not any(arm_link._alive(pid) for pid in pids), LINK_S + 8.0)
    stopped = time.monotonic() - dropped
    from mhseals_hardware.outputs import NEUTRAL_PWM as neutral
    assert pulses and pulses[-1] == [neutral] * 4
    assert stopped < LINK_S + 8.0
    print(f'real driver and mux stopped {stopped:.2f} s after the last beat')


def test_a_config_whose_pins_differ_from_the_boats_never_starts_a_unit(boat, tui):
    _, call, status, _ = boat
    node, *_ = tui
    driver = RemoteDriver(BOAT, RosLink(node), call=call)
    driver.start(FAKE, 'channel_map: [4, 3, 2, 1]\n')   # the boat (this test) keeps the defaults
    assert wait_for(lambda: driver.error, 20)
    assert 'channel_map' in driver.error and not driver.running('hardware')
    assert status()['state'] == 'off'


@pytest.mark.parametrize('signum', ['SIGKILL', 'SIGTERM'])
def test_a_killed_readback_never_leaves_the_real_driver_running(boat, tui, tmp_path, signum):
    import signal
    _, _, _, environment = boat
    node, *_ = tui
    topic = f'/arm_link_test_{os.getpid()}/pwm'
    config = tmp_path / 'hardware.yaml'
    save_config(dict(load_defaults(), output_backend='topic', pwm_topic=topic), config)
    argv = ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node', '--ros-args',
            '-p', f'config_file:={arm_link.CONFIG}', '-p', 'output_backend:=topic']
    readback = subprocess.Popen(
        [*arm_link.MODULE, 'readback', '--config-b64', base64.b64encode(config.read_bytes()).decode(),
         '--', *argv], env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        assert wait_for(lambda: arm_link.descendants(readback.pid), 15)
        [driver] = arm_link.descendants(readback.pid)       # the node itself, no ros2 run between
        time.sleep(3)                                       # well into its start: ROS up, outputs open
        readback.send_signal(getattr(signal, signum))
        assert wait_for(lambda: not arm_link._alive(driver), 5)
    finally:
        readback.kill()
