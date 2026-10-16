import pytest
import yaml

from mhseals_hardware.configuration import load_defaults
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.arming import Arming, driver_commands


def test_driver_argv_for_effort_and_velocity_control(tmp_path):
    config = tmp_path / 'hardware.yaml'
    [(name, argv)] = driver_commands(load_profile('boat'), config)
    assert name == 'hardware'
    assert argv == ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node', '--ros-args',
                    '-p', f'config_file:={config}', '-p', 'use_sim_time:=false',
                    '-p', 'output_backend:=odroid', '-r', 'cmd_vel:=/cmd_vel']
    children = dict(driver_commands(load_profile('crane_full_stack').with_velocity_control(),
                                    config))
    assert children['hardware'][-1] == 'cmd_vel:=/cmd_vel_effort'
    assert 'output_backend:=topic' in children['hardware']
    assert 'use_sim_time:=true' in children['velocity']
    assert 'input_topic:=/cmd_vel' in children['velocity']
    assert 'odom_topic:=/odom/local' in children['velocity']


class Ros:
    def __init__(self, subscribers=()):
        self.names = list(subscribers)

    def subscribers(self, topic):
        return self.names


class Processes:
    def __init__(self, ros, die=False):
        self.ros, self.die, self.started, self.stopped = ros, die, [], []

    def start(self, name, argv):
        self.started.append(name)
        if name == 'hardware' and not self.die:
            self.ros.names.append('/thruster_pwm_node')

    def running(self, name):
        return name in self.started and name not in self.stopped and not self.die

    def stop(self, name):
        self.stopped.append(name)


def arming(profile='boat', die=False, subscribers=(), tmp_path=None):
    ros = Ros(subscribers)
    clock = [0.0]
    processes = Processes(ros, die)

    def sleep(seconds):
        clock[0] += seconds

    return Arming(load_profile(profile), ros, processes, load_defaults(),
                  tmp_path / 'hardware.yaml', clock=lambda: clock[0], sleep=sleep), processes


def test_arm_snapshots_config_and_waits_for_the_driver(tmp_path):
    armed, processes = arming('crane_full_stack', tmp_path=tmp_path)
    armed.arm()
    assert armed.armed and processes.started == ['hardware'] and armed.fault() == ''
    assert yaml.safe_load((tmp_path / 'hardware.yaml').read_text())['output_backend'] == 'topic'
    processes.stopped.append('hardware')  # the driver dies while armed
    assert 'hardware exited' in armed.fault()
    armed.disarm()
    assert not armed.armed and armed.fault() == ''


def test_arm_refuses_a_second_driver_and_cleans_up_a_dead_one(tmp_path):
    busy, processes = arming(subscribers=['/thruster_pwm_node'], tmp_path=tmp_path)
    with pytest.raises(RuntimeError, match='already drives'):
        busy.arm()
    assert processes.started == []
    dead, processes = arming(die=True, tmp_path=tmp_path)
    with pytest.raises(RuntimeError, match='exited during startup'):
        dead.arm()
    assert processes.stopped == ['hardware'] and not dead.armed


def test_plants_without_outputs_arm_without_children(tmp_path):
    direct, processes = arming('crane_direct', tmp_path=tmp_path)
    direct.arm()
    assert direct.armed and processes.started == []
    direct.disarm()
    assert processes.stopped == []
