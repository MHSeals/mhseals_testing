"""Remote arming without ROS: the boat side's decision, the TUI's RemoteDriver, Arming over it."""

import base64
import json
import os
import shlex
import subprocess
import sys
from types import SimpleNamespace

import pytest

from mhseals_hardware.configuration import load_defaults
from mhseals_testing.devices import Host, Registry
from mhseals_testing.harness import unit
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros import arm_link
from mhseals_testing.ros.arm_link import LINK_S, STATE_STALE_S, RemoteDriver, Watch
from mhseals_testing.ros.arming import Arming

BOAT = Host('blastoise-odroid', ('boat_computer',), frozenset({'thrusters', 'mavros'}),
            '192.168.0.152', user='roboboat', container='roboboat_dev')


# --- boat side: when the children must stop ---------------------------------------------------

def beat(token='t1', armed=True):
    return json.dumps({'token': token, 'armed': armed})


def test_link_loss_stops_the_children_after_link_s_and_not_before():
    watch = Watch('t1', now=0.0)
    assert watch.reason(5.0) == '' and not watch.heard()     # waiting for the first beat
    watch.message(beat(), 5.0)
    for now in (5.5, 5.0 + LINK_S):
        assert watch.reason(now) == ''
    assert 'operator link lost' in watch.reason(5.0 + LINK_S + 0.05)


def test_the_bound_is_tighter_than_the_local_keeper_and_matches_the_mux():
    # A local driver stops when the TUI process dies (keeper poll 0.5 s), never on link loss;
    # manual commands go neutral after the mux's cmd_timeout either way.
    defaults = load_defaults()
    assert defaults['command_mux']['cmd_timeout'] <= LINK_S <= 1.0
    assert arm_link.BEAT_S * 5 <= LINK_S < STATE_STALE_S


def test_only_this_armings_token_keeps_it_alive_or_stops_it():
    watch = Watch('t1', now=0.0)
    watch.message(beat('other'), 0.1)
    watch.message('not json', 0.1)
    assert not watch.heard()
    watch.message(beat(), 0.2)
    watch.message(beat('other', armed=False), 0.3)
    assert watch.reason(0.3) == ''
    watch.message(beat(armed=False), 0.4)
    assert watch.reason(0.4) == 'disarmed by the operator'
    watch.message(beat(), 0.5)                    # a late beat never re-arms
    assert watch.reason(0.5) == 'disarmed by the operator'


def test_no_first_beat_means_no_children_and_an_exit():
    watch = Watch('t1', first_beat_s=15.0, now=0.0)
    assert watch.reason(14.9) == ''
    assert 'no operator beat' in watch.reason(15.1)


def encoded(text):
    return base64.b64encode(text.encode()).decode()


def test_the_boat_refuses_a_config_whose_physical_setup_differs_from_its_own(tmp_path, monkeypatch):
    monkeypatch.setattr(arm_link, 'rivals', lambda: [])
    own = tmp_path / 'hardware.yaml'
    own.write_text('channel_map: [1, 3, 2, 4]\nmanual_repeat_timeout: 0.5\n')
    refused = arm_link.check(encoded(''), own)['error']             # the laptop's defaults
    assert 'differs from this computer\'s in channel_map' in refused
    assert arm_link.check(encoded('channel_map: [1, 3, 2, 4]\n'), own) == {'ok': True}  # live keys may differ
    assert 'reversed' in arm_link.check(encoded('channel_map: [1, 3, 2, 4]\nreversed: [true, false, false, false]\n'),
                                        own)['error']
    assert 'unreadable' in arm_link.check(encoded('nonsense_key: 1\n'), own)['error']
    monkeypatch.setattr(arm_link, 'rivals', lambda: [42])
    assert 'already runs here (pids [42])' in arm_link.check(encoded(''), own)['error']


def test_the_rc_kill_switch_and_mode_channels_must_match_the_boats(tmp_path, monkeypatch):
    monkeypatch.setattr(arm_link, 'rivals', lambda: [])
    own = tmp_path / 'hardware.yaml'
    own.write_text('command_mux:\n  stop_ch: 6\n')
    assert 'command_mux.stop_ch' in arm_link.check(encoded('command_mux:\n  stop_ch: 7\n'), own)['error']
    assert arm_link.check(encoded('command_mux:\n  stop_ch: 6\n'), own) == {'ok': True}
    defaults = load_defaults()
    moved = dict(defaults, command_mux=dict(defaults['command_mux'], mode_ch=defaults['command_mux']['mode_ch'] + 1),
                 reverse_thrust_ratio=0.9 if defaults['reverse_thrust_ratio'] != 0.9 else 0.8)
    assert arm_link.mismatch(moved, defaults) == ['reverse_thrust_ratio', 'command_mux.mode_ch']


def test_children_round_trip_and_the_config_placeholder():
    children = [('hardware', ['ros2', 'run', 'x', '-p', 'config_file:=@CONFIG@']),
                ('mux', ['ros2', 'run', 'y'])]
    assert arm_link.split_children(arm_link.join_children(children)) == children
    assert arm_link.with_config(children[0][1], '/d/h.yaml')[-1] == 'config_file:=/d/h.yaml'
    with pytest.raises(ValueError):
        arm_link.split_children(['hardware'])


def fake_proc(tmp_path, processes):
    for pid, (ppid, argv) in processes.items():
        entry = tmp_path / str(pid)
        entry.mkdir()
        (entry / 'cmdline').write_bytes(b'\0'.join(word.encode() for word in argv) + b'\0')
        (entry / 'stat').write_text(f'{pid} (x y) S {ppid} 1 1')
    return tmp_path


def test_rivals_are_drivers_by_command_line_not_their_wrappers(tmp_path):
    proc = fake_proc(tmp_path, {
        10: (1, ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'run', 'mhseals_hardware', 'command_mux']),
        11: (10, ['/usr/bin/python3', '/ws/lib/mhseals_hardware/thruster_pwm_node', '--ros-args']),
        12: (1, ['python3', '-m', 'mhseals_testing.harness.keeper', '--', 'thruster_pwm_node']),
        13: (1, ['python3', '-m', 'mhseals_testing.ros.arm_link', 'run', '--', 'hardware',
                 'thruster_pwm_node']),
        14: (1, ['bash', '-c', 'python3 -m mhseals_testing.harness.unit start -- x thruster_pwm_node -r a']),
        15: (1, ['sleep', '60'])})
    assert arm_link.rivals(proc) == [10, 11]
    assert sorted(arm_link.descendants(1, proc)) == [10, 11, 12, 13, 14, 15]
    assert arm_link.descendants(10, proc) == [11]


def test_stop_reaches_the_node_under_a_wrapper_that_does_not_forward_sigint():
    # ros2 run waits on its node through the SIGINT it gets; only the node stops on it.
    node = 'import time; time.sleep(60)'
    wrapper = ('import signal, subprocess, sys\n'
               'signal.signal(signal.SIGINT, lambda *_: None)\n'
               f'sys.exit(subprocess.Popen([sys.executable, "-c", {node!r}]).wait())')
    child = arm_link.Child('hardware', [sys.executable, '-c', wrapper])
    try:
        import time
        deadline = time.time() + 5
        while not arm_link.descendants(child.process.pid) and time.time() < deadline:
            time.sleep(0.05)
        [grandchild] = arm_link.descendants(child.process.pid)
        started = time.monotonic()
        arm_link.stop_children([child])
        assert not child.running() and not arm_link._alive(grandchild)
        assert time.monotonic() - started < 2.0          # SIGINT did it, not the TERM/KILL steps
    finally:
        child.process.kill()


# --- unit: the driver runs only as arming's arm_link unit -------------------------------------

DRIVER_ARGV = ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node']


def test_a_driver_unit_needs_the_driver_flag_and_arm_link():
    wrapped = ['python3', '-m', 'mhseals_testing.ros.arm_link', 'run', '--token', 't', '--',
               'hardware', *DRIVER_ARGV]
    unit.refuse(wrapped, False, allow_driver=True)
    for argv, allowed in ((wrapped, False), (DRIVER_ARGV, True), (['python3', '-m', 'x', 'run',
                                                                  *DRIVER_ARGV], True)):
        with pytest.raises(unit.UnitError, match='never runs as a unit'):
            unit.refuse(argv, True, allow_driver=allowed)


def test_the_driver_flag_starts_only_armings_own_children():
    head = ['python3', '-m', 'mhseals_testing.ros.arm_link', 'run', '--token', 't', '--']
    nav = ['nav', 'ros2', 'launch', 'mhseals_nav', 'navigation.launch.py', '--next', 'hardware', *DRIVER_ARGV]
    with pytest.raises(unit.UnitError, match='only ros2 run mhseals_hardware'):
        unit.refuse(head + nav, False, allow_driver=True)
    with pytest.raises(unit.UnitError, match='only ros2 run mhseals_hardware'):
        unit.refuse(head + ['x', 'sleep', '60'], False, allow_driver=True)
    for name in ('thruster_pwm_node', 'command_mux', 'calibrated_velocity_controller'):
        unit.refuse(head + ['c', 'ros2', 'run', 'mhseals_hardware', name], False, allow_driver=True)


def test_the_driver_unit_lease_is_capped_at_the_short_lease(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    argv = ['python3', '-m', 'mhseals_testing.ros.arm_link', 'run', '--', 'hardware', *DRIVER_ARGV]
    with pytest.raises(unit.UnitError, match='at most 3 s'):
        unit.start('mhseals-arming', argv, lease_s=120.0, allow_driver=True)


def test_ros2_run_children_are_started_as_the_node_itself(tmp_path, monkeypatch):
    import ament_index_python.packages as index
    executable = tmp_path / 'lib' / 'mhseals_hardware' / 'thruster_pwm_node'
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/bin/sh\n')
    executable.chmod(0o755)
    monkeypatch.setattr(index, 'get_package_prefix', lambda package: str(tmp_path), raising=False)
    assert arm_link.direct([*DRIVER_ARGV, '--ros-args']) == [str(executable), '--ros-args']
    assert arm_link.direct(['sleep', '1']) == ['sleep', '1']
    executable.unlink()          # never back to ros2 run, whose wrapper swallows SIGINT
    with pytest.raises(ValueError, match='not an installed executable'):
        arm_link.direct(DRIVER_ARGV)

    def missing(package):
        raise LookupError(package)
    monkeypatch.setattr(index, 'get_package_prefix', missing, raising=False)
    with pytest.raises(ValueError, match='not installed here'):
        arm_link.direct(DRIVER_ARGV)


def test_a_killed_parent_takes_its_child_node_down():
    # SIGKILL skips every finally; PDEATHSIG still reaches the node, as it is a direct child.
    node = 'import time; time.sleep(60)'
    parent = subprocess.Popen([sys.executable, '-c',
                               'import sys, time; from mhseals_testing.ros.arm_link import Child\n'
                               f'c = Child("hardware", [sys.executable, "-c", {node!r}])\n'
                               'print(c.process.pid, flush=True); time.sleep(60)'],
                              stdout=subprocess.PIPE, text=True)
    child = int(parent.stdout.readline())
    parent.kill()
    parent.wait()
    import time
    deadline = time.time() + 3
    while arm_link._alive(child) and time.time() < deadline:
        time.sleep(0.05)
    assert not arm_link._alive(child)


def test_the_unit_command_carries_the_short_lease_and_the_driver_flag():
    from mhseals_testing.harness.remote import unit_command
    words = shlex.split(unit_command('start', 'mhseals-arming', ['a'], {}, lease_s=3.0, driver=True))
    assert words[words.index('--lease-s') + 1] == '3.0' and '--driver' in words
    assert '--driver' not in shlex.split(unit_command('start', 'mhseals-camera', ['a']))


# --- TUI side ---------------------------------------------------------------------------------

class Link:
    def __init__(self):
        self.calls, self.latest = [], (None, None)

    def set(self, token, armed):
        self.calls.append(('set', token, armed))

    def estop(self):
        self.calls.append(('estop',))

    def state(self):
        return self.latest

    def report(self, token, at, phase='running', children=None, **extra):
        children = {'hardware': True, 'mux': True} if children is None else children
        self.latest = (dict(token=token, phase=phase, children=children, **extra), at)


class Ssh:
    """The ssh call: records commands; ``down`` answers like a dropped link."""

    def __init__(self, answer=None, down=False):
        self.commands, self.answer, self.down, self.link = [], answer, down, None

    def __call__(self, host, command, jump, timeout):
        if self.link is not None:
            self.link.calls.append(('ssh', command.split()[3]))
        self.commands.append(command)
        if self.down:
            return [], 'ssh: timeout'
        return [self.answer or {'state': 'running', 'adopted': False}], None


def remote(ssh=None, clock=None):
    link, ssh = Link(), ssh or Ssh()
    ssh.link = link
    now = clock or [100.0]
    return RemoteDriver(BOAT, link, by={'host': 'tranquility'}, call=ssh,
                        clock=lambda: now[0]), link, ssh, now


def test_start_beats_before_the_ssh_start_and_ships_the_config():
    driver, link, ssh, _ = remote()
    driver.start([('hardware', ['ros2', 'run', 'x'])], 'pwm_channels: [0, 1, 2, 3]\n')
    driver.worker.shutdown(wait=True)
    assert link.calls[0] == ('set', driver.token, True) and link.calls[1] == ('ssh', 'check')
    check, start = ssh.commands[0].split(' && ')
    assert shlex.split(check)[3] == 'check' and shlex.split(start)[3] == 'start'
    words = shlex.split(start)
    assert '--driver' in words and words[words.index('--lease-s') + 1] == str(arm_link.LEASE_S)
    shipped = words[words.index('--config-b64') + 1]
    assert base64.b64decode(shipped).decode() == 'pwm_channels: [0, 1, 2, 3]\n'
    assert words[words.index('--token') + 1] == driver.token


def test_a_unit_already_running_is_never_adopted_for_arming():
    driver, link, _, _ = remote(Ssh({'state': 'running', 'adopted': True, 'by': {'host': 'blastoise-odroid'}}))
    driver.start([('hardware', ['x'])], '')
    driver.worker.shutdown(wait=True)
    assert 'already runs on blastoise-odroid' in driver.error and not driver.running('hardware')
    assert link.calls[-1] == ('set', driver.token, False)       # stop beating for it


def test_a_failed_ssh_start_is_the_startup_failure():
    driver, link, _, _ = remote(Ssh(down=True))
    driver.start([('hardware', ['x'])], '')
    driver.worker.shutdown(wait=True)
    assert driver.error == 'blastoise-odroid: ssh: timeout' and not driver.running('hardware')
    assert driver.last_line('hardware') == driver.error


def test_running_follows_the_boats_state_and_faults_when_it_goes_stale():
    driver, link, _, now = remote()
    driver.start([('hardware', ['x']), ('mux', ['y'])], '')
    assert driver.running('hardware') and not driver.confirmed()      # nothing heard yet
    link.report('someone else', now[0])
    assert driver.running('hardware') and not driver.confirmed()
    link.report(driver.token, now[0], phase='waiting', children={})
    assert driver.running('hardware') and not driver.confirmed()
    link.report(driver.token, now[0])
    assert driver.running('mux') and driver.confirmed()
    now[0] += STATE_STALE_S + 0.1
    assert not driver.running('hardware') and not driver.confirmed()
    assert 'no state from blastoise-odroid' in driver.last_line('hardware')
    link.report(driver.token, now[0], phase='stopped', children={'hardware': False, 'mux': False},
                reason='operator link lost (no beat for 1 s)')
    assert not driver.running('hardware')
    assert driver.last_line('hardware') == 'operator link lost (no beat for 1 s)'
    link.report(driver.token, now[0], phase='stopped', children={'hardware': False},
                reason='hardware exited (1)', last={'hardware': 'OSError: no pwmchip'})
    assert driver.last_line('hardware') == 'hardware exited (1) (hardware: OSError: no pwmchip)'


def test_disarm_and_estop_act_over_dds_while_ssh_is_down():
    driver, link, ssh, _ = remote(Ssh(down=True))
    driver.token = 'tok'
    driver.stop()
    assert link.calls[:2] == [('set', 'tok', False), ('estop',)]    # before ssh, never waiting for it
    driver.worker.shutdown(wait=True)
    assert link.calls[2] == ('ssh', 'stop')


def test_readback_runs_on_the_boat_and_reports_its_answer():
    driver, _, ssh, _ = remote(Ssh({'confirmed': True}))
    assert driver.readback('a: 1\n', ['ros2', 'run', 'x', 'config_file:=@CONFIG@']) is True
    words = shlex.split(ssh.commands[0])
    assert words[:4] == ['python3', '-m', 'mhseals_testing.ros.arm_link', 'readback']
    driver, _, _, _ = remote(Ssh({'error': 'a thruster driver or mux already runs here (pids [9])'}))
    with pytest.raises(RuntimeError, match='already runs here'):
        driver.readback('', ['x'])


# --- target ------------------------------------------------------------------------------------

def registry(*hosts):
    return Registry({host.name: host for host in hosts}, {})


def profile_on(me, outputs='odroid'):
    return SimpleNamespace(outputs=outputs, placed=(me, {}))


LAPTOP = Host('tranquility', ('operator',), frozenset(), '192.168.0.2', user='peace')


def test_the_target_is_the_one_other_thrusters_host():
    assert arm_link.target(profile_on(LAPTOP), registry(LAPTOP, BOAT)) == (BOAT, None)
    assert arm_link.target(profile_on(BOAT), registry(LAPTOP, BOAT)) is None        # arms locally
    assert arm_link.target(profile_on(LAPTOP, 'topic'), registry(LAPTOP, BOAT)) is None
    assert arm_link.target(profile_on(LAPTOP), registry(LAPTOP)) is None            # refuses
    no_ssh = Host('blastoise-odroid', capabilities=frozenset({'thrusters'}), ssh=False)
    assert arm_link.target(profile_on(LAPTOP), registry(LAPTOP, no_ssh)) is None
    two = Host('other', capabilities=frozenset({'thrusters'}))
    assert arm_link.target(profile_on(LAPTOP), registry(LAPTOP, BOAT, two)) is None


# --- Arming over the remote driver -------------------------------------------------------------

class Ros:
    def __init__(self):
        self.subs, self.pubs = [], ['/thruster_pwm_node']

    def subscribers(self, topic):
        return self.subs

    def publishers(self, topic):
        return self.pubs if topic == '/thruster_pwm_node/status' else []

    def nodes(self):
        return ['/thruster_pwm_node'] if self.subs else []

    def parameter_service(self, name):
        return True


class NoProcesses:
    def __getattr__(self, name):
        raise AssertionError(f'a remote arming touched the local processes ({name})')


def remote_arming(tmp_path, ssh=None):
    driver, link, ssh, now = remote(ssh)
    ros = Ros()
    arming = Arming(load_profile('boat'), ros, NoProcesses(), load_defaults(),
                    tmp_path / 'hardware.yaml', clock=lambda: now[0], sleep=lambda s: None,
                    remote=driver)
    return arming, driver, link, ros, now


def test_remote_arm_ships_the_snapshot_and_arms_only_when_the_boat_confirms(tmp_path):
    arming, driver, link, ros, now = remote_arming(tmp_path)
    arming.begin()
    driver.worker.shutdown(wait=True)
    shipped = shlex.split(driver.call.commands[0].split(' && ')[1])
    config = base64.b64decode(shipped[shipped.index('--config-b64') + 1]).decode()
    assert config == (tmp_path / 'hardware.yaml').read_text()        # the session snapshot
    assert 'config_file:=@CONFIG@' in shipped and 'thruster_pwm_node' in shipped
    assert arming.poll() == 'starting'
    ros.subs = ['/thruster_pwm_node']
    assert arming.poll() == 'starting'                                 # graph alone is not enough
    link.report(driver.token, now[0])
    assert arming.poll() == 'armed' and arming.fault() == ''


def test_remote_arm_times_out_with_the_boats_reason(tmp_path):
    arming, driver, link, ros, now = remote_arming(tmp_path)
    arming.begin()
    link.report(driver.token, now[0], phase='waiting', children={})
    now[0] += arming.timeout + 1
    link.report(driver.token, now[0], phase='waiting', children={})
    with pytest.raises(RuntimeError, match='thruster_pwm_node is running but not discovered'):
        arming.poll()
    assert ('estop',) in link.calls                     # cancel stops it the same way


def armed(tmp_path):
    arming, driver, link, ros, now = remote_arming(tmp_path)
    arming.begin()
    ros.subs = ['/thruster_pwm_node']
    link.report(driver.token, now[0])
    assert arming.poll() == 'armed'
    return arming, driver, link, ros, now


def test_a_dead_remote_driver_is_a_fault_from_the_boats_state(tmp_path):
    arming, driver, link, ros, now = armed(tmp_path)
    link.report(driver.token, now[0], children={'hardware': False, 'mux': True},
                last={'hardware': 'Traceback: no pwm chip'})
    assert arming.fault() == 'hardware on blastoise-odroid stopped: Traceback: no pwm chip'


def test_a_silent_boat_is_a_fault_within_the_stale_bound(tmp_path):
    arming, driver, link, ros, now = armed(tmp_path)
    now[0] += STATE_STALE_S - 0.1
    assert arming.fault() == ''
    now[0] += 0.2
    assert 'hardware, mux on blastoise-odroid stopped: no state from' in arming.fault()


def test_a_driver_gone_from_the_graph_is_a_fault(tmp_path):
    arming, driver, link, ros, now = armed(tmp_path)
    ros.pubs = []
    assert arming.fault() == 'thruster_pwm_node on blastoise-odroid left the ROS graph'


def test_remote_disarm_stops_over_dds_then_ssh(tmp_path):
    arming, driver, link, ros, now = armed(tmp_path)
    driver.call.down = True
    arming.disarm()
    assert not arming.armed
    assert link.calls[-2:] == [('set', driver.token, False), ('estop',)]
    driver.worker.shutdown(wait=True)
    assert link.calls[-1] == ('ssh', 'stop')


def test_remote_pin_readback_runs_on_the_boat(tmp_path):
    arming, driver, link, ros, now = remote_arming(tmp_path, Ssh({'confirmed': True}))
    config = dict(load_defaults(), pwm_channels=[4, 5, 6, 7])
    assert arming.restart_driver(config, lambda: pytest.fail('read locally')) is True
    words = shlex.split(driver.call.commands[0])
    shipped = base64.b64decode(words[words.index('--config-b64') + 1]).decode()
    assert '- 4' in shipped or '[4, 5, 6, 7]' in shipped
    assert words[words.index('--') + 1:][:4] == ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node']


def test_the_cli_refuses_a_link_bound_that_is_not_short(capsys):
    with pytest.raises(SystemExit):
        arm_link.main(['run', '--token', 't', '--config-b64', '', '--link-s', '30', '--',
                       'hardware', 'x'])
    assert 'link-s' in capsys.readouterr().err


@pytest.mark.parametrize('verb', ['run', 'readback'])
def test_the_module_runs_as_a_cli(verb):
    done = subprocess.run([sys.executable, '-m', 'mhseals_testing.ros.arm_link', verb, '--help'],
                          capture_output=True, text=True, env=dict(os.environ))
    assert done.returncode == 0 and '--config-b64' in done.stdout
