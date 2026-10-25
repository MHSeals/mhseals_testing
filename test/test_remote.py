"""Remote parts: the ssh argv, the unit commands, and Units over a fake ssh and the real unit CLI."""

import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile

import pytest

from mhseals_testing.devices import Host
from mhseals_testing.harness import remote
from mhseals_testing.harness.components import Component
from mhseals_testing.harness.remote import Units, reason, ssh_argv, unit_command
from mhseals_testing.harness.unit import LEASE_S

JETSON = Host('blastoise-jetson', address='192.168.0.153', user='roboboat', container='roboboat_dev',
              via='tranquility')
GATEWAY = Host('tranquility', address='192.168.0.2', user='peace', container='roboboat_dev')


CAMERA_ARGV = ('ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=jetson', 'only:=camera')


def item(part='camera', host=JETSON, argv=CAMERA_ARGV, kind='sensing'):
    return Component(part, part.capitalize(), tuple(argv), host=host, part=part, profile='jetson',
                     kind=kind)


@pytest.fixture(autouse=True)
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'run'))
    (tmp_path / 'run').mkdir()


def test_ssh_argv_reaches_the_container_with_the_operator_key_through_the_jump(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'cfg'))
    argv = ssh_argv(JETSON, 'echo hi', GATEWAY)
    assert argv[0] == 'ssh' and 'BatchMode=yes' in argv and f'ConnectTimeout={remote.CONNECT_S}' in argv
    assert f'ControlPath={tmp_path}/run/mhseals/ssh-%C' in argv
    assert str(tmp_path / 'cfg/mhseals/ssh/id_ed25519') in argv
    jump = next(arg for arg in argv if arg.startswith('ProxyCommand='))
    assert '-W %h:%p peace@192.168.0.2' in jump and 'id_ed25519' in jump
    assert 'ssh-%%C' in jump and 'ssh-%C' not in jump.replace('%%C', '')
    assert argv[-2] == 'roboboat@192.168.0.153'
    # One line for the remote shell, which splits it back into exactly these words.
    assert shlex.split(argv[-1]) == ['docker', 'exec', '-i', 'roboboat_dev', 'bash', '-c', 'echo hi']


@pytest.mark.parametrize('hostile', ['x; touch /tmp/pwned', '$(touch /tmp/pwned)', '`id`', 'a b',
                                     "it's", 'x && reboot', '-oProxyCommand=sh'])
def test_no_value_reaches_the_remote_shell_as_syntax(hostile):
    # Even a container name past the registry's check stays one word for the remote shell.
    host = Host('h', address='10.0.0.9', user='u', container=hostile)
    argv = ssh_argv(host, f'echo {hostile}')
    assert shlex.split(argv[-1]) == ['docker', 'exec', '-i', hostile, 'bash', '-c', f'echo {hostile}']


@pytest.mark.skipif(shutil.which('ssh') is None, reason='no ssh client')
def test_ssh_expands_the_proxy_command_and_hands_the_inner_ssh_its_control_path(tmp_path, monkeypatch):
    # A stand-in inner ssh on PATH records what the real outer ssh expanded the ProxyCommand into.
    real = shutil.which('ssh')
    bin_dir, seen = tmp_path / 'bin', tmp_path / 'seen'
    bin_dir.mkdir()
    (bin_dir / 'ssh').write_text(f'#!/bin/sh\nprintf "%s\\n" "$@" > {seen}\nexit 1\n')
    (bin_dir / 'ssh').chmod(0o755)
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'cfg'))
    run = tempfile.TemporaryDirectory(prefix='mh')     # a socket path stays under 108 bytes
    monkeypatch.setenv('XDG_RUNTIME_DIR', run.name)
    argv = ssh_argv(JETSON, 'true', GATEWAY)
    config = subprocess.run([real, '-G', *argv[1:-1]], capture_output=True, text=True, timeout=10)
    assert config.returncode == 0, config.stderr
    monkeypatch.setenv('PATH', f'{bin_dir}:{os.environ["PATH"]}')
    done = subprocess.run([real, *argv[1:]], capture_output=True, text=True, timeout=20,
                          stdin=subprocess.DEVNULL)
    assert 'unknown key' not in done.stderr and 'percent_expand' not in done.stderr, done.stderr
    assert seen.exists(), done.stderr
    inner = seen.read_text().splitlines()
    assert f'ControlPath={run.name}/mhseals/ssh-%C' in inner
    run.cleanup()
    assert inner[-3:] == ['-W', '192.168.0.153:22', 'peace@192.168.0.2']


def test_the_commander_argument_is_only_set_when_asked():
    plain = shlex.split(unit_command('start', 'mhseals-camera', ('ros2', 'launch', 'x'), {'host': 'a'}))
    assert '--commander' not in plain and plain[plain.index('--') + 1:] == ['ros2', 'launch', 'x']
    assert '--commander' in shlex.split(unit_command('start', 'mhseals-nav', ('x',), commander=True))
    assert shlex.split(unit_command('stop', 'mhseals-camera'))[-2:] == ['stop', 'mhseals-camera']


@pytest.mark.parametrize('stderr, short', [
    ('ssh: connect to host 192.168.0.153 port 22: Connection timed out', 'ssh: timeout'),
    ('roboboat@192.168.0.153: Permission denied (publickey).', 'ssh: key refused'),
    ('Error response from daemon: container abc is not running', 'ssh: container stopped'),
    ('', 'ssh: exit 255')])
def test_failures_read_as_a_short_reason(stderr, short):
    assert reason(stderr, 255) == short


class FakeSsh:
    def __init__(self):
        self.calls, self.reply, self.error, self.extra = [], {}, None, []

    def __call__(self, host, command, jump, timeout):
        self.calls.append((host.name, command, jump.name if jump else None))
        if self.error:
            return [], self.error
        return [*self.extra, *(dict(self.reply.get(word, {}), unit=word) for word in self.reply
                               if 'status' in command)], None


def test_a_running_unit_is_adopted_then_renewed_and_a_failed_read_keeps_the_report():
    ssh = FakeSsh()
    now = [100.0]
    units = Units([item()], hosts={'tranquility': GATEWAY}, run=ssh, clock=lambda: now[0])
    ssh.reply = {'mhseals-camera': {'state': 'running'}}
    units.refresh()
    assert units.status(item()) == ({'unit': 'mhseals-camera', 'state': 'running'}, None)
    assert units.held_by_host() == {'blastoise-jetson': ['Camera']}
    assert ssh.calls[0][2] == 'tranquility' and 'renew' not in ssh.calls[0][1]
    units.refresh()
    assert 'renew mhseals-camera' in ssh.calls[1][1] and ssh.calls[1][1].endswith('status; }')
    assert units.trouble() == {}
    ssh.error = 'ssh: timeout'
    now[0] = 103.0
    units.refresh()
    now[0] = 108.0
    units.refresh()
    assert units.status(item()) == ({'unit': 'mhseals-camera', 'state': 'running'}, 'ssh: timeout')
    assert units.trouble() == {'blastoise-jetson': ('ssh: timeout', 103.0, 100.0)}   # since the first failure
    assert units.held_by_host() == {'blastoise-jetson': ['Camera']}   # still renewed once it answers
    ssh.error = None
    units.refresh()
    assert units.trouble() == {}


def test_the_status_read_probes_each_part_and_falls_back_on_an_older_device():
    command = remote.status_command([item()])
    probe, fallback = command.split(' || ')
    words = shlex.split(probe.removeprefix('{ err=$(').removesuffix(' 2>&1 >&3); } 3>&1'))
    assert words[:5] == ['python3', '-m', 'mhseals_testing.harness.unit', 'status', '--probe']
    assert json.loads(words[5]) == [{'unit': 'mhseals-camera', 'part': 'camera', 'profile': 'jetson',
                                     'launch': None, 'nodes': []}]
    assert fallback.endswith(f'; {unit_command("status")}; }}')


def test_a_failed_probe_is_reported_never_read_as_off():
    """The real shell line on a device whose probe fails: its error comes back, not a quiet off."""
    bad = Component('camera', 'Camera', CAMERA_ARGV, host=JETSON, part='bad name', profile='jetson')
    results, error = local(JETSON, remote.status_command([bad]), None, 30)
    assert error is None
    assert any('unit' not in found and (found.get('error') or found.get('probe_error'))
               for found in results)


def test_a_probe_error_keeps_the_unit_facts_reads_the_rest_unknown_and_raises_trouble():
    ssh = FakeSsh()
    dogs = Component('watchdogs', 'Watchdogs', CAMERA_ARGV, host=JETSON, part='watchdogs',
                     profile='odroid', carried_by='Odometry')
    now = [50.0]
    units = Units([item(), dogs], hosts={'tranquility': GATEWAY}, run=ssh, clock=lambda: now[0])
    ssh.reply = {'mhseals-camera': {'state': 'running'}}
    ssh.extra = [{'probe_error': 'error: unrecognized arguments: --probe'}]
    units.refresh()
    assert units.status(item())[0]['state'] == 'running'                  # a unit is a fact
    assert units.status(dogs)[0] == {'unit': 'mhseals-watchdogs', 'state': 'unknown',
                                     'probe_error': 'error: unrecognized arguments: --probe'}
    assert units.probe_trouble() == {'blastoise-jetson': ('error: unrecognized arguments: --probe', 50.0)}
    now[0] = 60.0
    units.refresh()
    assert units.probe_trouble()['blastoise-jetson'][1] == 50.0          # since the first failure
    ssh.extra = []
    units.refresh()
    assert units.probe_trouble() == {}
    assert units.status(dogs)[0]['state'] == 'off'


def test_options_share_one_connection_with_keepalives():
    argv = ssh_argv(GATEWAY, 'true')
    assert f'ControlPersist={remote.PERSIST_S}' in argv and 'ControlMaster=auto' in argv
    assert f'ServerAliveInterval={remote.ALIVE_S}' in argv


def test_an_ssh_failure_is_retried_once(monkeypatch):
    runs = []

    def run(argv, **_):
        runs.append(argv)
        return subprocess.CompletedProcess(argv, 255 if len(runs) == 1 else 0,
                                           '{"unit": "mhseals-camera", "state": "off"}\n', '')
    monkeypatch.setattr(remote.subprocess, 'run', run)
    assert remote.call(GATEWAY, 'true') == ([{'unit': 'mhseals-camera', 'state': 'off'}], None)
    assert len(runs) == 2


def test_a_stale_unit_of_a_carried_part_is_never_adopted_or_renewed():
    """The boat's watchdogs run inside odometry's launch: an old mhseals-watchdogs unit must expire."""
    ssh = FakeSsh()
    ssh.reply = {'mhseals-camera': {'state': 'running'}, 'mhseals-watchdogs': {'state': 'running'}}
    dogs = Component('watchdogs', 'Safety watchdogs', CAMERA_ARGV, host=JETSON, part='watchdogs',
                     profile='odroid', carried_by='Odometry')
    units = Units([item(), dogs], hosts={'tranquility': GATEWAY}, run=ssh)
    units.refresh()
    units.refresh()
    assert units.held_by_host() == {'blastoise-jetson': ['Camera']}
    assert not any('renew mhseals-watchdogs' in call[1] for call in ssh.calls)
    assert units.status(dogs)[0]['state'] == 'running'      # still read: the device's report


def test_a_watching_tui_reads_status_but_keeps_nothing_alive():
    ssh = FakeSsh()
    ssh.reply = {'mhseals-camera': {'state': 'running'}}
    units = Units([item()], hosts={'tranquility': GATEWAY}, run=ssh, renew=False)
    units.refresh()
    units.refresh()
    assert units.held_by_host() == {} and not any('renew' in call[1] for call in ssh.calls)


def test_a_host_without_ssh_is_unreachable_without_a_call():
    vlp = Host('vlp16', address='192.168.0.201', ssh=False)
    ssh = FakeSsh()
    units = Units([item(host=vlp)], run=ssh)
    units.refresh()
    assert units.status(item(host=vlp)) == (None, 'no ssh to vlp16') and ssh.calls == []


def local(host, command, jump, timeout):
    """ssh + docker exec stood in for by this machine: the real unit CLI runs here."""
    command = command.replace('python3 -m', f'{shlex.quote(sys.executable)} -m')
    done = subprocess.run(['bash', '-c', command], capture_output=True, text=True, timeout=timeout)
    results = [json.loads(line) for line in done.stdout.splitlines() if line.startswith('{')]
    return (results, None) if results or done.returncode == 0 else ([], reason(done.stderr, done.returncode))


def test_start_adopt_renew_and_stop_through_the_real_unit_cli():
    sleeper = item('sleeper', argv=('sleep', '60'))
    first = Units([sleeper], hosts={'tranquility': GATEWAY}, run=local, by={'host': 'operator'})
    first.do_start(sleeper, False)
    assert first.take_notices() == ['Sleeper started on blastoise-jetson (mhseals-sleeper)']
    record, error = first.status(sleeper)
    assert error is None and record['state'] == 'running' and record['by'] == {'host': 'operator'}
    second = Units([sleeper], hosts={'tranquility': GATEWAY}, run=local)      # another TUI
    second.refresh()
    assert second.held_by_host() == {'blastoise-jetson': ['Sleeper']}       # adopted
    second.refresh()                                                         # renews it
    assert first.close(stop=True) == 'stopped Sleeper'
    second.refresh()
    assert second.status(sleeper)[0]['state'] == 'exited' and second.held_by_host() == {}


def test_quit_stops_only_what_this_tui_started_and_keeps_adopted_units():
    sleeper, camera = item('sleeper', argv=('sleep', '60')), item('napper', argv=('sleep', '60'))
    first = Units([sleeper], hosts={'tranquility': GATEWAY}, run=local)       # another operator
    first.do_start(sleeper, False)
    second = Units([sleeper, camera], hosts={'tranquility': GATEWAY}, run=local)
    second.refresh()                                                         # adopts the sleeper
    second.do_start(camera, False)                                           # starts its own
    assert second.held_by_host(adopted=False) == {'blastoise-jetson': ['Napper']}
    assert second.held_by_host(adopted=True) == {'blastoise-jetson': ['Sleeper']}
    assert second.close(stop=True) == f'stopped Napper; kept Sleeper (lease {LEASE_S:.0f} s)'
    first.refresh()
    assert first.status(sleeper)[0]['state'] == 'running'                   # the other TUI's unit runs on
    third = Units([sleeper], hosts={'tranquility': GATEWAY}, run=local)
    third.refresh()
    assert third.close(stop=True, adopted=True) == 'stopped Sleeper'        # only on the explicit choice
    first.refresh()
    assert first.status(sleeper)[0]['state'] == 'exited'


def test_a_commander_unit_is_refused_without_the_go_ahead():
    nav = item('navigation', argv=('bash', '-c', 'sleep 60', 'navigation.launch.py'), kind='commander')
    units = Units([nav], hosts={'tranquility': GATEWAY}, run=local)
    units.do_start(nav, False)
    assert units.status(nav) == (None, None)
    assert 'can command motion' in units.take_notices()[0]
    units.do_start(nav, True)
    assert units.status(nav)[0]['commander'] is True
    assert units.close(stop=True) == 'stopped Navigation'


def test_a_driver_is_refused_even_with_the_go_ahead():
    driver = item('pwm', argv=('ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node'), kind='commander')
    units = Units([driver], hosts={'tranquility': GATEWAY}, run=local)
    units.do_start(driver, True)
    assert units.status(driver) == (None, None) and 'never runs as a unit' in units.take_notices()[0]
