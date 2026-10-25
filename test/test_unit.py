"""Units: a device part kept by an ownerless keeper on a lease; start, adopt, renew, stop, refuse."""

import json
import os
import signal
import subprocess
import sys

import pytest

from mhseals_testing.harness import unit
from mhseals_testing.harness.keeper import lease_expired
from mhseals_testing.harness.registry import alive
from test_keeper import WRAPPER, wait_for

SLEEP = [sys.executable, '-c', 'import time; time.sleep(60)']


@pytest.fixture(autouse=True)
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path / 'run'))
    (tmp_path / 'run').mkdir()
    yield
    for record in unit.statuses():
        if record['state'] == 'running':
            unit.stop(record['unit'], wait=5)


def group_gone(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return True
    return False


def test_start_status_stop(tmp_path):
    started = unit.start('mhseals-camera', SLEEP, by={'host': 'blastoise-odroid', 'pid': 7})
    assert started['state'] == 'running' and not started['adopted']
    assert started['by'] == {'host': 'blastoise-odroid', 'pid': 7} and alive(started['pid'])
    assert started['lease_age'] < 5 and started['log'].startswith(str(tmp_path))
    stopped = unit.stop('mhseals-camera')
    assert stopped['state'] == 'exited' and stopped['code'] is not None
    assert not alive(started['pid']) and group_gone(started['pid'])


def test_a_second_start_adopts_the_running_unit():
    first = unit.start('mhseals-lidar', SLEEP)
    second = unit.start('mhseals-lidar', SLEEP)
    assert second['adopted'] and second['keeper'] == first['keeper']
    assert [record['unit'] for record in unit.statuses()] == ['mhseals-lidar']


def test_stop_reaches_a_wrapper_that_ignores_sigint():
    started = unit.start('mhseals-ekf', [sys.executable, '-c', WRAPPER])
    assert unit.stop('mhseals-ekf')['state'] == 'exited'
    assert group_gone(started['pid'])


def test_an_unrenewed_lease_stops_the_unit_and_renewing_keeps_it():
    kept = unit.start('mhseals-slam', SLEEP, lease_s=1.0)
    dropped = unit.start('mhseals-detections', SLEEP, lease_s=1.0)

    def renewed_while_dropped_stops():
        unit.renew('mhseals-slam')
        return unit.status('mhseals-detections')['state'] == 'exited'
    assert wait_for(renewed_while_dropped_stops, timeout=8)
    assert not alive(dropped['pid']) and alive(kept['pid'])
    with pytest.raises(unit.UnitError, match='not running'):
        unit.renew('mhseals-detections')


def test_the_unit_survives_the_process_that_started_it(tmp_path):
    """Like `docker exec -d` returning: the starter exits, the part keeps running."""
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    out = subprocess.run([sys.executable, '-m', 'mhseals_testing.harness.unit', 'start', 'mhseals-odometry',
                          '--', *SLEEP], capture_output=True, text=True, env=env, timeout=20)
    record = json.loads(out.stdout)
    assert out.returncode == 0 and record['state'] == 'running'
    assert unit.status('mhseals-odometry')['state'] == 'running'


def test_a_killed_keeper_takes_its_child_down():
    started = unit.start('mhseals-costmap', SLEEP)
    os.kill(started['keeper'], signal.SIGKILL)
    assert wait_for(lambda: not alive(started['pid']))     # PDEATHSIG from the keeper


@pytest.mark.parametrize('argv', [
    ['ros2', 'run', 'mhseals_hardware', 'thruster_pwm_node'],
    ['ros2', 'run', 'mhseals_hardware', 'command_mux', '--ros-args'],
])
def test_drivers_never_run_as_units_even_with_the_commander_go_ahead(argv):
    with pytest.raises(unit.UnitError, match='never runs as a unit'):
        unit.start('mhseals-x', argv, allow_commander=True)


def kinds(profile, part):
    return {'navigation': 'commander', 'mission': 'commander', 'camera': 'sensing',
            'lidar': 'sensing'}.get(part)


@pytest.mark.parametrize('argv, expected', [
    (['ros2', 'launch', 'mhseals_nav', 'navigation.launch.py', 'command_mux:=true'], True),
    (['ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=odroid', 'only:=navigation'], True),
    (['ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=jetson', 'only:=camera,lidar'], False),
    (['ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=jetson', 'only:=camera,ghost'], True),
    (['ros2', 'launch', 'mhseals_nav', 'sensors.launch.py'], False),
])
def test_commander_parts_need_the_tuis_go_ahead(argv, expected):
    assert unit.commander(argv, kinds) is expected
    if expected:
        with pytest.raises(unit.UnitError, match='can command motion'):
            unit.refuse(argv, False, kinds)
    unit.refuse(argv, True, kinds)          # the go-ahead allows it; a driver it never would


def test_a_whole_device_launch_without_readable_profiles_counts_as_commander(monkeypatch):
    monkeypatch.setitem(sys.modules, 'mhseals_nav.device_profile', None)
    assert unit.commander(['ros2', 'launch', 'mhseals_nav', 'device.launch.py', 'device:=jetson'])


def test_stop_and_status_of_an_unknown_unit_are_off_and_bad_names_are_refused():
    assert unit.stop('mhseals-ghost') == {'unit': 'mhseals-ghost', 'state': 'off'}
    for name in ('camera', 'mhseals-../x', 'mhseals-Cam'):
        with pytest.raises(unit.UnitError, match='mhseals-<part>'):
            unit.status(name)


def test_the_commander_argument_is_not_in_the_help(capsys):
    with pytest.raises(SystemExit):
        unit.main(['start', '--help'])
    assert '--commander' not in capsys.readouterr().out


def test_lease_expired(tmp_path):
    lease = tmp_path / 'l'
    assert lease_expired(lease, 10, 0)
    lease.touch()
    mtime = lease.stat().st_mtime
    assert not lease_expired(lease, 10, mtime + 5) and lease_expired(lease, 10, mtime + 11)
