import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from mhseals_testing.harness.bags import record_command
from mhseals_testing.harness.processes import ProcessManager, unique_path


def test_children_log_stop_cleanly_and_report_exit(tmp_path):
    manager = ProcessManager(tmp_path / 'logs')
    manager.start('sleeper', [sys.executable, '-c', 'import time; print("up", flush=True); time.sleep(60)'])
    manager.start('quick', [sys.executable, '-c', 'raise SystemExit(3)'])
    manager.processes['quick'].wait(timeout=5)
    assert manager.running('sleeper') and manager.exited() == ['quick']
    assert 'EXITED 3' in manager.summary()
    manager.stop_all()
    assert manager.processes == {} and manager.summary() == 'None'
    assert 'up' in (tmp_path / 'logs' / 'sleeper.log').read_text()


def test_unique_paths_and_bag_records_everything(tmp_path):
    (tmp_path / 'session').mkdir()
    (tmp_path / 'session_01').mkdir()
    assert unique_path(tmp_path / 'session').name == 'session_02'
    assert unique_path(tmp_path / 'other').name == 'other'


def test_concurrent_unique_path_claims_get_distinct_names(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(8) as pool:
        names = list(pool.map(lambda _: unique_path(tmp_path / 'same').name, range(8)))
    assert len(set(names)) == 8 and names[0].startswith('same')


def test_full_bag_excludes_heavy_topics_instead_of_recording_all(tmp_path):
    command = record_command(tmp_path / 'b', 'full')
    assert '-a' not in command and '--all-topics' in command
    regex = re.compile(command[command.index('--exclude-regex') + 1])
    heavy_topics = ('/front_camera/depth/image', '/points', '/costmap/costmap', '/cam/image_raw/compressed',
                    '/lidar/cloud_registered', '/cam/camera_info', '/deskewed_points')
    for heavy in heavy_topics:
        assert regex.fullmatch(heavy)
    kept_topics = ('/mhseals_testing/events', '/mavros/local_position/odom', '/waypoints',
                   '/mavros/mission/waypoints', '/mavros/rallypoint/rallypoints',
                   '/mavros/camera/image_captured', '/mhseals_testing/waypoints', '/perception/image_status')
    for kept in kept_topics:
        assert not regex.fullmatch(kept)
    assert command[-2:] == ['-o', str(tmp_path / 'b')]


def test_full_bag_excludes_heavy_message_types_the_name_regex_misses(tmp_path):
    command = record_command(tmp_path / 'b', 'full')
    start = command.index('--exclude-topic-types') + 1
    types = command[start:command.index('-s')]
    for heavy in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
                  'stereo_msgs/msg/DisparityImage'):
        assert heavy in types
    assert not any(t.startswith('--') for t in types)


def test_full_bag_keeps_the_minimap_costmap_and_object_tracks_but_not_rtabmap_grids(tmp_path):
    command = record_command(tmp_path / 'b', 'full')
    regex = re.compile(command[command.index('--exclude-regex') + 1])
    types = command[command.index('--exclude-topic-types') + 1:command.index('-s')]
    assert 'nav_msgs/msg/OccupancyGrid' not in types
    for kept in ('/mhseals_testing/minimap/costmap', '/objects/map'):
        assert not regex.fullmatch(kept)
    for dropped in ('/map', '/grid_prob_map', '/local_costmap/costmap', '/global_costmap/costmap_updates'):
        assert regex.fullmatch(dropped)


def test_child_dies_with_a_killed_parent_that_ignored_sigint(tmp_path):
    # A suite started with & or from a script inherits SIGINT=SIG_IGN; the child must not.
    parent = (
        'import signal, subprocess, sys, time\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
        'from mhseals_testing.harness.processes import ProcessManager\n'
        f'manager = ProcessManager({str(tmp_path / "logs")!r})\n'
        'child = manager.start("sleeper", [sys.executable, "-c", "import time; time.sleep(60)"])\n'
        'print(child.pid, flush=True)\n'
        'time.sleep(60)\n')
    holder = subprocess.Popen([sys.executable, '-c', parent], stdout=subprocess.PIPE, text=True)
    child = int(holder.stdout.readline())
    holder.kill()
    holder.wait()
    deadline = time.time() + 5
    while time.time() < deadline and os.path.exists(f'/proc/{child}') and 'Z' not in _state(child):
        time.sleep(0.1)
    assert not os.path.exists(f'/proc/{child}') or 'Z' in _state(child)


def test_stop_interrupts_a_child_when_the_suite_ignores_signals(tmp_path):
    ignored = [signal.signal(signum, signal.SIG_IGN) for signum in (signal.SIGINT, signal.SIGTERM)]
    manager = ProcessManager(tmp_path / 'logs')
    try:
        child = manager.start('sleeper', [sys.executable, '-c', 'import time; print("up", flush=True); time.sleep(60)'])
        _wait_up(tmp_path / 'logs' / 'sleeper.log')  # SIGINT during interpreter start exits 1
        started = time.time()
        manager.stop('sleeper', timeout=3)
        assert child.returncode == -signal.SIGINT
        assert time.time() - started < 3
    finally:
        signal.signal(signal.SIGINT, ignored[0])
        signal.signal(signal.SIGTERM, ignored[1])


def test_a_child_gets_default_sigterm_when_the_suite_ignores_it(tmp_path):
    previous = signal.signal(signal.SIGTERM, signal.SIG_IGN)
    manager = ProcessManager(tmp_path / 'logs')
    try:
        child = manager.start('sleeper', [sys.executable, '-c', 'import time; time.sleep(60)'])
        child.terminate()
        child.wait(timeout=3)
        assert child.returncode == -signal.SIGTERM
    finally:
        signal.signal(signal.SIGTERM, previous)
        manager.stop_all()


def _state(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
    except OSError:
        return 'Z'


def test_manager_children_survive_a_handover_and_record_for_the_registry(tmp_path):
    manager = ProcessManager(tmp_path / 'logs')
    child = manager.start('sleeper', [sys.executable, '-c', 'import time; time.sleep(60)'])
    record = manager.records()['sleeper']
    assert record['pid'] == child.pid and record['pgid'] == child.pid and record['keeper']
    other = ProcessManager(tmp_path / 'logs')
    adopted = other.adopt('sleeper', record)
    assert other.running('sleeper') and adopted.pid == child.pid
    other.stop('sleeper', timeout=3)
    assert not other.running('sleeper')
    assert manager.processes['sleeper'].wait(timeout=5) is not None


def _wait_up(log):
    deadline = time.time() + 5
    while time.time() < deadline and 'up' not in (log.read_text() if log.exists() else ''):
        time.sleep(0.02)


class SlowChild:
    """Exits 0.3 s after its SIGINT, like a node shutting down."""

    def __init__(self, order, name):
        self.order, self.name, self.code = order, name, None

    def poll(self):
        return self.code

    def signal(self, signum):
        self.order.append(self.name)

    def wait(self, timeout):
        time.sleep(0.3)
        self.code = 0


def test_stop_all_stops_children_at_once_and_the_recorder_after(tmp_path):
    manager, order = ProcessManager(tmp_path / 'logs'), []
    manager.processes = {name: SlowChild(order, name) for name in ('bag', 'a', 'b', 'c')}
    began = time.monotonic()
    manager.stop_all()
    assert time.monotonic() - began < 0.9 and order[-1] == 'bag' and manager.processes == {}


def test_discovery_settles_once_the_graph_stops_growing():
    from mhseals_testing.ui.app import settle_discovery
    now, graphs = [0.0], iter([['/me'], ['/me', '/a'], ['/me', '/a', '/b'], ['/me', '/a', '/b']])
    found = settle_discovery(lambda: next(graphs), clock=lambda: now[0],
                             sleep=lambda s: now.__setitem__(0, now[0] + s))
    assert found == {'/me', '/a', '/b'} and now[0] == 0.75
    quiet = settle_discovery(lambda: ['/me'], clock=lambda: now[0], sleep=lambda s: now.__setitem__(0, now[0] + s))
    assert quiet == {'/me'}
