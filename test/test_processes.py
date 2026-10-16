import sys

from mhseals_testing.ros.processes import ProcessManager, bag_command, unique_path


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
    assert bag_command(tmp_path / 'b')[:4] == ['ros2', 'bag', 'record', '-a']
