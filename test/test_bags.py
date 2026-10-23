"""Recorder argv per mode, segment numbering, and the ROS-gated checks against real ros2 bag."""
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

from mhseals_testing.harness.bags import Recorder, load_profiles, next_segment, record_command

FILTERS = ('.*image$', ['sensor_msgs/msg/Image'])


def value(command, flag):
    return command[command.index(flag) + 1]


def test_lean_argv_lists_topics_and_includes_the_regex(tmp_path):
    command = record_command(tmp_path / '00', 'lean')
    topics = load_profiles()['lean']['topics']
    assert command[:5] == ['ros2', 'bag', 'record', '--topics', topics[0]]
    assert command[4:4 + len(topics)] == topics
    assert value(command, '-e') == load_profiles()['lean']['regex']
    assert '--exclude-regex' not in command and '-a' not in command and '--all-topics' not in command
    assert (value(command, '-s'), value(command, '-d'), value(command, '--max-cache-size')) == ('mcap', '120', '4194304')
    assert value(command, '-o') == str(tmp_path / '00')


def test_lean_argv_merges_profile_extras_once(tmp_path):
    plain = record_command(tmp_path, 'lean', 'boat')
    crane = record_command(tmp_path, 'lean', 'crane_full_stack')
    assert '/crane/odom' not in plain and '/crane/odom' in crane and crane.count('/clock') == 1


def test_full_argv_excludes_heavy_topics_and_never_uses_a(tmp_path):
    command = record_command(tmp_path / '01', 'full', filters=FILTERS)
    assert '--all-topics' in command and '-a' not in command and '--topics' not in command
    assert value(command, '--exclude-regex') == FILTERS[0]
    assert value(command, '--exclude-topic-types') == 'sensor_msgs/msg/Image'
    assert value(command, '-d') == '120' and value(command, '-o') == str(tmp_path / '01')


def test_off_has_no_command(tmp_path):
    with pytest.raises(ValueError):
        record_command(tmp_path, 'off')


def test_next_segment_numbers_after_the_highest(tmp_path):
    assert next_segment(tmp_path / 'bag') == tmp_path / 'bag' / '00'
    for name in ('00', '01', '04', 'notes'):
        (tmp_path / 'bag' / name).mkdir(parents=True, exist_ok=True)
    assert next_segment(tmp_path / 'bag').name == '05'


class FakeManager:
    """ProcessManager's start/stop/running without keepers: ``started`` holds each command."""

    def __init__(self):
        self.started, self.live = [], set()

    def start(self, name, command):
        self.started.append(command)
        self.live.add(name)

    def stop(self, name, timeout=10):
        self.live.discard(name)

    def running(self, name):
        return name in self.live


def test_off_then_on_starts_the_next_segment(tmp_path):
    manager = FakeManager()
    started = manager.started
    recorder = Recorder(manager, tmp_path / 'bag', filters=FILTERS)
    first = recorder.set_mode('lean')
    assert first.name == '00' and started[0][-1] == str(first)
    assert recorder.set_mode('off') is None and not manager.running('bag')
    second = recorder.set_mode('full')
    assert second.name == '01' and '--all-topics' in started[1] and recorder.mode == 'full'
    with pytest.raises(ValueError):
        recorder.set_mode('loud')


needs_ros = pytest.mark.skipif(shutil.which('ros2') is None, reason='needs ros2')


@needs_ros
def test_ros_accepts_topics_with_include_regex(tmp_path):
    command = record_command(tmp_path / '00', 'lean')
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               start_new_session=True)
    time.sleep(4)
    assert process.poll() is None, process.stdout.read()  # an argv error would have exited already
    process.send_signal(signal.SIGINT)
    output = process.communicate(timeout=20)[0]
    assert 'unrecognized' not in output and 'error' not in output.lower().replace('0 error', '')
    assert (tmp_path / '00' / 'metadata.yaml').exists()  # SIGINT finalizes the segment
