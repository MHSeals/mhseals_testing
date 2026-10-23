"""Recorder argv and bag segments (bags.md section 2, ROS-free).

One session writes ``bag/NN/`` segments, each a ``ros2 bag record`` child. The Recording row
(``lean``, ``full``, ``off``) ends the current segment and, unless the new value is ``off``,
starts the next. Never ``-a``: it records images and clouds at ~50 MB/s and, on Jazzy, services.
"""
import os
from pathlib import Path
import re

import yaml

MODES = ('lean', 'full', 'off')
SPLIT_S = 120
CACHE_BYTES = 4 * 1024 * 1024
STOP_WAIT_S = 20  # SIGINT, up to 20 s for metadata.yaml, then SIGTERM, then SIGKILL (ProcessManager.stop)
CONFIG = Path(__file__).resolve().parents[2] / 'config'


def load_profiles(path=None):
    """config/bags/topics.yaml (source tree, else the installed share)."""
    path = Path(path or CONFIG / 'bags' / 'topics.yaml')
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'bags' / 'topics.yaml'
    return yaml.safe_load(path.read_text())


def record_command(segment, mode, profile=None, profiles=None, filters=None):
    """The ``ros2 bag record`` argv for one segment in ``lean`` or ``full`` mode.

    lean: ``--topics <list> -e <regex>`` (the regex *includes* more topics, so it is never
    ``--exclude-regex``), plus the profile's ``extra`` topics. full: ``--all-topics`` with the
    heavy-topic excludes from config/bags.yaml.
    """
    if mode not in ('lean', 'full'):
        raise ValueError(f'no recorder command for mode {mode!r}')
    common = ['ros2', 'bag', 'record', '-s', 'mcap', '-d', str(SPLIT_S),
              '--max-cache-size', str(CACHE_BYTES), '--node-name', f'rosbag2_recorder_{os.getpid()}',
              '--disable-keyboard-controls', '-o', str(segment)]
    if mode == 'full':
        from mhseals_testing.harness.processes import bag_filters
        regex, types = filters or bag_filters()
        types = ['--exclude-topic-types', *types] if types else []
        return ['ros2', 'bag', 'record', '--all-topics', '--exclude-regex', regex, *types, *common[3:]]
    profiles = profiles or load_profiles()
    lean = profiles['lean']
    topics = list(lean['topics'])
    for topic in profiles.get('extra', {}).get(profile, {}).get('topics', []):
        if topic not in topics:
            topics.append(topic)
    return ['ros2', 'bag', 'record', '--topics', *topics, '-e', lean['regex'], *common[3:]]


def next_segment(bag_dir):
    """``bag_dir/NN``: one past the highest existing two-digit segment, starting at 00."""
    taken = [int(p.name) for p in Path(bag_dir).glob('[0-9][0-9]*') if re.fullmatch(r'\d+', p.name)]
    return Path(bag_dir) / f'{max(taken, default=-1) + 1:02d}'


class Recorder:
    """The Recording row: ``set_mode`` ends the running segment and starts the next unless ``off``."""

    def __init__(self, processes, bag_dir, profile=None, profiles=None, filters=None):
        self.processes, self.bag_dir, self.profile = processes, Path(bag_dir), profile
        self.profiles, self.filters = profiles, filters
        self.mode, self.segment, self.last = 'off', None, None

    def set_mode(self, mode):
        """Switch mode; returns the new segment path (None for ``off``)."""
        if mode not in MODES:
            raise ValueError(f'Recording is one of {MODES}, not {mode!r}')
        self.processes.stop('bag', timeout=STOP_WAIT_S)
        self.mode, self.segment = mode, None
        if mode != 'off':
            self.bag_dir.mkdir(parents=True, exist_ok=True)
            self.segment = next_segment(self.bag_dir)
            if self.last is not None and self.segment.name <= self.last:  # ros2 makes the dir late
                self.segment = self.bag_dir / f'{int(self.last) + 1:02d}'
            self.last = self.segment.name
            self.processes.start('bag', record_command(self.segment, mode, self.profile,
                                                       self.profiles, self.filters))
        return self.segment

    def stop(self):
        self.set_mode('off')
