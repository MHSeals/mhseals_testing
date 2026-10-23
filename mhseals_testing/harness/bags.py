"""Bags (bags.md, ROS-free): the recorder's argv and segments, and the sessions on disk.

Recorder: one session writes ``bag/NN/`` segments, each a ``ros2 bag record`` child. The
Recording row (``lean``, ``full``, ``off``) ends the current segment and, unless the new value
is ``off``, starts the next. Never ``-a``: it records images and clouds at ~50 MB/s and, on
Jazzy, services.

Sessions: list them, size the disk, pick prune candidates, edit titles and pins. Sessions live
under ``data.root()`` as ``sessions/<id>/`` (``session.json``, optional ``summary.json``,
``runs/``, ``events.jsonl``, ``bag/NN/``). The older flat layout (``<root>/<name>/`` next to
``<name>.report.json``) is still listed; it keeps its edits in ``<name>.meta.json`` because it
has no ``session.json``. Listing never starts or stops a recorder.
"""

from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import re
import shutil

import yaml

from mhseals_testing import data

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


# --- sessions on disk ---------------------------------------------------------------

SCHEMA = 'mhseals.session/1'
EDITABLE = ('title', 'note', 'pinned', 'sent')
RESERVED = {'sessions', 'logs', 'runs', 'config-history', 'calibration', 'sim'}   # root folders that are not legacy bags


@dataclass
class Bag:
    """One session as the Bags tab shows it."""
    id: str
    path: Path
    legacy: bool = False
    title: str = ''
    note: str = ''
    pinned: bool = False
    sent: bool = False
    recording: bool = False
    unfinalized: bool = False
    empty: bool = False
    size: int = 0              # bytes of everything under the session
    start: float = 0.0         # epoch seconds
    duration: float = 0.0      # seconds
    markers: int = 0
    health_fails: int = 0
    outcome: str = ''
    sim: bool = False

    @property
    def meta_path(self):
        """The file that holds title, note, pinned and sent."""
        return self.path.parent / f'{self.id}.meta.json' if self.legacy else self.path / 'session.json'


def _epoch(value):
    """Epoch seconds from a number or an ISO string; 0.0 when absent or unreadable."""
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return datetime.fromisoformat(str(value).replace('Z', '+00:00')).timestamp()
    except ValueError:
        return 0.0


def _size(path):
    """Bytes of every regular file under ``path``; symlinks are not followed."""
    total = 0
    for folder, _, names in os.walk(path):
        for name in names:
            try:
                total += os.lstat(os.path.join(folder, name)).st_size
            except OSError:
                pass
    return total


def _has_unfinalized(path):
    """True when a segment folder holds ``.mcap`` files but no ``metadata.yaml`` (the recorder never closed it)."""
    return any(any(name.endswith('.mcap') for name in names) and 'metadata.yaml' not in names
               for _, _, names in os.walk(path))


def _markers(path):
    return sum(1 for record in data.read_jsonl(path / 'events.jsonl') if record.get('phase') == 'marker')


def _load(folder, legacy, recording):
    """Build a Bag from one session folder; every read tolerates a missing or torn file."""
    bag = Bag(id=folder.name, path=folder, legacy=legacy, recording=folder.name in recording)
    meta = data.read_json(bag.meta_path, {})
    summary = data.read_json(folder / 'summary.json', {})
    if legacy:
        report = data.read_json(folder.parent / f'{folder.name}.report.json', {})
        results = report.get('results') or []
        runs = len(results)
        bag.outcome = f'{runs} results' if runs else ''
    else:
        runs = len(list((folder / 'runs').glob('*.json')))
        bag.outcome = str(summary.get('outcome') or (f'{runs} runs' if runs else ''))
        bag.health_fails = int(summary.get('health_fails') or 0)
    bag.title, bag.note = str(meta.get('title') or ''), str(meta.get('note') or '')
    bag.pinned, bag.sent = bool(meta.get('pinned')), bool(meta.get('sent'))
    bag.sim = bool((meta.get('profile') or {}).get('sim')) if not legacy else False
    header = {} if legacy else data.read_json(folder / 'session.json', {})
    start, end = _epoch(header.get('start')), _epoch(header.get('end'))
    bag.start = start or folder.stat().st_mtime
    bag.duration = max(0.0, end - start) if start and end else 0.0
    bag.markers = _markers(folder)
    bag.size = _size(folder) + (_size_of_siblings(folder) if legacy else 0)
    bag.unfinalized = _has_unfinalized(folder) and not bag.recording
    bag.empty = not runs and not bag.markers
    return bag


def _size_of_siblings(folder):
    return sum(f.stat().st_size for f in folder.parent.glob(f'{folder.name}.*') if f.is_file())


def scan(root=None, recording=()):
    """Every session under ``root`` (default ``data.root()``), newest first.

    ``recording`` is the set of session ids a recorder is writing right now; those are never
    reported unfinalized and never pruned or deleted.
    """
    root = Path(root) if root else data.root()
    recording = set(recording)
    found = []
    for folder in sorted((root / 'sessions').glob('*')) if (root / 'sessions').is_dir() else []:
        if folder.is_dir():
            found.append(_load(folder, False, recording))
    for folder in sorted(root.glob('*')):
        if folder.is_dir() and folder.name not in RESERVED and not folder.name.startswith('.'):
            found.append(_load(folder, True, recording))
    return sorted(found, key=lambda bag: bag.start, reverse=True)


def disk(root=None):
    """``(used, total, bags)`` bytes: the filesystem holding ``root`` and the sessions' own size."""
    root = Path(root) if root else data.root()
    probe = root
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    usage = shutil.disk_usage(probe)
    return usage.used, usage.total, _size(root) if root.is_dir() else 0


def prune_candidates(bags, budget):
    """Sessions to delete, in order: empties first, then oldest until the total fits ``budget`` bytes.

    Pinned and recording sessions are never listed, even when that leaves the total over budget.
    Empties are listed whether or not the budget is exceeded; nothing is deleted without a confirm.
    """
    eligible = sorted((b for b in bags if not b.pinned and not b.recording), key=lambda b: b.start)
    chosen = [b for b in eligible if b.empty]
    total = sum(b.size for b in bags) - sum(b.size for b in chosen)
    for bag in (b for b in eligible if not b.empty):
        if total <= budget:
            break
        chosen.append(bag)
        total -= bag.size
    return chosen


def set_meta(bag, **fields):
    """Edit title, note, pinned or sent: atomic rewrite of the meta file, and the Bag is updated too."""
    unknown = set(fields) - set(EDITABLE)
    if unknown:
        raise ValueError(f'cannot edit {sorted(unknown)}')
    record = data.read_json(bag.meta_path, {}) or {}
    record.update(fields)
    data.write_json(bag.meta_path, record, SCHEMA)
    for name, value in fields.items():
        setattr(bag, name, value)


def delete(bag):
    """Remove a session and its legacy sidecars; refused for pinned or recording ones."""
    if bag.pinned or bag.recording:
        raise PermissionError(f'{bag.id} is {"recording" if bag.recording else "pinned"}')
    shutil.rmtree(bag.path)
    if bag.legacy:
        for sidecar in bag.path.parent.glob(f'{bag.id}.*'):
            sidecar.unlink()
