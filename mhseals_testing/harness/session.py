"""Session directories under the data root. ROS-free.

``<root>/sessions/<YYYYMMDD_HHMMSS>_<host>_<profile>/`` holds ``session.json``
(the header), ``logs/`` and ``snapshots/``; later pieces add ``bag/``, runs and
the journal beside them. Every path comes from ``data``, so nothing depends on
the working directory. Legacy flat sessions (bag dirs next to ``*.report.json``)
are listed read-only by ``scan_legacy``.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import re
import socket
from pathlib import Path

from mhseals_testing import data

SCHEMA = 'mhseals.session/1'
EDITABLE = ('title', 'note', 'pinned')
_RESERVED = {'sessions', 'logs', 'config-history', 'calibration', 'sim'}


def _slug(text):
    return re.sub(r'[^A-Za-z0-9.-]+', '-', str(text)).strip('-') or 'x'


@dataclass(frozen=True)
class Session:
    id: str
    directory: Path

    @property
    def file(self):
        return self.directory / 'session.json'

    @property
    def logs(self):
        return self.directory / 'logs'

    @property
    def snapshots(self):
        return self.directory / 'snapshots'


def create(profile, host=None, clock=None, git=None, config=None, now=None):
    """Claim a fresh session directory and write its header.

    ``profile`` is a name; ``clock`` ({domain, time_scale}), ``git`` (provenance)
    and ``config`` (config-history version ids) are stored as given. The
    directory is claimed with mkdir, so concurrent starts in the same second get
    ``_02``, ``_03``... suffixes instead of sharing a name.
    """
    now = now or datetime.now().astimezone()    # the id is local time, created is UTC
    host = host or socket.gethostname()
    base = f'{now.strftime("%Y%m%d_%H%M%S")}_{_slug(host)}_{_slug(profile)}'
    sessions = data.path('sessions')
    sessions.mkdir(parents=True, exist_ok=True)
    index = 1
    while True:
        session_id = base if index == 1 else f'{base}_{index:02d}'
        try:
            (sessions / session_id).mkdir()
            break
        except FileExistsError:
            index += 1
    session = Session(session_id, sessions / session_id)
    session.logs.mkdir()
    session.snapshots.mkdir()
    data.write_json(session.file, {
        'id': session_id, 'title': '', 'note': '', 'pinned': False,
        'host': host, 'profile': profile,
        'clock': clock or {'domain': 'wall', 'time_scale': 1.0},
        'created': now.astimezone(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'git': git or {}, 'config': config or {}}, SCHEMA)
    return session


def update(session, **fields):
    """Atomically rewrite ``title``, ``note`` or ``pinned``; other keys are refused. Returns the new header."""
    unknown = set(fields) - set(EDITABLE)
    if unknown:
        raise ValueError(f'not editable: {sorted(unknown)}')
    header = data.read_json(session.file)
    if not isinstance(header, dict):
        raise FileNotFoundError(f'{session.file} is missing or unreadable')
    header.update(fields)
    data.write_json(session.file, header, SCHEMA)
    return header


_STAMP = re.compile(r'_(\d{8}_\d{6})$')


def _legacy_time(entry):
    """Trailing ``YYYYmmdd_HHMMSS`` of a legacy name (read as UTC), else the mtime."""
    match = _STAMP.search(entry.name)
    if match:
        try:
            return datetime.strptime(match.group(1), '%Y%m%d_%H%M%S').replace(tzinfo=timezone.utc).timestamp()
        except ValueError:
            pass
    return entry.stat().st_mtime


def scan_legacy():
    """Flat pre-``sessions/`` entries under the root, newest first; never writes.

    A legacy session is a top-level directory holding rosbag2 data (a
    ``metadata.yaml`` or ``bag_NN`` directory), or one that has a sibling
    ``<name>.report.json``.
    """
    root = data.root()
    found = []
    entries = sorted(root.iterdir(), key=_legacy_time, reverse=True) if root.is_dir() else []
    for entry in entries:
        if not entry.is_dir() or entry.name in _RESERVED or entry.name.startswith('.'):
            continue
        is_bag = (entry / 'metadata.yaml').exists() or any(entry.glob('bag_*'))
        if is_bag or (root / f'{entry.name}.report.json').exists():
            found.append({'name': entry.name, 'path': entry, 'legacy': True})
    return found
