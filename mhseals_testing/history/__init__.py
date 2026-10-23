"""Versioned config history: every config change is a line in one append-only log.

Objects (snapshots of a subsystem's value) are content-addressed and written once;
``changes.jsonl`` says which object replaced which. See config_history.md. No ROS here.
"""

import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import socket

from mhseals_testing import data

CHANGE = 'mhseals.config_change/1'
LABEL = 'mhseals.config_label/1'
VERSION = 'mhseals.config_version/1'


def _canonical(value):
    """Stable JSON: sorted keys, floats to 9 significant digits, so equal content hashes equal."""
    def clean(node):
        if isinstance(node, float):
            return float(f'{node:.9g}')
        if isinstance(node, dict):
            return {str(k): clean(v) for k, v in node.items()}
        if isinstance(node, (list, tuple)):
            return [clean(v) for v in node]
        return node
    return json.dumps(clean(value), sort_keys=True, separators=(',', ':'))


def digest(value):
    """The 12 hex characters naming a version object."""
    return hashlib.sha256(_canonical(value).encode()).hexdigest()[:12]


class History:
    """Reads and appends the change log under ``<data root>/config-history``."""

    def __init__(self, root=None):
        self.dir = Path(root) if root else data.path('config-history')
        self.log = self.dir / 'changes.jsonl'

    def _object(self, sub, sha):
        return self.dir / 'versions' / sub / f'{sha}.json'

    def _lines(self):
        return data.read_jsonl(self.log)

    def changes(self, sub=None):
        """Change records, oldest first, with the newest label's name and note folded in."""
        changes, labels = [], {}
        for line in self._lines():
            if line.get('schema') == LABEL:
                labels.setdefault(line['id'], {}).update({k: line[k] for k in ('name', 'note') if line.get(k) is not None})
            elif line.get('schema') == CHANGE:
                changes.append(line)
        folded = [{**c, **labels.get(c['id'], {})} for c in changes]
        return [c for c in folded if sub in (None, c['subsystem'])]

    def head(self, sub):
        """The newest change of ``sub``, or None."""
        found = self.changes(sub)
        return found[-1] if found else None

    def version(self, change):
        """The stored object a change produced."""
        return data.read_json(self._object(change['subsystem'], change['after']))

    def record(self, sub, value, raw=None, *, source, session=None, revert_of=None, applied='confirmed', meta=None,
               summary=()):
        """Append a change for ``sub``; None when ``value`` equals the head.

        Callers ``observe`` the current state first so an outside edit is not merged into this change.
        """
        sha = digest(value)
        head = self.head(sub)
        if head and head['after'] == sha:
            return None
        obj = self._object(sub, sha)
        if not obj.exists():
            data.write_json(obj, {'subsystem': sub, 'sha': sha, 'value': value, 'raw': raw, 'meta': meta or {}}, VERSION)
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.dir / 'lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)         # the id counter and the append are one step
            head = self.head(sub)
            if head and head['after'] == sha:
                return None                          # another writer recorded the same content first
            last = max((int(c['id'][1:]) for c in self.changes()), default=0)
            entry = {'id': f'c{last + 1:04d}', 'time': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
                     'host': socket.gethostname(), 'actor': f'{source.split(":")[0]}:{os.getpid()}', 'subsystem': sub,
                     'before': head['after'] if head else None, 'after': sha, 'source': source, 'session': session,
                     'revert_of': revert_of, 'applied': applied, 'name': '', 'note': '', 'summary': list(summary)}
            data.append_jsonl(self.log, entry, CHANGE)
        return {'schema': CHANGE, **entry}

    def observe(self, sub, value, raw=None, meta=None):
        """Record a value found on disk or on the FCU that no in-process writer announced."""
        return self.record(sub, value, raw, source='external', applied='observed', meta=meta)

    def label(self, ref, *, name=None, note=None):
        """Name or annotate a change; appended, so earlier lines are never rewritten."""
        change = self.resolve(ref)
        data.append_jsonl(self.log, {'id': change['id'], 'time': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
                                     'name': name, 'note': note}, LABEL)

    def resolve(self, ref, sub=None):
        """A change from ``c0142``, a name (newest wins), ``current`` or ``previous``."""
        found = self.changes(sub)
        if ref in ('current', 'previous'):
            index = -1 if ref == 'current' else -2
            if len(found) >= -index:
                return found[index]
        else:
            for change in reversed(found):
                if ref in (change['id'], change.get('name')):
                    return change
        raise KeyError(f'no config change matches {ref!r}')
