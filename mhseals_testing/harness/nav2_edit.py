"""The operator's Nav2 parameter file as a watched, never-auto-applied surface.

``nav2.yaml`` is the file the Navigation component is started with. An outside
save is noticed (debounced by one poll) and left *pending*: the Navigation row
says so and ``e`` previews the key-by-key diff against what was last applied.
Applying goes through an injected ``applier(changes) -> dict`` (the Nav2Apply
seam); without one the preview is read-only. No ROS and no Rich here.
"""

from pathlib import Path

import yaml

NAME = 'Navigation'


def flatten(node, prefix=''):
    """``{'a': {'b': 1}}`` as ``{'a.b': 1}``; lists stay values."""
    if not isinstance(node, dict):
        return {prefix: node} if prefix else {}
    flat = {}
    for key, value in node.items():
        flat.update(flatten(value, f'{prefix}.{key}' if prefix else str(key)))
    return flat


def diff(before, after):
    """Sorted ``(key, old, new)`` for every flattened key that differs (None = absent)."""
    old, new = flatten(before), flatten(after)
    return [(key, old.get(key), new.get(key)) for key in sorted(old.keys() | new.keys())
            if old.get(key) != new.get(key)]


class Nav2File:
    """Tracks ``path`` against the last applied content."""

    def __init__(self, path, applier=None):
        self.path, self.applier = Path(path), applier
        self.applied = self.load() or {}
        self.seen = self.key()
        self.settled = self.seen
        self.error = ''

    def key(self):
        try:
            stat = self.path.stat()
        except OSError:
            return None
        return stat.st_mtime_ns, stat.st_size, stat.st_ino

    def load(self):
        try:
            data = yaml.safe_load(self.path.read_text())
        except (OSError, yaml.YAMLError):
            return None
        return data if isinstance(data, dict) else {}

    def poll(self):
        """One tick; the toast text once a changed file has settled, else ''."""
        key, previous = self.key(), self.seen
        self.seen = key
        if key == self.settled or key != previous:
            return ''  # unchanged, or still being written
        self.settled = key
        self.error = self.problem()
        if self.error:
            return f'{self.path.name} rejected ({self.error})'
        return f'{self.path.name} changed · e preview' if self.changes() else ''

    def problem(self):
        """Why the file cannot be used ('' when it parses to a mapping)."""
        try:
            data = yaml.safe_load(self.path.read_text())
        except OSError:
            return ''
        except yaml.YAMLError as error:
            mark = getattr(error, 'problem_mark', None)
            return f'line {mark.line + 1}' if mark else 'unparsable'
        return '' if data is None or isinstance(data, dict) else 'not a mapping'

    def changes(self):
        """Pending ``(key, old, new)``; [] for a missing or broken file (the last good one stays)."""
        if self.problem():
            return []
        return diff(self.applied, self.load() or {})

    @property
    def pending(self):
        return bool(self.changes())

    def row(self):
        """The Navigation component row for the status list."""
        if self.error:
            return dict(group='Components', state='ERROR', name=NAME, value='error',
                        detail=f'{self.path.name}: {self.error}', required=False)
        changes = self.changes()
        if changes:
            return dict(group='Components', state='WARN', name=NAME, value='pending',
                        detail=f'{len(changes)} key(s) in {self.path.name} not applied · e preview',
                        required=False)
        return dict(group='Components', state='OK', name=NAME, value='applied',
                    detail=f'{self.path.name} · e Nav2 parameters', required=False)

    def apply(self):
        """Hand the pending changes to the applier; the outcome text. Pending stays on failure."""
        changes = self.changes()
        if self.applier is None:
            return 'Nav2 apply is not wired in this session'
        if not changes:
            return 'Nav2 parameters already applied'
        try:
            outcome = self.applier({key: new for key, _, new in changes}) or {}
        except Exception as error:  # the applier reports its own refusals; never crash the page
            return f'Nav2 apply failed: {error}'
        if outcome.get('refused') or outcome.get('blocked'):
            return f"Nav2 apply refused: {outcome.get('refused') or outcome.get('blocked')}"
        self.applied = self.load() or {}
        return f'Nav2 applied {len(changes)} key(s)'
