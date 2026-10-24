"""The operator's Nav2 parameter file as a watched, never-auto-applied surface.

``nav2.yaml`` is the file the Navigation component is started with. An outside
save is noticed (debounced by one poll) and left *pending*: the Navigation row
says so and ``e`` previews the key-by-key diff against what was last applied.
Applying goes through an injected ``applier(changes) -> dict`` (the Nav2Apply
seam); without one the preview is read-only. No ROS and no Rich here.
"""

from collections.abc import Mapping
from copy import deepcopy
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


def same_value(left, right):
    """Compare YAML values without Python's bool/numeric equality aliasing."""
    if isinstance(left, bool) or isinstance(right, bool):
        return type(left) is type(right) and left == right
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        return (len(left) == len(right) and all(
            any(same_value(key, other) and same_value(value, right[other])
                for other in right) for key, value in left.items()))
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(map(same_value, left, right))
    return type(left) is type(right) and left == right


def diff(before, after):
    """Sorted ``(key, old, new)`` for every flattened key that differs (None = absent)."""
    old, new = flatten(before), flatten(after)
    return [(key, old.get(key), new.get(key)) for key in sorted(old.keys() | new.keys())
            if not same_value(old.get(key), new.get(key))]


def separator():
    """The UI's separator (ASCII-safe); imported late so this module stays UI-free."""
    from mhseals_testing.ui import theme
    return theme.SEPARATOR


class Nav2File:
    """Tracks ``path`` against the last applied content.

    The file is parsed once, when it settles; ``current`` is the last good parse. A
    missing or broken file keeps ``current`` (never reads as "every key removed").
    """

    def __init__(self, path, applier=None):
        self.path, self.applier = Path(path), applier
        self.error = ''
        self.current = {}
        self.read()
        self.applied = {}
        self.verified = False
        self.seen = self.settled = self.key()

    def key(self):
        try:
            stat = self.path.stat()
        except OSError:
            return None
        return stat.st_mtime_ns, stat.st_size, stat.st_ino

    def read(self):
        """Parse the file into ``current``; set ``error`` ('' when usable or missing)."""
        self.error = ''
        try:
            data = yaml.safe_load(self.path.read_text())
        except OSError:
            self.error = 'unreadable'
            return  # keep the last good content
        except yaml.YAMLError as error:
            mark = getattr(error, 'problem_mark', None)
            self.error = f'line {mark.line + 1}' if mark else 'unparsable'
            return
        if data is not None and not isinstance(data, dict):
            self.error = 'not a mapping'
            return
        self.current = data or {}

    def poll(self):
        """One tick; the toast text once a changed file has settled, else ''."""
        key, previous = self.key(), self.seen
        self.seen = key
        if key == self.settled or key != previous:
            return ''  # unchanged, or still being written
        self.settled = key
        self.read()
        if self.error:
            return f'{self.path.name} rejected ({self.error})'
        return f'{self.path.name} changed{separator()}e preview' if self.changes() else ''

    def changes(self):
        """Pending ``(key, old, new)`` to apply; keys removed from the file are not included."""
        return [c for c in diff(self.applied, self.current) if c[2] is not None]

    def removed(self):
        """Keys dropped from the file; they cannot be unset live (a restart resets them)."""
        return [key for key, _, new in diff(self.applied, self.current) if new is None]

    @property
    def pending(self):
        return not self.verified or bool(self.changes() or self.removed())

    def row(self):
        """The Navigation component row for the status list."""
        if self.error:
            return dict(group='Components', state='ERROR', name=NAME, value='error',
                        detail=f'{self.path.name}: {self.error}', required=False)
        changes = self.changes() + self.removed()
        if self.pending:
            return dict(group='Components', state='WARN', name=NAME, value='pending',
                        detail=f'{len(changes)} key(s) in {self.path.name} not applied'
                               f'{separator()}e preview', required=False)
        return dict(group='Components', state='OK', name=NAME, value='applied',
                    detail=f'{self.path.name}{separator()}e Nav2 parameters', required=False)

    def apply(self, content=None):
        """Hand ``content`` (default: current; pass the previewed parse) to the applier.

        A receipt must have status='verified' and a 'verified' mapping exactly matching
        the requested keys/values, backed by consumer evidence from the adapter.
        Accepted parameter dictionaries alone do not satisfy this contract.
        Returns the outcome text. Pending stays on failure, and a save made after the
        preview stays pending because exactly the previewed content is recorded.
        """
        content = self.current if content is None else content
        changes = [c for c in diff(self.applied, content) if c[2] is not None]
        if self.applier is None:
            return 'Nav2 apply is not wired in this session'
        if self.error:
            return f'Nav2 apply unavailable: {self.error}'
        if any(new is None for _, _, new in diff(self.applied, content)):
            return 'Nav2 removals pending: restart verification required'
        if not changes and self.verified:
            return 'Nav2 parameters already applied'
        # Once an adapter is invoked, previous evidence cannot certify current
        # runtime state if the attempt fails or only partially takes effect.
        expected = ({key: new for key, _, new in changes} if self.verified
                    else flatten(content))
        self.verified = False
        try:
            outcome = self.applier(expected) or {}
        except Exception as error:  # the applier reports its own refusals; never crash the page
            return f'Nav2 apply failed: {error}'
        if not isinstance(outcome, dict):
            return 'Nav2 apply unverified: invalid receipt'
        if outcome.get('refused') or outcome.get('blocked'):
            return f"Nav2 apply refused: {outcome.get('refused') or outcome.get('blocked')}"
        evidence = outcome.get('verified')
        if (outcome.get('status') != 'verified' or not isinstance(evidence, Mapping)
                or not same_value(evidence, expected)
                or outcome.get('error') or outcome.get('partial')):
            return 'Nav2 apply unverified: complete consumer evidence required'
        self.applied = deepcopy(content)
        self.verified = True
        return f'Nav2 applied {len(changes)} key(s)'
