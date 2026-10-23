"""Hot-reload classes for the watched surfaces (authority_hotreload.md section 7.2). ROS-free.

``Reload.apply`` compares a surface's old and new parsed YAML, sorts every changed key into
its class from ``config/reload.yaml`` and keeps the ones that cannot apply yet pending:
plant fields while the boat is armed, child restarts until the operator asks. The caller
applies the ``live`` and ``next`` keys itself and shows ``Reload.note(surface)`` on the row.
"""

from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from mhseals_testing.profiles import PROFILE_DIR

RULES = PROFILE_DIR.parent / 'reload.yaml'
LIVE, NEXT, DISARMED, CHILD = 'live', 'next', 'disarmed', 'child'


def load_rules(path=RULES):
    with open(path) as handle:
        return yaml.safe_load(handle)


def leaves(data, prefix=''):
    """Flatten nested mappings into {'a.b': value}; lists and scalars are leaves."""
    if not isinstance(data, dict):
        return {prefix: data} if prefix else {}
    out = {}
    for key, value in data.items():
        out.update(leaves(value, f'{prefix}.{key}' if prefix else str(key)))
    return out


def classify(rules, surface, before, after):
    """{class: [changed keys]} for ``before`` -> ``after`` (parsed YAML) on ``surface``."""
    spec = rules[surface]
    prefixes = sorted(((p, name) for name in (LIVE, NEXT, DISARMED, CHILD)
                       for p in spec.get(name, ())), key=lambda item: -len(item[0]))
    old, new = leaves(before), leaves(after)
    out = {}
    for key in sorted(old.keys() | new.keys()):
        if old.get(key) == new.get(key) and (key in old) == (key in new):
            continue
        name = next((n for p, n in prefixes if key == p or key.startswith(p + '.')), spec['default'])
        out.setdefault(name, []).append(key)
    return out


@dataclass
class Reload:
    rules: dict
    pending: dict = field(default_factory=dict)  # surface -> {class: [keys]}

    def apply(self, surface, before, after, armed=False):
        """Return the keys to apply now: {'live': [...], 'next': [...]}; the rest stay pending."""
        changes = classify(self.rules, surface, before, after)
        held = self.pending.setdefault(surface, {})
        if not armed and DISARMED in held:  # the boat is disarmed now: release what waited
            changes[DISARMED] = sorted(set(changes.get(DISARMED, ())) | set(held.pop(DISARMED)))
        for name in (CHILD,) + ((DISARMED,) if armed else ()):
            if name in changes:  # merge, so a later edit never drops keys already waiting
                held[name] = sorted(set(held.get(name, ())) | set(changes.pop(name)))
        if not held:
            del self.pending[surface]
        return changes

    def disarmed(self, surface):
        """Plant keys held for ``surface``, released once the boat is disarmed (else None)."""
        held = self.pending.get(surface, {})
        keys = held.pop(DISARMED, None)
        if not held:
            self.pending.pop(surface, None)
        return keys

    def settle(self, surface, name):
        """The operator restarted what the ``name`` keys needed; drop them from pending."""
        held = self.pending.get(surface, {})
        keys = held.pop(name, None)
        if not held:
            self.pending.pop(surface, None)
        return keys

    def note(self, surface):
        """Row text for what is waiting on ``surface``, or ''."""
        held = self.pending.get(surface, {})
        if DISARMED in held:
            return f'↻ {len(held[DISARMED])} fields pending (disarm)'
        if CHILD in held:
            return '↻ base restart needed' if surface == 'rtk' else '↻ restart needed'
        return ''


def copies(source, share):
    """install.symlinked fact: the files under ``share`` that are copies, not links.

    Empty means a hot reload of ``source`` reaches what runs; any entry names a file that a
    source edit would not reach. None when nothing is installed. ``source`` is where links
    must point.
    """
    share, source = Path(share), Path(source).resolve()
    if not share.is_dir():
        return None
    return [str(path.relative_to(share)) for path in sorted(share.rglob('*'))
            if path.is_file() and (not path.is_symlink() or source not in path.resolve().parents)]


def topics_changed(old, new):
    """True when the recorder's topic set differs (order is irrelevant)."""
    return set(old) != set(new)


class Segments:
    """Recording row: topic profile changes wait for the next segment, Enter starts one now."""

    def __init__(self, restart, emit):
        self.restart, self.emit, self.topics, self.waiting = restart, emit, (), None

    def recording(self, topics):
        """The topics the live segment was started with."""
        self.topics, self.waiting = tuple(topics), None

    def changed(self, topics):
        """The profile changed: hold it for the next segment (None clears)."""
        self.waiting = tuple(topics) if topics_changed(self.topics, topics) else None

    @property
    def note(self):
        return '↻ topics changed' if self.waiting is not None else ''

    def enter(self):
        """Start a new segment now with the waiting topics (about a 1 s gap)."""
        if self.waiting is None:
            return False
        old, new = self.topics, self.waiting
        self.restart(new)
        self.recording(new)
        self.emit('segment_restart', added=sorted(set(new) - set(old)),
                  removed=sorted(set(old) - set(new)))
        return True


def rtk_restart(reload, base, config, positions, emit):
    """Enter on the RTK row: restart the base on the new ``config``, without a re-survey when
    a position is known (the one in use, else the first saved)."""
    if not reload.pending.get('rtk', {}).get(CHILD):
        return False
    position = base.position or next(iter(positions.values()), None)
    base.close()
    base.config = deepcopy(config)
    base.start(survey=position is None, position=position)
    emit('rtk_restart', keys=reload.settle('rtk', CHILD), survey=position is None)
    return True
