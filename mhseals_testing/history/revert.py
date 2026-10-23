"""Revert a subsystem to an earlier config version: plan, then execute (config_history.md section 5).

ROS-free. The host supplies an ``Env`` (blocker probes and the mux hold) and one backend per
subsystem (read live state, apply a version, verify it). A revert is always a new change with
``revert_of``; a failed verify restores the previous version and records nothing.
"""

import contextlib
from dataclasses import dataclass, field
import fcntl
from pathlib import Path
import uuid

import yaml

from mhseals_testing import history as hist

def _settings_path():
    """config/history/revert.yaml in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parents[2] / 'config' / 'history' / 'revert.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'history' / 'revert.yaml'
    except (ImportError, KeyError):
        return source


SETTINGS = _settings_path()


def load_settings(path=None):
    return yaml.safe_load(Path(path or SETTINGS).read_text())


@dataclass
class Env:
    """Host hooks. ``probes`` maps a probe name to a callable returning True while it blocks.

    ``hold(token)`` is a context manager keeping the mux in neutral for the apply and verify;
    it releases by token, so it never clears an operator's hold.
    """
    probes: dict = field(default_factory=dict)
    armed: object = lambda: False
    hold: object = lambda token: contextlib.nullcontext()


class Backend:
    """What a subsystem must offer. ``live_keys`` is the set of top-level keys safe to change
    while armed (None means nothing is live-safe)."""
    live_keys = None

    def read(self):
        """(value, raw, meta) of the live state."""
        raise NotImplementedError

    def apply(self, version):
        """Write the stored version object to the subsystem."""
        raise NotImplementedError

    def verify(self, version, timeout):
        """True once the subsystem shows the version's value; False or TimeoutError when it does not."""
        raise NotImplementedError


@dataclass
class Plan:
    sub: str
    target: dict
    current: dict
    keys: list
    blockers: list
    hold: bool = False       # armed with live-only keys: neutral hold around the apply

    @property
    def ok(self):
        return not self.blockers and self.target['after'] != self.current['after']


@dataclass
class Outcome:
    status: str              # reverted | stale | blocked | noop | rolled_back | restore_failed
    detail: str = ''
    change: dict = None


def changed_keys(a, b):
    """Top-level keys whose values differ, over the overrides when the value has them."""
    a, b = (v.get('overrides', v) for v in (a, b))
    return sorted(k for k in set(a) | set(b) if hist._canonical(a.get(k)) != hist._canonical(b.get(k)))


def blockers(sub, env, settings, keys=(), live_keys=None):
    """Reasons a revert of ``sub`` cannot run now; empty when it can."""
    texts = {**settings['blockers']['common'], **settings['blockers'].get(sub, {})}
    probe = lambda name: env.probes.get(name) or (env.armed if name == 'armed' else lambda: False)
    found = [text for name, text in texts.items() if probe(name)()]
    if sub == 'hardware' and env.armed():
        outside = [k for k in keys if k not in (live_keys or ())]
        if outside:
            found.append('armed: pins, chips or backend change (' + ', '.join(outside) + ')')
    return found


def plan(history, backends, env, ref, sub, settings=None):
    """Preview what reverting ``sub`` to ``ref`` would change and what blocks it."""
    settings = settings or load_settings()
    target, current = history.resolve(ref, sub), history.head(sub)
    if current is None:
        raise KeyError(f'no history for {sub}')
    keys = changed_keys(history.version(current)['value'], history.version(target)['value'])
    live = backends[sub].live_keys
    found = blockers(sub, env, settings, keys, live)
    return Plan(sub, target, current, keys, found, hold=sub == 'hardware' and env.armed() and not found)


def execute(history, backends, env, p, settings=None, source='tui:revert', session=None):
    """Re-check, hold neutral if armed, apply, verify, record; roll back on a failed verify.

    A separate non-blocking ``revert.lock`` covers the whole run (``History.record`` takes its own
    ``lock`` per call, so reusing that file would deadlock).
    """
    history.dir.mkdir(parents=True, exist_ok=True)
    with open(history.dir / 'revert.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return Outcome('blocked', 'the history lock is held')
        return _execute(history, backends, env, p, settings, source, session)


def _execute(history, backends, env, p, settings, source, session):
    settings = settings or load_settings()
    backend, sub = backends[p.sub], p.sub
    fresh = plan(history, backends, env, p.target['id'], sub, settings)
    if fresh.blockers:
        return Outcome('blocked', '; '.join(fresh.blockers))
    live, raw, meta = backend.read()
    if hist.digest(live) != fresh.current['after']:
        history.observe(sub, live, raw, meta)      # the world moved: record it, preview again
        return Outcome('stale', 'live state differed from the head; review the new diff')
    if fresh.target['after'] == fresh.current['after']:
        return Outcome('noop', 'already at that version')
    target, previous = history.version(fresh.target), history.version(fresh.current)
    timeout = settings['verify_timeout_s'][sub]
    with env.hold(f'revert-{uuid.uuid4().hex[:8]}') if fresh.hold else contextlib.nullcontext():
        if _applied(backend, target, timeout):
            change = history.record(sub, target['value'], target['raw'], source=source, session=session,
                                    revert_of=fresh.target['id'], meta=target.get('meta'),
                                    summary=[f"revert to {fresh.target['id']}"])
            return Outcome('reverted', change=change)
        if _applied(backend, previous, timeout):
            return Outcome('rolled_back', 'verify failed; previous version restored')
    live, raw, meta = backend.read()
    history.observe(sub, live, raw, meta)         # record the true state
    return Outcome('restore_failed', 'verify failed and the previous version did not restore')


def _applied(backend, version, timeout):
    try:
        backend.apply(version)
        return bool(backend.verify(version, timeout))
    except (TimeoutError, OSError):
        return False
