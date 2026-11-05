"""The backend's wire protocol: newline-delimited JSON objects over a local stream socket. ROS-free.

A client opens with ``hello`` (its protocol version and who it is). The backend answers ``welcome``
with the full ``snapshot``, its ``seq`` and clock ``t``, or ``refuse`` with why, and closes. After that the
backend pushes ``delta {seq, base, t, set, del}`` frames: a merge patch over the snapshot by dotted path
(``set`` maps a path to its new value, ``del`` lists removed paths). ``base`` is the ``seq`` of the
client's previous frame; any other base is a gap and the client resyncs from a fresh ``welcome``. Paths
change in rate groups: the ``control`` group goes out on every change, never coalesced; every other group
at most at its configured rate, with only its newest values (latest wins). The client sends ``request``
frames (``id``, ``op``, ``args``) that get one ``reply`` each (``id`` and ``data`` or ``error``). Clients
``ping`` every heartbeat and the backend answers ``pong``; a reconnecting client starts over from a
fresh ``welcome``, so a lost link never leaves it with a partial state.

Versions are ``[major, minor]``: majors must match; a newer minor only adds optional fields, which
readers ignore.
"""

from dataclasses import dataclass
import json
from pathlib import Path

from mhseals_testing.profiles import PROFILE_DIR
from mhseals_testing.strict_yaml import load_strict

VERSION = (2, 0)   # 2.0: path deltas in rate groups (1.x pushed whole data blobs)
CONTROL = 'control'   # the rate group sent on every change, never coalesced
KINDS = frozenset({'hello', 'welcome', 'refuse', 'delta', 'request', 'reply', 'ping', 'pong'})
CONFIG = PROFILE_DIR.parent / 'backend.yaml'


class FrameError(ValueError):
    """A frame that is not one JSON object of a known kind, or is too long."""


@dataclass(frozen=True)
class Settings:
    attach_timeout: float
    heartbeat: float
    link_timeout: float
    reconnect: tuple
    rates: dict           # group -> Hz
    slow_client: float
    socket: str
    max_frame: int
    outbox: int
    requests: int
    priority: frozenset   # ops answered ahead of the request queue

    def period(self, group):
        """Seconds between a group's sends to one client: 0 for control, else 1 / rate; KeyError when unknown."""
        return 0.0 if group == CONTROL else 1.0 / self.rates[group]


def load_settings(path=CONFIG):
    """The link settings of ``config/backend.yaml``; a missing, extra or non-positive value is an error."""
    data = load_strict(Path(path).read_text(), path=Path(path).name)
    tune, internal = data.get('tune') or {}, data.get('internal') or {}
    if set(data) != {'tune', 'internal'}:
        raise ValueError(f'{Path(path).name}: wants exactly tune and internal, got {sorted(data)}')
    values = {**tune, **internal}
    expected = set(Settings.__dataclass_fields__)
    if set(values) != expected:
        raise ValueError(f'{Path(path).name}: missing {sorted(expected - set(values))}, '
                         f'unknown {sorted(set(values) - expected)}')
    name, first, longest = Path(path).name, *values['reconnect']
    rates = values['rates']
    numbers = [values['attach_timeout'], values['heartbeat'], values['link_timeout'], first, longest,
               values['slow_client'], values['max_frame'], values['outbox'], values['requests']]
    if not isinstance(rates, dict) or not rates or CONTROL in rates:
        raise ValueError(f'{name}: rates maps each group but {CONTROL} (always on change) to its Hz')
    if not all(_positive(n) for n in numbers + list(rates.values())):
        raise ValueError(f'{name}: timeouts, waits, rates and sizes must be positive numbers')
    if values['link_timeout'] <= values['heartbeat'] or longest < first:
        raise ValueError(f'{name}: link_timeout must exceed heartbeat, and reconnect be [first, longest]')
    priority = values['priority']
    if not isinstance(priority, list) or not all(isinstance(op, str) and op for op in priority):
        raise ValueError(f'{name}: priority is a list of op names')
    return Settings(float(values['attach_timeout']), float(values['heartbeat']), float(values['link_timeout']),
                    (float(first), float(longest)), {str(g): float(hz) for g, hz in rates.items()},
                    float(values['slow_client']), str(values['socket']), int(values['max_frame']),
                    int(values['outbox']), int(values['requests']), frozenset(priority))


def _positive(n):
    return isinstance(n, (int, float)) and not isinstance(n, bool) and n > 0


def encode(frame, max_frame):
    """One frame as bytes, newline-terminated."""
    if not isinstance(frame, dict) or frame.get('kind') not in KINDS:
        raise FrameError(f'not a frame: {frame!r:.80}')
    data = json.dumps(frame, separators=(',', ':'), allow_nan=False).encode() + b'\n'
    if len(data) > max_frame:
        raise FrameError(f'{frame["kind"]} frame of {len(data)} bytes exceeds {max_frame}')
    return data


def decode(line):
    """The frame of one line (bytes, newline optional)."""
    try:
        frame = json.loads(line)
    except ValueError as error:
        raise FrameError(f'not JSON: {error}') from None
    if not isinstance(frame, dict) or frame.get('kind') not in KINDS:
        raise FrameError(f'unknown frame: {str(frame)[:80]}')
    return frame


def hello(who, version=VERSION):
    return {'kind': 'hello', 'version': list(version), 'who': who}


def check_hello(frame, version=VERSION):
    """None for an acceptable ``hello``, else why it is refused."""
    if frame.get('kind') != 'hello':
        return f'expected hello first, got {frame.get("kind")}'
    theirs = frame.get('version')
    if not (isinstance(theirs, list) and len(theirs) == 2 and all(isinstance(v, int) for v in theirs)):
        return 'hello without a [major, minor] version'
    if theirs[0] != version[0]:
        return f'protocol {theirs[0]}.{theirs[1]} is incompatible with the backend\'s {version[0]}.{version[1]}'
    if not isinstance(frame.get('who'), str) or not frame['who']:
        return 'hello without who'
    return None


def keys(path):
    """A dotted path's keys (``panels.gnss.h_acc``); keys never hold a dot."""
    return path.split('.')


def lookup(model, path):
    """(True, value) at ``path`` in the nested ``model``, else (False, None)."""
    node = model
    for key in keys(path):
        if not isinstance(node, dict) or key not in node:
            return False, None
        node = node[key]
    return True, node


def apply(model, changes=None, removed=()):
    """Merge one patch into ``model`` in place: removals first, then each set path (making parent mappings)."""
    for path in removed:
        *parents, last = keys(path)
        node = model
        for key in parents:
            node = node.get(key) if isinstance(node, dict) else None
        if isinstance(node, dict):
            node.pop(last, None)
    for path, value in (changes or {}).items():
        *parents, last = keys(path)
        node = model
        for key in parents:
            if not isinstance(node.get(key), dict):
                node[key] = {}
            node = node[key]
        node[last] = value
    return model
