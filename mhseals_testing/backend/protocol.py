"""The backend's wire protocol: newline-delimited JSON objects over a local stream socket. ROS-free.

A client opens with ``hello`` (its protocol version and who it is). The backend answers ``welcome``
with the full ``snapshot`` and its sequence number, or ``refuse`` with why, and closes. After that the
backend pushes ``delta`` frames (each with the next ``seq``), and the client sends ``request`` frames
(``id``, ``op``, ``args``) that get one ``reply`` each (``id`` and ``data`` or ``error``). Clients
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

VERSION = (1, 0)
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
    socket: str
    max_frame: int


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
    first, longest = values['reconnect']
    numbers = [values['attach_timeout'], values['heartbeat'], values['link_timeout'], first, longest,
               values['max_frame']]
    if not all(isinstance(n, (int, float)) and not isinstance(n, bool) and n > 0 for n in numbers):
        raise ValueError(f'{Path(path).name}: timeouts, reconnect waits and max_frame must be positive numbers')
    if values['link_timeout'] <= values['heartbeat'] or longest < first:
        raise ValueError(f'{Path(path).name}: link_timeout must exceed heartbeat, and reconnect be [first, longest]')
    return Settings(float(values['attach_timeout']), float(values['heartbeat']), float(values['link_timeout']),
                    (float(first), float(longest)), str(values['socket']), int(values['max_frame']))


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
