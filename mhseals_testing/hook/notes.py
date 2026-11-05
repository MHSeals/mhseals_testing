"""Notes to the TUI: the channel a line goes to (this machine's, the sim's or a boat's over ssh), the per-session
rate cap (``State``) and secret redaction. Stdlib only."""

import fcntl
import json
import os
from pathlib import Path
import re
import time

from .hosts import SIM, remote, target_of

SCHEMA = 'mhseals.agent/1'
CMD_MAX = 300
_VALUE = r'''("[^"]*"|'[^']*'|[^\s'"]+)'''
SECRETS = (   # agent_channel._SECRETS, kept equal by test_agent_hook
    re.compile(r'(?i)(\b(?:token|pass(?:wd|word)?|secret|key)\s*=\s*)' + _VALUE),
    re.compile(r'(?i)(--(?:token|pass(?:wd|word)?|secret|api-?key|key)(?:\s+|=))' + _VALUE),
    re.compile(r'(\bsshpass\b(?:\s+-[^p\s]\S*)*\s+-p\s*)' + _VALUE),
    re.compile(r'(?i)(\bauthorization\s*:\s*(?:(?:bearer|basic|token)\s+)?)' + _VALUE),
    re.compile(r'(?i)(\bbearer\s+)' + _VALUE))


WINDOW_S = 10.0


def private_dir(directory=None):
    """``directory``, else ``$XDG_RUNTIME_DIR/boat_agent_hook``, else ``~/.cache/boat_agent_hook``:
    user-only (0700), never a shared /tmp another user could plant a state file in."""
    if directory:
        return Path(directory)
    base = os.environ.get('XDG_RUNTIME_DIR') or Path.home() / '.cache'
    out = Path(base) / 'boat_agent_hook'
    out.mkdir(mode=0o700, parents=True, exist_ok=True)
    return out


def sim_channel():
    """The sim TUI's channel on this machine: config/sim/stack.yaml ``overlay`` (``$BOAT_SIM_OVERLAY`` wins), mounted
    as the container's /home/roboboat/autodemo, so its data/agent/channel.jsonl; read without PyYAML."""
    overlay = os.environ.get('BOAT_SIM_OVERLAY')
    if not overlay:
        try:
            text = (Path(__file__).resolve().parents[2] / 'config' / 'sim' / 'stack.yaml').read_text()
            overlay = re.search(r'^overlay:\s*(\S+)', text, re.M).group(1)
        except (OSError, AttributeError):
            overlay = '~/.cache/boat-sim/ws'
    return Path(overlay).expanduser() / 'data' / 'agent' / 'channel.jsonl'


def local_channel():
    """This machine's channel: data.path('agent', 'channel.jsonl') (``$MHSEALS_DATA``, else
    ``$ROS_WS/bags``, else ``~/astro_dock/bags``), without importing the package."""
    data = os.environ.get('MHSEALS_DATA')
    if data and os.path.isabs(data):
        return Path(data).resolve() / 'agent' / 'channel.jsonl'
    ws = os.environ.get('ROS_WS')
    ws = Path(ws).resolve() if ws and os.path.isabs(ws) else Path.home() / 'astro_dock'
    return ws / 'bags' / 'agent' / 'channel.jsonl'


def redact(cmd):
    """``agent_channel.redact``: secret values hidden, cut to CMD_MAX chars."""
    for pattern in SECRETS:
        cmd = pattern.sub(r'\1…', cmd)
    return cmd[:CMD_MAX]


class State:
    """Per-session send times (a sliding window) and pre-call start times, in a small temp file.

    A context manager: the file stays locked (flock) from read to write, so parallel tool calls
    of one session never lose a send or a start time.
    """

    def __init__(self, session, directory=None):
        try:
            directory = private_dir(directory)
        except OSError:
            directory = Path(os.environ.get('TMPDIR') or '/tmp')
        self.path = directory / f'boat_agent_hook-{re.sub(r"[^A-Za-z0-9-]", "", session)[:64]}.json'
        self.handle, self.data = None, {}

    def __enter__(self):
        try:
            self.handle = open(self.path, 'a+')
            fcntl.flock(self.handle, fcntl.LOCK_EX)
            self.handle.seek(0)
            self.data = json.loads(self.handle.read() or '{}')
        except (OSError, ValueError):
            self.data = {}
        if not isinstance(self.data.get('sent'), list):
            self.data['sent'] = []
        self.data.setdefault('folded', 0)
        self.data.setdefault('started', {})
        return self

    def __exit__(self, *_):
        if self.handle is None:
            return
        try:
            self.handle.seek(0)
            self.handle.truncate()
            self.handle.write(json.dumps(self.data))
        except OSError:
            pass
        finally:
            self.handle.close()     # releases the lock

    def admit(self, now, limit):
        """(send?, folded count to report): at most ``limit`` sends in any WINDOW_S."""
        self.data['sent'] = [t for t in self.data['sent'] if now - t < WINDOW_S]
        if len(self.data['sent']) >= limit:
            self.data['folded'] += 1
            return False, 0
        self.data['sent'].append(now)
        folded, self.data['folded'] = self.data['folded'], 0
        return True, folded


def log(text):
    base = Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local' / 'state')
    try:
        base.mkdir(parents=True, exist_ok=True)
        with open(base / 'boat_agent_hook.log', 'a') as handle:
            handle.write(f'{time.strftime("%Y-%m-%dT%H:%M:%S")} {text}\n')
    except OSError:
        pass


def deliver(line, config):
    """Append one line to the target's channel: here unless a boat session names another host (then a
    detached ssh); never waits."""
    data = (json.dumps(line, separators=(',', ':')) + '\n').encode()
    target, host = target_of(config), remote(config)
    path = config.get('path') or ''         # agent.yaml hook.path, ~ the target's home
    try:
        if host is None:                    # local, the sim, or this is the target
            path = {'local': local_channel, SIM: sim_channel}.get(target, lambda: os.path.expanduser(path))()
            if target in ('local', SIM) and not Path(path).parent.is_dir():
                return              # no TUI has run here: nobody reads the channel, nothing to log

            fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o666)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
            return
        if not path:
            return log(f'deliver to {target}: no hook.path (agent.yaml unread)')
        import shlex, subprocess      # only for a remote target: keeps the hook's import cost low
        quoted = '~/' + shlex.quote(path[2:]) if path.startswith('~/') else shlex.quote(path)
        child = subprocess.Popen(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=2', host,
             f'cat >> {quoted}'], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        child.stdin.write(data)
        child.stdin.close()
    except OSError as error:
        log(f'deliver to {target}: {error!r}')
