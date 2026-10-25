"""Claude Code hook: tell the TUI what an agent runs on the boat, and stop raw motion commands.

Stdlib only (PyYAML when importable), so it runs as ``python3 -I agent_hook.py start|pre|post``
from the checkout on the dev machine; the hook JSON arrives on stdin (spec agent-control section 6).

* ``start`` prints the agent's boat name and how to drive (``boat_agent`` verbs only).
* ``pre`` denies a command matching ``hook.deny`` (raw /cmd_vel, /goal_pose, mux, MAVROS
  arming/command/mode/params, Nav2 goals and behaviors, pkill; ``.`` spans lines) and names the
  verb to use instead. A boat-bound command (``hook.match``, not
  ``hook.skip``) becomes a channel ``note`` line ``{phase: pre, tool_id, cmd}``.
* ``post`` (PostToolUse and PostToolUseFailure) sends the same tool_id with ``exit`` and ``dur_s``:
  a success is exit 0; a failure's exit is the ``Exit code N`` first line of ``error``.

Lines go to ``hook.path`` on ``hook.target`` over a detached ``ssh ... cat >>`` (appended directly
when this is the target), at most ``hook.max_per_10s`` per session; the excess folds into the next
line as ``folded``. Delivery failures go to ``$XDG_STATE_HOME/boat_agent_hook.log`` and never reach
the agent. It exits 0 always: a deny is a JSON decision, not an error. Advisory, not a boundary:
the mux, the claim and the red external chip are the safety.
"""

import fcntl
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import time

SCHEMA = 'mhseals.agent/1'
CMD_MAX = 300
_VALUE = r'''("[^"]*"|'[^']*'|[^\s'"]+)'''
SECRETS = (   # agent_channel._SECRETS, kept equal by test_agent_hook
    re.compile(r'(?i)(\b(?:token|pass(?:wd|word)?|secret|key)\s*=\s*)' + _VALUE),
    re.compile(r'(?i)(--(?:token|pass(?:wd|word)?|secret|api-?key|key)(?:\s+|=))' + _VALUE),
    re.compile(r'(\bsshpass\b(?:\s+-[^p\s]\S*)*\s+-p\s*)' + _VALUE),
    re.compile(r'(?i)(\bauthorization\s*:\s*(?:(?:bearer|basic|token)\s+)?)' + _VALUE),
    re.compile(r'(?i)(\bbearer\s+)' + _VALUE))
CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'agent.yaml'
DEFAULTS = {
    'target': 'blastoise', 'path': '/var/home/roboboat/astro_dock/bags/agent/channel.jsonl',
    'match': [r'\bssh\s+(-\S+\s+)*(blastoise|squirtle)\b', r'\bros2\b', r'\bdocker\s+exec\b'],
    'skip': [r'\bboat_(trial|agent)\b'], 'max_per_10s': 6,
    'deny': [r'ros2\s+topic\s+pub\b.*(/cmd_vel|/command_mux|/goal_pose|/mavros/(rc/override|setpoint))',
             r'ros2\s+action\s+send_goal\b.*(navigate|/(spin|backup|drive_on_heading|follow_waypoints)\b)',
             r'ros2\s+service\s+call\b.*/mavros/(cmd/(arming|command)|set_mode|param)',
             r'ros2\s+param\s+set\s+\S*mavros', r'\b(pkill|killall)\b']}
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
EXIT_LINE = re.compile(r'Exit code (-?\d+)')
DENY_REASON = ('raw motion/FCU command bypasses the claim; use boat_agent run|goal|stop '
               '(ssh blastoise ' + BOAT_AGENT + ' claim --who {who} first); '
               'arming and hold release stay with the operator')


def hook_config(path=CONFIG):
    """``hook:`` of config/agent.yaml, else the built-in defaults."""
    try:
        import yaml
        found = (yaml.safe_load(Path(path).read_text()) or {}).get('hook') or {}
    except Exception:          # no PyYAML under -I, or no file: the defaults are the shipped values
        found = {}
    return {**DEFAULTS, **{k: v for k, v in found.items() if k in DEFAULTS}}


def who_of(event):
    """``$BOAT_AGENT``, else claude- and the first 4 hex digits of the session id."""
    explicit = os.environ.get('BOAT_AGENT')
    if explicit:
        return explicit
    digits = re.sub(r'[^0-9a-f]', '', str(event.get('session_id', '')).lower())
    return f'claude-{digits[:4] or "0000"}'


def start_text(who):
    return (f'Your boat agent name is {who}. Act on the boat only through '
            f'`ssh blastoise {BOAT_AGENT} <verb> --who {who}` (JSON out): `status` first, then '
            f'`claim --reason ...`, `run KEY|goal X Y --claim ID --dry-run`, the real run, `follow --until ...`, '
            f'`release --claim ID`; `stop` any time; `ask "..."` instead of guessing. Exit codes: 0 ok, '
            f'2 refused (read hint), 3 revoked/overridden (stop; a human took over; claim again only if '
            f'asked), 4 no TUI, 5 agents blocked, 6 timeout. Never publish /cmd_vel, mux or MAVROS '
            f'commands, and never arm or release a hold. Run in sim first; use run_in_background for runs '
            f'over 9 min (the CLI keeps the lease).')


def matches(patterns, text):
    """Any pattern found; ``.`` spans newlines, so a multi-line command cannot hide a match."""
    return any(re.search(p, text, re.S) for p in patterns)


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
        directory = Path(directory or tempfile.gettempdir())
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
    """Append one line on the target: locally when this is it, else a detached ssh; never waits."""
    data = (json.dumps(line, separators=(',', ':')) + '\n').encode()
    try:
        if socket.gethostname().split('.')[0] == config['target']:
            fd = os.open(config['path'], os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o666)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
            return
        child = subprocess.Popen(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=2', config['target'],
             f"cat >> '{config['path']}'"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        child.stdin.write(data)
        child.stdin.close()
    except OSError as error:
        log(f'deliver to {config["target"]}: {error!r}')


def handle(phase, event, config=None, send=None, now=None, state_dir=None):
    """The hook's stdout text (may be empty) for one event; sends at most one note line."""
    config = config or hook_config()
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    who = who_of(event)
    if phase == 'start':
        return json.dumps({'hookSpecificOutput': {'hookEventName': 'SessionStart',
                                                  'additionalContext': start_text(who)}})
    command = str((event.get('tool_input') or {}).get('command') or '')
    if phase == 'pre' and matches(config['deny'], command):
        return json.dumps({'hookSpecificOutput': {
            'hookEventName': 'PreToolUse', 'permissionDecision': 'deny',
            'permissionDecisionReason': DENY_REASON.format(who=who)}})
    if not matches(config['match'], command) or matches(config['skip'], command):
        return ''
    tool_id = event.get('tool_use_id')
    note = {'phase': phase, 'tool_id': tool_id, 'cmd': redact(command),
            'exit': None, 'dur_s': None, 'target': config['target']}
    with State(str(event.get('session_id', 'none')), state_dir) as state:
        if phase == 'pre':
            state.data['started'] = {**dict(list(state.data['started'].items())[-31:]), str(tool_id): now}
        else:
            failed = EXIT_LINE.match(str(event.get('error') or ''))
            if event.get('hook_event_name') == 'PostToolUseFailure' or 'error' in event:
                note['exit'] = int(failed[1]) if failed else None    # None: interrupted or never started
            else:
                note['exit'] = 0       # PostToolUse fires only for a call that succeeded
            began = state.data['started'].pop(str(tool_id), None)
            took = event.get('duration_ms')
            note['dur_s'] = (round(took / 1000, 2) if isinstance(took, (int, float))
                             else None if began is None else round(now - began, 2))
        admitted, folded = state.admit(now, config['max_per_10s'])
    if admitted:
        line = {'schema': SCHEMA, 't_ns': int(now * 1e9), 'host': socket.gethostname(), 'pid': os.getpid(),
                'who': who, 'kind': 'note', **note, **({'folded': folded} if folded else {})}
        send(line)
    return ''


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    phase = args[0] if args else ''
    try:
        event = json.loads(sys.stdin.read() or '{}')
        out = handle(phase, event if isinstance(event, dict) else {}) if phase in ('start', 'pre', 'post') else ''
    except Exception as error:          # the hook never breaks the agent's tool call
        log(f'{phase}: {error!r}')
        out = ''
    if out:
        print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
