"""The agent hook (spec agent-control section 6): playbook, raw-motion denies, notes to the TUI.

astro_dock's ``.agents/hook.py`` loads this file (``src/<repo>/<package>/agent_hook.py``) and calls
``agent_event`` for Claude Code and Codex alike with one neutral event (``agent``, ``event``
start|pre|post, ``session_id``, ``tool_id``, ``command``, ``exit``, ``duration_ms``, ``failed``).
Imported under ``python3 -I`` on the dev machine: no ROS. PyYAML, when present, reads
config/agent.yaml (about 15 ms per call); without it the built-in defaults apply.

* ``start`` returns the playbook: the agent's boat name and the ``boat_agent`` verbs.
* ``pre`` denies a command matching ``hook.deny`` (raw /cmd_vel, /goal_pose, mux, MAVROS
  arming/command/mode/params, Nav2 goals and behaviors, pkill; ``.`` spans lines) with the verb
  to use instead, and notes it as ``phase: deny``. A boat-bound command (``hook.match``, not
  ``hook.skip``) becomes a channel ``note`` line ``{phase: pre, tool_id, cmd, agent}``.
* ``post`` sends the same tool_id with ``exit`` and ``dur_s`` (the CLI's duration, else since pre).

Lines go to ``hook.path`` on ``hook.target`` over a detached ``ssh ... cat >>`` (appended directly
when this is the target), at most ``hook.max_per_10s`` per session; the excess folds into the next
line as ``folded``. Delivery failures go to ``$XDG_STATE_HOME/boat_agent_hook.log`` and never reach
the agent. Advisory, not a boundary: the mux, the claim and the red external chip are the safety.
"""

import fcntl
import json
import os
from pathlib import Path
import re
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
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'agent.yaml'
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
# Why a denied command is denied and the verb that does it through the claim, first match wins.
USE = ((r'/cmd_vel|/mavros/(rc/override|setpoint)', 'raw motion', 'run KEY --claim ID'),
       (r'mavros', 'an FCU arming, mode or parameter change', None),
       (r'command_mux', 'commanding the mux', 'stop'),
       (r'\b(pkill|killall)\b', 'killing processes', 'stop'),
       (r'goal_pose|navigate|follow_waypoints', 'a raw Nav2 goal', 'goal X Y [YAW] --claim ID'),
       (r'.', 'raw motion', 'run KEY --claim ID'))


def private_dir(directory=None):
    """``directory``, else ``$XDG_RUNTIME_DIR/boat_agent_hook``, else ``~/.cache/boat_agent_hook``:
    user-only (0700), never a shared /tmp another user could plant a state file in."""
    if directory:
        return Path(directory)
    base = os.environ.get('XDG_RUNTIME_DIR') or Path.home() / '.cache'
    out = Path(base) / 'boat_agent_hook'
    out.mkdir(mode=0o700, parents=True, exist_ok=True)
    return out


def hook_config(path=CONFIG):
    """``hook:`` of config/agent.yaml over the built-in defaults."""
    try:
        import yaml
        found = (yaml.safe_load(Path(path).read_text()) or {}).get('hook') or {}
    except Exception:          # no PyYAML, or a missing or broken file: the defaults are the shipped values
        return dict(DEFAULTS)
    return {**DEFAULTS, **{k: v for k, v in found.items() if k in DEFAULTS}}


def who_of(event):
    """``$BOAT_AGENT``, else the agent CLI (claude, codex) and the first 4 hex digits of the session."""
    explicit = os.environ.get('BOAT_AGENT')
    if explicit:
        return explicit
    digits = re.sub(r'[^0-9a-f]', '', str(event.get('session_id', '')).lower())
    return f'{event.get("agent") or "agent"}-{digits[:4] or "0000"}'


def start_text(who, agent='claude'):
    """The playbook the agent gets at session start (and after a compaction or /clear)."""
    cli = f'ssh blastoise {BOAT_AGENT} <verb> --who {who}'
    background = 'Bash run_in_background' if agent == 'claude' else 'a background shell'
    return '\n'.join((
        f'RoboBoat playbook: you are boat agent {who}.',
        f'Act on the boat only through `{cli}` (JSON out):',
        '  1. `status`: what runs and who drives.',
        '  2. `claim --reason "..."`: ask the operator for the boat.',
        '  3. `run KEY` or `goal X Y [YAW]` with `--claim ID --dry-run`, then for real.',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  `stop` any time. `ask "..."` instead of guessing. `note "..."` writes a line in the TUI log.',
        'Exit codes: 0 ok, 2 refused (read hint), 3 revoked or overridden (stop: a human took over; '
        'claim again only if asked), 4 no TUI, 5 agents blocked, 6 timeout.',
        'Never publish /cmd_vel, /goal_pose, mux or MAVROS commands, and never arm or release a hold: '
        'the hook blocks them and names the verb. Run in sim first. For runs over 9 min use '
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands.'))


def deny_reason(command, who):
    """One line: what was blocked, why, and the boat_agent verb that does it instead."""
    what, verb = next((w, v) for p, w, v in USE if re.search(p, plain(command), re.S))
    head = f'Blocked: {what} from a shell bypasses the boat claim and the operator.'
    if verb is None:
        return f'{head} Those stay with the operator; ask them: ssh blastoise {BOAT_AGENT} ask "..." --who {who}'
    return (f'{head} Use: ssh blastoise {BOAT_AGENT} {verb} --who {who}'
            + ('' if verb == 'stop' else f' (claim first: claim --reason "..." --who {who})'))


def use_of(command):
    """The short verb a deny names, for the TUI row."""
    verb = next(v for p, _, v in USE if re.search(p, plain(command), re.S))
    return f'boat_agent {verb.split()[0]}' if verb else 'operator'


def plain(text):
    """The command as the shell reads its words: continuations joined, quotes dropped, so
    ``/cmd''_vel`` or ``ros2 'topic' pub`` cannot hide a match."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '')


def matches(patterns, text):
    """Any pattern found in ``plain(text)``; ``.`` spans newlines, so a multi-line command cannot
    hide a match."""
    text = plain(text)
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


def hostname():
    return os.uname().nodename


def deliver(line, config):
    """Append one line on the target: locally when this is it, else a detached ssh; never waits."""
    data = (json.dumps(line, separators=(',', ':')) + '\n').encode()
    try:
        if hostname().split('.')[0] == config['target']:
            fd = os.open(config['path'], os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o666)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
            return
        import subprocess      # only off the target: keeps the hook's import cost low
        child = subprocess.Popen(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=2', config['target'],
             f"cat >> '{config['path']}'"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        child.stdin.write(data)
        child.stdin.close()
    except OSError as error:
        log(f'deliver to {config["target"]}: {error!r}')


def agent_event(event, config=None, send=None, now=None, state_dir=None):
    """The answer to one neutral event (None, ``{'context'}`` or ``{'deny'}``); sends at most one note."""
    phase, agent = event.get('event'), event.get('agent') or 'agent'
    who = who_of(event)
    if phase == 'start':
        return {'context': start_text(who, agent)}
    if phase not in ('pre', 'post'):
        return None
    config = config or hook_config()
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    command = str(event.get('command') or '')
    answer = None
    if phase == 'pre' and matches(config['deny'], command):
        answer, phase = {'deny': deny_reason(command, who)}, 'deny'
    elif not matches(config['match'], command) or matches(config['skip'], command):
        return None
    tool_id = event.get('tool_id')
    note = {'phase': phase, 'tool_id': tool_id, 'cmd': redact(command), 'exit': None, 'dur_s': None,
            'target': config['target'], 'agent': agent}
    if phase == 'deny':
        note['use'] = use_of(command)
    with State(str(event.get('session_id') or 'none'), state_dir) as state:
        if phase == 'pre':
            state.data['started'] = {**dict(list(state.data['started'].items())[-31:]), str(tool_id): now}
        elif phase == 'post':
            note['exit'] = event.get('exit')            # None: interrupted or not reported
            began = state.data['started'].pop(str(tool_id), None)
            took = event.get('duration_ms')
            note['dur_s'] = (round(took / 1000, 2) if isinstance(took, (int, float))
                             else None if began is None else round(now - began, 2))
        admitted, folded = state.admit(now, config['max_per_10s'])
    if admitted:
        send({'schema': SCHEMA, 't_ns': int(now * 1e9), 'host': hostname(), 'pid': os.getpid(),
              'who': who, 'kind': 'note', **note, **({'folded': folded} if folded else {})})
    return answer
