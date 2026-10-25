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

Lines go to this machine's channel (``agent/channel.jsonl`` under the data root, where a sim or
dev TUI reads) unless a boat session names its host: ``$BOAT_AGENT_TARGET`` (set for the agent
session that works on the boat), else ``hook.target`` (``local`` as shipped). Only then do they go
to ``hook.path`` on that host over a detached ``ssh ... cat >>`` (appended directly when this is
it), so sim and dev work never reaches the boat. At most ``hook.max_per_10s`` per session; the
excess folds into the next line as ``folded``. Delivery failures go to ``$XDG_STATE_HOME/boat_agent_hook.log`` and never reach
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
    'target': 'local', 'path': '/var/home/roboboat/astro_dock/bags/agent/channel.jsonl',
    'match': [r'\bssh\s+(-\S+\s+)*(blastoise|squirtle)\b', r'\bros2\b', r'\bdocker\s+exec\b'],
    'skip': [r'\bboat_(trial|agent)\b'], 'max_per_10s': 6,
    'deny': [r'ros2\s+topic\s+pub\b.*(\bcmd_vel|\bcommand_mux\b|\bgoal_pose|\bmavros/(rc/override|setpoint|manual_control))',
             r'ros2\s+action\s+send_goal\b.*(navigate|(?<![\w-])(spin|backup|drive_on_heading|follow_waypoints'
             r'|follow_gps_waypoints|follow_path|dock_robot|undock_robot|assisted_teleop)\b)',
             r'ros2\s+service\s+call\b.*\bmavros/(cmd/(arming|command)|set_mode|param/(set|push|load))',
             r'ros2\s+(param\s+(set|load|delete)|lifecycle\s+set)\b.*'
             r'\b(mavros|command_mux|\w*watchdog|thruster_pwm\w*|\w*velocity_controller)\b',
             r'ros2\s+service\s+call\b.*\b(mavros\w*|command_mux|\w*watchdog|thruster_pwm\w*|\w*velocity_controller)'
             r'/(set_parameters\w*|load_parameters|change_state)\b',
             r'ros2\s+bag\s+play\b(?!.*--topics\b)',
             r'ros2\s+bag\s+play\b.*--topics\b.*(cmd_vel|thrusters?\b|thruster_|command_mux'
             r'|mavros/(rc/override|setpoint|manual_control))',
             r'ros2\s+run\s+(mavros\s+mav|teleop_twist_\w+|mhseals_hardware\s+keyboard_control)\b',
             r'\b(pkill|killall)\b',
             r'\bkill\b.*\b(command_mux|\w*watchdog|thruster_pwm\w*|velocity_controller)\b']}
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the verb that does it through the claim, first match wins.
USE = ((r'ros2\s+bag\s+play', 'replaying a bag that can carry motion topics (play only --topics without '
        'cmd_vel, thruster or mux topics)', 'run KEY --claim ID'),
       (r'cmd_vel|mavros/(rc/override|setpoint|manual_control)|teleop|keyboard_control', 'raw motion',
        'run KEY --claim ID'),
       (r'mavros', 'an FCU arming, mode or parameter change', None),
       (r'ros2\s+(param|lifecycle)\b|/(set_parameters\w*|load_parameters|change_state)\b',
        'changing the thruster driver, the mux or a safety watchdog', None),
       (r'\b(pkill|killall|kill)\b', 'killing processes', 'stop'),
       (r'command_mux', 'commanding the mux', 'stop'),
       (r'goal_pose|navigate|follow_\w*waypoints|follow_path|dock_robot|assisted_teleop', 'a raw Nav2 goal',
        'goal X Y [YAW] --claim ID'),
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


def target_of(config):
    """The host whose TUI gets the notes: ``$BOAT_AGENT_TARGET``, else ``hook.target``; 'local' (or
    empty) is this machine."""
    return (os.environ.get('BOAT_AGENT_TARGET') or config.get('target') or 'local').strip() or 'local'


def remote(config):
    """The target host when it is another machine, else None."""
    target = target_of(config)
    return None if target == 'local' or hostname().split('.')[0] == target else target


def local_channel():
    """This machine's channel: data.path('agent', 'channel.jsonl') (``$MHSEALS_DATA``, else
    ``$ROS_WS/bags``, else ``~/astro_dock/bags``), without importing the package."""
    data = os.environ.get('MHSEALS_DATA')
    if data and os.path.isabs(data):
        return Path(data).resolve() / 'agent' / 'channel.jsonl'
    ws = os.environ.get('ROS_WS')
    ws = Path(ws).resolve() if ws and os.path.isabs(ws) else Path.home() / 'astro_dock'
    return ws / 'bags' / 'agent' / 'channel.jsonl'


def cli_of(config):
    """How the agent calls boat_agent: over ssh to a remote target, else this machine's checkout."""
    host = remote(config)
    return f'ssh {host} {BOAT_AGENT}' if host else BOAT_AGENT


def who_of(event):
    """``$BOAT_AGENT``, else the agent CLI (claude, codex) and the first 4 hex digits of the session."""
    explicit = os.environ.get('BOAT_AGENT')
    if explicit:
        return explicit
    digits = re.sub(r'[^0-9a-f]', '', str(event.get('session_id', '')).lower())
    return f'{event.get("agent") or "agent"}-{digits[:4] or "0000"}'


def start_text(who, agent='claude', config=None):
    """The playbook the agent gets at session start (and after a compaction or /clear)."""
    cli = f'{cli_of(config or hook_config())} <verb> --who {who}'
    background = 'Bash run_in_background' if agent == 'claude' else 'a background shell'
    return '\n'.join((
        f'RoboBoat playbook: you are boat agent {who}.',
        f'Act on the boat only through `{cli}` (JSON out):',
        '  1. `status`: what runs and who drives.',
        '  2. `claim --reason "..."`: ask the operator for the boat.',
        '  3. `run KEY` (`run mission --task NAME`) or `goal X Y [YAW]` with `--claim ID --dry-run`, then for real.',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  `stop` any time. `ask "..."` instead of guessing. `note "..."` writes a line in the TUI log.',
        'Exit codes: 0 ok, 2 refused (read hint), 3 revoked or overridden (stop: a human took over; '
        'claim again only if asked), 4 no TUI, 5 agents blocked, 6 timeout.',
        'Never publish /cmd_vel, /goal_pose, mux or MAVROS commands, and never arm or release a hold: '
        'the hook blocks them and names the verb. Run in sim first. For runs over 9 min use '
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands.'))


def deny_reason(command, who, config=None):
    """One line: what was blocked, why, and the boat_agent verb that does it instead."""
    what, verb = next((w, v) for p, w, v in USE if re.search(p, plain(command), re.S))
    head = f'Blocked: {what} from a shell bypasses the boat claim and the operator.'
    cli = cli_of(config or hook_config())
    if verb is None:
        return f'{head} Those stay with the operator; ask them: {cli} ask "..." --who {who}'
    return (f'{head} Use: {cli} {verb} --who {who}'
            + ('' if verb == 'stop' else f' (claim first: claim --reason "..." --who {who})'))


def use_of(command):
    """The short verb a deny names, for the TUI row."""
    verb = next(v for p, _, v in USE if re.search(p, plain(command), re.S))
    return f'boat_agent {verb.split()[0]}' if verb else 'operator'


def plain(text):
    """The command as the shell reads its words: continuations joined, quotes and escapes dropped,
    so ``/cmd''_vel``, ``/cmd\\_vel`` or ``ros2 'topic' pub`` cannot hide a match."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '').replace('\\', '')


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
    """Append one line to the target's channel: here unless a boat session names another host (then a
    detached ssh); never waits."""
    data = (json.dumps(line, separators=(',', ':')) + '\n').encode()
    target, host = target_of(config), remote(config)
    try:
        if host is None:
            path = local_channel() if target == 'local' else config['path']   # or this is the target host
            fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o666)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
            return
        import subprocess      # only for a remote target: keeps the hook's import cost low
        child = subprocess.Popen(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=2', host,
             f"cat >> '{config['path']}'"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        child.stdin.write(data)
        child.stdin.close()
    except OSError as error:
        log(f'deliver to {target}: {error!r}')


def agent_event(event, config=None, send=None, now=None, state_dir=None):
    """The answer to one neutral event (None, ``{'context'}`` or ``{'deny'}``); sends at most one note."""
    phase, agent = event.get('event'), event.get('agent') or 'agent'
    who = who_of(event)
    if phase not in ('start', 'pre', 'post'):
        return None
    config = config or hook_config()
    if phase == 'start':
        return {'context': start_text(who, agent, config)}
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    command = str(event.get('command') or '')
    answer = None
    if phase == 'pre' and matches(config['deny'], command):
        answer, phase = {'deny': deny_reason(command, who, config)}, 'deny'
    elif not matches(config['match'], command) or matches(config['skip'], command):
        return None
    tool_id = event.get('tool_id')
    note = {'phase': phase, 'tool_id': tool_id, 'cmd': redact(command), 'exit': None, 'dur_s': None,
            'target': target_of(config), 'agent': agent}
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
