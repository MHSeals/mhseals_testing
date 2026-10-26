"""The agent hook (spec agent-control section 6): playbook, raw-motion denies, notes to the TUI.

astro_dock's ``.agents/hook.py`` loads this file (``src/<repo>/<package>/agent_hook.py``) and calls
``agent_event`` for Claude Code and Codex alike with one neutral event (``agent``, ``event``
start|pre|post, ``session_id``, ``tool_id``, ``command``, ``exit``, ``duration_ms``, ``failed``).
Imported under ``python3 -I`` on the dev machine: no ROS. PyYAML, when present, reads
config/agent.yaml (about 15 ms per call); without it the built-in defaults apply.

* ``start`` returns a brief (the agent's boat name, the one rule, where the playbook is); the
  whole playbook (the ``boat_agent`` verbs, exit codes) in a boat session or for ``--playbook``.
* ``pre`` denies a command that runs something matching ``hook.deny`` (raw /cmd_vel, /goal_pose,
  mux, MAVROS arming/command/mode/params, Nav2 goals and behaviors, killing the boat's control) or,
  over ssh to one of ``hook.hosts``, ``hook.deny_boat``: each command the shell would run is matched
  from its start, wrappers taken off (see ``runs``), so text a command only reads is never denied.
  The reason names the command and the verb to use instead; the TUI gets it as ``phase: deny``. A
  boat-bound command (``hook.match``, not ``hook.skip``) becomes a channel ``note`` line
  ``{phase: pre, tool_id, cmd, agent}``.
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
import shlex
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
             r'\b(mavros|command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller)\b',
             r'ros2\s+service\s+call\b.*\b(mavros\w*|command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller)'
             r'/(set_parameters\w*|load_parameters|change_state)\b',
             r'ros2\s+bag\s+play\b(?!.*--topics\b)',
             r'ros2\s+bag\s+play\b.*--topics\b.*(cmd_vel|thrusters?\b|thruster_|command_mux'
             r'|mavros/(rc/override|setpoint|manual_control))',
             r'ros2\s+run\s+(mavros\s+mav|teleop_twist_\w+|mhseals_hardware\s+keyboard_control)\b',
             r'ros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)|lifecycle\s+set|run)'
             r'\s+(--?[\w-]+\s+)*[^\s-]*\$',
             r'(pkill|killall|kill)\b.*\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller'
             r'|mavros\w*|boat_(trial|test)\w*)\b'],
    'hosts': ['blastoise', 'squirtle'],
    'deny_boat': [r'(pkill|killall)\b', r'(docker|podman)\b.*\s(stop|kill|restart|rm|down|pause)\b',
                  r'systemctl\s+(--user\s+)?(stop|kill|restart|isolate|reboot|poweroff|halt)\b',
                  r'(reboot|poweroff|halt|shutdown)\b']}
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the verb that does it through the claim, first match wins.
USE = ((r'^(pkill|killall|kill)\b', 'killing boat processes', 'stop'),
       (r'^(docker|podman|systemctl|reboot|poweroff|halt|shutdown)\b',
        "stopping or restarting the boat's software or computer", None),
       (r'\$', 'a ros2 command whose target is a shell variable the hook cannot read (spell the name out)', None),
       (r'ros2\s+bag\s+play', 'replaying a bag that can carry motion topics (play only --topics without '
        'cmd_vel, thruster or mux topics)', 'run KEY --claim ID'),
       (r'cmd_vel|mavros/(rc/override|setpoint|manual_control)|teleop|keyboard_control', 'raw motion',
        'run KEY --claim ID'),
       (r'mavros', 'an FCU arming, mode or parameter change', None),
       (r'ros2\s+(param|lifecycle)\b|/(set_parameters\w*|load_parameters|change_state)\b',
        'changing the thruster driver, the mux or a safety watchdog', None),
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


def brief_text(who, config=None):
    """What every session gets at start (and after a compaction or /clear) unless it works on the
    boat: the one rule and where the playbook is, about 100 tokens instead of the playbook's 350."""
    cli = cli_of(config or hook_config())
    return (f'RoboBoat workspace (drives a real boat). You are boat agent {who}. Act on the boat or its sim '
            f'only through `{cli} <verb> --who {who}`; never publish /cmd_vel, /goal_pose, mux or MAVROS '
            'commands or kill its nodes yourself (the hook blocks those and names the verb). Before your '
            f'first boat command read the playbook: `BOAT_AGENT={who} python3 .agents/hook.py --playbook`. '
            'Repo-only work needs none of this.')


def start_text(who, agent='claude', config=None):
    """The playbook: at start in a boat session (``$BOAT_AGENT_TARGET``), else ``hook.py --playbook``."""
    config = config or hook_config()
    cli = f'{cli_of(config)} <verb> --who {who}'
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
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands'
        + (f' ({remote(config)}).' if remote(config) else
           ': this machine\'s TUI; a boat session sets BOAT_AGENT_TARGET=blastoise in your environment.')))


def deny_reason(hit, host=None, who='agent', config=None):
    """One line: the command blocked, why, and the boat_agent verb that does it instead."""
    what, verb = next((w, v) for p, w, v in USE if re.search(p, hit))
    where = f' on {host}' if host else ''
    head = f'Blocked `{hit[:80]}`{where}: {what} from a shell bypasses the boat claim and the operator.'
    here = not host or hostname().split('.')[0] == host
    cli = cli_of(config or hook_config()) if here else f'ssh {host} {BOAT_AGENT}'    # the boat it aimed at
    if verb is None:
        return f'{head} Those stay with the operator; ask them: {cli} ask "..." --who {who}'
    return (f'{head} Use: {cli} {verb} --who {who}'
            + ('' if verb == 'stop' else f' (claim first: claim --reason "..." --who {who})')
            + f'. Verbs and exit codes: BOAT_AGENT={who} python3 .agents/hook.py --playbook')


def use_of(hit):
    """The short verb a deny names, for the TUI row."""
    verb = next(v for p, _, v in USE if re.search(p, hit))
    return f'boat_agent {verb.split()[0]}' if verb else 'operator'


def plain(text):
    """The command as the shell reads its words: continuations joined, quotes and escapes dropped,
    so ``/cmd''_vel``, ``/cmd\\_vel`` or ``ros2 'topic' pub`` cannot hide a match."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '').replace('\\', '')


def matches(patterns, text):
    """Any pattern found in ``plain(text)``; ``.`` spans newlines (the TUI-note filter)."""
    text = plain(text)
    return any(re.search(p, text, re.S) for p in patterns)


# What a shell would run. A deny pattern is matched from the start of each command the shell runs
# (wrappers taken off), never against text a command only reads: a grep pattern, an echo, a commit
# message or a heredoc into a file is data. Code handed to an interpreter (python -c, a heredoc into
# python, a script written and run in the same call) is searched line by line; so is a command the
# parser cannot read. Variables, eval'd strings built at run time and scripts from earlier calls still
# get through: advisory, the mux and the claim are the boundary.
SEPARATORS = {';', '&', '&&', '||', '|', '|&', '\n', ';;', '(', ')'}
REDIRECTS = {'<', '>', '>>', '>|', '&>', '&>>', '>&', '<&', '<>'}
WRITES = {'>', '>>', '>|', '&>', '&>>'}
PUNCT = re.compile(r'<<<|<<|>>|>&|<&|<>|&>>|&>|>\||\|\||&&|\|&|;;|[\s\S]')
HEREDOC = re.compile(r'(?<!<)<<(?!<)-?\s*([\'"]?)([\w.-]+)\1')
SUBST = re.compile(r'\$\(([^\n]*)|`([^`]*)`')
KEYWORDS = {'!', '{', '}', 'if', 'then', 'else', 'elif', 'fi', 'do', 'done', 'while', 'until', 'time',
            'exec', 'command', 'builtin', 'nohup', 'setsid'}
ASSIGN = re.compile(r'[A-Za-z_]\w*=')
SHELLS = {'sh', 'bash', 'zsh', 'dash', 'ksh'}
CODE = re.compile(r'(python[\d.]*|node|perl|ruby)$')
# A wrapper runs its operands: (option letters that take a value, long ones, positionals before the command).
WRAPPERS = {'sudo': ('ugpChDrtUT', (), 0), 'doas': ('uC', (), 0), 'env': ('uCS', (), 0),
            'timeout': ('sk', ('--signal', '--kill-after'), 1), 'nice': ('n', (), 0), 'ionice': ('cnp', (), 0),
            'stdbuf': ('ioe', (), 0), 'xargs': ('IinPLdaEs', (), 0), 'watch': ('nd', ('--interval',), 0)}
SSH = 'BbcDEeFIiJLlmOoPpQRSWw'
DOCKER = ('Hcl', ('--context', '--host', '--config', '--log-level'))
COMPOSE = ('fp', ('--file', '--project-name', '--project-directory', '--profile', '--env-file', '--ansi',
                  '--progress', '--parallel'))
EXEC = ('eEuw', ('--env', '--env-file', '--user', '--workdir', '--detach-keys', '--index'))


def operands(args, valued='', long_valued=(), positionals=0):
    """``args`` past the options (getopt style: ``valued`` letters and ``long_valued`` take a value)
    and past ``positionals`` leading operands."""
    i = 0
    while i < len(args) and args[i].startswith('-') and args[i] != '-':
        if args[i] == '--':
            i += 1
            break
        if args[i].startswith('--'):
            i += 1 + (args[i] in long_valued)
            continue
        letters = args[i][1:]
        hit = next((k for k, letter in enumerate(letters) if letter in valued), None)
        i += 1 + (hit == len(letters) - 1)
    return args[i + positionals:]


def tokens_of(text):
    """Shell words and operators; a quoted word stays one word. Raises ValueError on an open quote."""
    lexer = shlex.shlex(text, posix=True, punctuation_chars=';&|()<>`\n')
    lexer.whitespace, lexer.commenters, lexer.whitespace_split = ' \t\r', '', True
    out = []
    for token in lexer:
        out += PUNCT.findall(token) if set(token) <= set(';&|()<>`\n') else [token]
    return out


def parse(text, bodies=None):
    """``[pipeline [command {argv, stdin, writes, subs}]]`` of ``text``. Heredoc bodies come out of the
    text first (``bodies`` shared with nested substitutions); ``subs`` are ``$(...)`` and backticks."""
    if bodies is None:
        lines, kept, found = text.replace('\\\n', '').split('\n'), [], []
        i = 0
        while i < len(lines):
            kept.append(lines[i])
            i += 1
            for match in HEREDOC.finditer(kept[-1]):
                dash, body = '<<-' in match.group(0), []
                while i < len(lines) and (lines[i].lstrip('\t') if dash else lines[i]) != match.group(2):
                    body.append(lines[i])
                    i += 1
                i += 1
                found.append(('\n'.join(body), bool(match.group(1))))
        bodies, text = iter(found), '\n'.join(kept)
    return build(tokens_of(text), bodies)


def build(tokens, bodies):
    pipelines, pipe = [], []
    cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': []}

    def end(piped):
        nonlocal cmd, pipe
        if any(cmd.values()):
            pipe.append(cmd)
        cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': []}
        if not piped and pipe:
            pipelines.append(pipe)
            pipe = []

    def nested(text):
        for match in SUBST.finditer(text):
            cmd['subs'].append(parse(match.group(1) or match.group(2) or '', bodies))

    i = 0
    while i < len(tokens):
        token = tokens[i]
        i += 1
        if token == '`' or (token == '(' and cmd['argv'] and cmd['argv'][-1].endswith('$')):
            depth, j = 1, i
            while j < len(tokens) and depth:
                depth += (-(tokens[j] == '`') if token == '`' else (tokens[j] == '(') - (tokens[j] == ')'))
                j += 1
            inner = tokens[i:j - (depth == 0)]
            cmd['subs'].append(build(inner, bodies))
            cmd['argv'] += [t for t in inner if t not in SEPARATORS]     # `kill $(pgrep x)` names x
            i = j
        elif token in ('|', '|&'):
            end(True)
        elif token in SEPARATORS:
            end(False)
        elif token == '<<':
            i += 1                                                       # the delimiter word
            body, quoted = next(bodies, ('', True))
            cmd['stdin'].append(body)
            if not quoted:                                               # $(...) in it runs
                nested(body)
        elif token == '<<<' or token in REDIRECTS:
            if i < len(tokens) and (token == '<<<' or token in WRITES):
                cmd['stdin' if token == '<<<' else 'writes'].append(tokens[i])
            i += 1
        else:
            cmd['argv'].append(token)
            nested(token)                                                # "$(...)" inside quotes
    end(False)
    return pipelines


def runs(text, hosts=(), on_boat=None, depth=0, out=None, pipelines=None):
    """``[(kind, on_boat, text)]``: ``('cmd', ..)`` each command the shell runs from ``text``, its
    wrappers taken off, as words joined by spaces; ``('code', ..)`` code handed to an interpreter.
    ``on_boat`` is the boat host once an ssh names one of ``hosts`` (what runs there, its heredoc too)."""
    out = [] if out is None else out
    try:
        pipelines = parse(text) if pipelines is None else pipelines
    except ValueError:
        pipelines = None
    if pipelines is None or depth > 8:
        out.append(('code', on_boat, text))
        return out
    written = {}
    for pipeline in pipelines:
        feed = []                              # what earlier stages of the pipe can print
        for cmd in pipeline:
            for sub in cmd['subs']:
                if sub:
                    runs('', hosts, on_boat, depth + 1, out, sub)
            reads, boat, script = unwrap(cmd['argv'], hosts, on_boat, depth, out, feed)
            for text_in in (feed + cmd['stdin']) if reads else ():
                if reads == 'shell':
                    runs(text_in, hosts, boat, depth + 1, out)
                else:
                    out.append(('code', boat, text_in))
            if script in written:
                out.append(('code', boat, written[script]))
            for path in cmd['writes']:
                if cmd['stdin']:
                    written[path] = '\n'.join(cmd['stdin'])
            feed = feed + [' '.join(cmd['argv'][1:])] + cmd['stdin']
    return out


def unwrap(argv, hosts, on_boat, depth, out, feed=()):
    """Take the wrappers off one command and add what finally runs to ``out``. Returns (how it reads
    stdin: None, 'shell' or 'code'; on the boat?; the script file it runs, if any)."""
    def again(text):                           # a command string a shell re-reads (ssh, sh -c, eval, watch)
        try:
            pipelines = parse(text)
        except ValueError:
            pipelines = None
        if pipelines and len(pipelines) == 1 and len(pipelines[0]) == 1 and not any(
                pipelines[0][0][k] for k in ('stdin', 'writes', 'subs')):
            return pipelines[0][0]['argv']
        runs(text, hosts, on_boat, depth + 1, out, pipelines)
        return None

    while argv:
        name = os.path.basename(argv[0])
        if name in KEYWORDS or ASSIGN.match(argv[0]):
            argv = argv[1:]
        elif name in WRAPPERS:
            argv = operands(argv[1:], *WRAPPERS[name])
            if name == 'xargs':
                argv = argv + list(feed)
            elif name == 'watch':
                argv = again(' '.join(argv))
        elif name == 'ssh':
            rest = operands(argv[1:], SSH)
            if not rest:
                return None, on_boat, None
            host = re.split(r'[.:]', rest[0].rsplit('@', 1)[-1])[0]
            on_boat = on_boat or (host if host in hosts else None)
            if len(rest) == 1:
                return 'shell', on_boat, None
            argv = again(' '.join(rest[1:]))
        elif name in ('docker', 'podman'):
            rest = operands(argv[1:], *DOCKER)
            rest = operands(rest[1:], *COMPOSE) if rest[:1] == ['compose'] else rest
            if rest[:1] != ['exec']:
                break
            argv = operands(rest[1:], *EXEC, positionals=1)
        elif name in SHELLS:
            flags, i = '', 1
            while i < len(argv) and argv[i][:1] in '-+' and argv[i] not in ('-', '--'):
                if argv[i] in ('-o', '+o', '-O', '+O', '--rcfile', '--init-file'):
                    i += 1
                elif not argv[i].startswith('--'):
                    flags += argv[i][1:]
                i += 1
            rest = argv[i + (argv[i:i + 1] == ['--']):]
            if 'c' in flags:
                argv = again(rest[0] if rest else '')
            elif rest and 's' not in flags and rest[0] != '-':
                return None, on_boat, rest[0]
            else:
                return 'shell', on_boat, None
        elif name == 'eval':
            argv = again(' '.join(argv[1:]))
        elif name in ('source', '.'):
            return None, on_boat, (argv[1:] or [None])[0]
        elif CODE.match(name):
            args = argv[1:]
            for k, arg in enumerate(args):
                python_c = name[0] == 'p' and re.fullmatch(r'-[A-Za-z]*c', arg)
                if arg in ('-c', '-e', '--eval', '-p', '--print') or python_c:
                    out.append(('code', on_boat, args[k + 1] if k + 1 < len(args) else ''))
                    return None, on_boat, None
                if arg == '-m':
                    return None, on_boat, None
                if arg == '-' or not arg.startswith('-'):
                    return ('code', on_boat, None) if arg == '-' else (None, on_boat, arg)
            return 'code', on_boat, None
        else:
            break
    if argv:
        out.append(('cmd', on_boat, ' '.join([os.path.basename(argv[0])] + argv[1:])))
        return None, on_boat, argv[0]
    return None, on_boat, None


def denied(command, config):
    """``(the command or code line it blocks, the boat host it runs on or None)`` for the first denied
    thing ``command`` runs, else None."""
    hosts = set(config['hosts']) | {target_of(config)} - {'local'}
    for kind, on_boat, text in runs(command, hosts):
        patterns = config['deny'] + (config['deny_boat'] if on_boat else [])
        if kind == 'cmd':
            hit = next((text for p in patterns if re.match(p, text)), None)
        else:
            lines = plain(text).split('\n')
            hit = next((m.group(0) for line in lines for p in patterns for m in [re.search(p, line)] if m), None)
        if hit:
            return hit, on_boat
    return None


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
            if target == 'local' and not Path(path).parent.is_dir():
                return              # no TUI has run here: nobody reads the channel, nothing to log

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
        full = event.get('source') == 'playbook' or remote(config) is not None
        return {'context': start_text(who, agent, config) if full else brief_text(who, config)}
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    command = str(event.get('command') or '')
    answer = blocked = None
    if phase == 'pre':
        blocked = denied(command, config)
    if blocked:
        answer, phase = {'deny': deny_reason(*blocked, who=who, config=config)}, 'deny'
    elif not matches(config['match'], command) or matches(config['skip'], command):
        return None
    tool_id = event.get('tool_id')
    note = {'phase': phase, 'tool_id': tool_id, 'cmd': redact(command), 'exit': None, 'dur_s': None,
            'target': target_of(config), 'agent': agent}
    if phase == 'deny':
        note['use'] = use_of(blocked[0])
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
