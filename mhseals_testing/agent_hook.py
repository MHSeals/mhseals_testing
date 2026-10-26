"""The agent hook (spec agent-control section 6): playbook, raw-motion denies, notes to the TUI.

astro_dock's ``.agents/hook.py`` loads this file (``src/<repo>/<package>/agent_hook.py``) and calls
``agent_event`` for Claude Code and Codex alike with one neutral event (``agent``, ``event``
start|pre|post, ``session_id``, ``tool_id``, ``command``, ``exit``, ``duration_ms``, ``failed``).
Imported under ``python3 -I`` on the dev machine: no ROS. PyYAML, when present, reads
config/agent.yaml (about 15 ms per call); without it the built-in defaults apply.

* ``start`` returns a brief (the agent's boat name, the one rule, where the playbook is); the
  whole playbook (the ``boat_agent`` verbs, exit codes) in a boat session or for ``--playbook``.
* ``pre`` denies a command whose text matches ``hook.deny`` (raw /cmd_vel, /goal_pose, mux,
  MAVROS arming/command/mode/params, Nav2 goals and behaviors, pkill/killall, killing the boat's
  control) or, when it names a boat (``boats``), ``hook.deny_boat``. A call made only of commands
  that read or print text (``text_only``: grep, echo, git commit, pytest ...) is never denied.
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
    'match': [r'\bssh\s+(-\S+\s+)*(blastoise(-odroid|-jetson)?|squirtle(-jetson)?)\b', r'\bros2\b',
              r'\bdocker\s+exec\b'],
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
             r'ros2\s+run\s+(mavros\s+mav\w*|teleop_twist_\w+|mhseals_hardware\s+keyboard_control)\b',
             r'ros2\s+run\s+mhseals_hardware\s+(thruster_pwm_node|command_mux|calibrated_velocity_controller)\b',
             r'ros2\s+(topic\s+pub|service\s+call)\b.*(\boperator/heartbeat|\bmhseals_testing/authority'
             r'|\bthruster_pwm_node/estop|\bthrusters/)',
             r'python[\d.]*(\s+-\w+)*\s+(-[A-Za-z]*c|-)\s.*\bcreate_publisher\b',
             r'ros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)|lifecycle\s+set|run)'
             r'\s+(--?[\w-]+\s+)*[^\s-]*\$',
             r'\bxargs\b[^|;&\n]*\bros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)'
             r'|lifecycle\s+set|run)\b',
             r'\b(pkill|killall)\b', r'\bxargs\s+(-\S+\s+)*(sudo\s+)?kill\b',
             r'\bkill\b.*\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller'
             r'|mavros\w*|boat_(trial|test)\w*)\b'],
    'hosts': ['blastoise-odroid', 'blastoise-jetson', 'squirtle-jetson'],
    'deny_boat': [r'\b(pkill|killall|kill)\b', r'\b(docker|podman)\b.*\s(stop|kill|restart|rm|down|pause)\b',
                  r'\bsystemctl\s+(--user\s+)?(stop|kill|restart|isolate|reboot|poweroff|halt)\b',
                  r'\b(reboot|poweroff|halt|shutdown)\b', r'\b(tel)?init\s+[0-6sS]\b',
                  r'\bloginctl\s+(terminate|kill)', r'sysrq']}
NETWORK = 'config/boat-network.env'    # legacy path; STATIC_LEASES "MAC=hostname=address ...": the boat computers
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the verb that does it through the claim, first match wins.
USE = ((r'^(xargs\s+(-\S+\s+)*(sudo\s+)?)?(pkill|killall|kill)\b','killing processes (kill only your own, by exact PID)', 'stop'),
       (r'^(docker|podman|systemctl|reboot|poweroff|halt|shutdown|init|telinit|loginctl)\b|sysrq',
        "stopping or restarting the boat's software or computer", None),
       (r'ros2\s+bag\s+play', 'replaying a bag that can carry motion topics (play only --topics without '
        'cmd_vel, thruster or mux topics)', 'run KEY --claim ID'),
       (r'ros2\s+run\s+mhseals_hardware\s+(thruster_pwm_node|command_mux|calibrated_velocity_controller)',
        'arming (starting the thruster driver or the mux)', 'run KEY --sim-arm --claim ID'),
       (r'cmd_vel|thrusters/|mavros/(rc/override|setpoint|manual_control)|teleop|keyboard_control|create_publisher',
        'raw motion', 'run KEY --claim ID'),
       (r'operator/heartbeat|mhseals_testing/authority', "the operator's heartbeat or authority", None),
       (r'estop', 'the thruster e-stop', 'stop'),
       (r'mavros', 'an FCU arming, mode or parameter change', None),
       (r'\$', 'a ros2 command whose target is a shell variable the hook cannot read (spell the name out)', None),
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
        '     Sim only: `run KEY --sim-arm` arms a verified sim for that run and disarms after; a real boat '
        'refuses it.',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  `stop` any time. `ask "..."` instead of guessing. `note "..."` writes a line in the TUI log.',
        'Exit codes: 0 ok, 2 refused (read hint), 3 revoked or overridden (stop: a human took over; '
        'claim again only if asked), 4 no TUI, 5 agents blocked, 6 timeout.',
        'Never publish /cmd_vel, /goal_pose, mux or MAVROS commands, never arm (but --sim-arm) or release a hold: '
        'the hook blocks them and names the verb; never answer a TUI popup. Run in sim first. For runs over 9 min use '
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands'
        + (f' ({remote(config)}).' if remote(config) else
           ': this machine\'s TUI; a boat session sets BOAT_AGENT_TARGET=blastoise-odroid in your environment '
           '(`blastoise` still works for one release; it means blastoise-odroid).')))


def deny_reason(hit, host=None, who='agent', config=None):
    """One line: the command blocked, why, and the boat_agent verb that does it instead."""
    config = config or hook_config()
    what, verb = next((w, v) for p, w, v in USE if re.search(p, hit))
    where = f' on {host}' if host else ''
    head = f'Blocked `{hit[:80]}`{where}: {what} from a shell bypasses the boat claim and the operator.'
    here = not host or hostname().split('.')[0] == host
    # A bare vessel name (blastoise: a removed ssh alias) is reached through its first host.
    target = host if host in config['hosts'] else next(
        (h for h in config['hosts'] if h.split('-')[0] == host), host)
    cli = cli_of(config) if here else f'ssh {target} {BOAT_AGENT}'    # the boat it aimed at
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


# Two tiers. Tier A: every command in the call (each part of a pipe or list, each $(...)) only reads,
# searches or prints text (READ_ONLY, git commit/log/diff/show/status, pytest): its arguments are data,
# so a grep pattern, an echo, a commit message or a heredoc into a file never denies. Tier B, anything
# else: the whole command text is searched (quotes and escapes dropped, ``.`` spanning newlines), so
# whatever wraps or hides a denied command is still denied. Variables, strings built at run time and
# scripts from earlier calls still get through: advisory, the mux and the claim are the boundary.
SEPARATORS = {';', '&', '&&', '||', '|', '|&', '\n', ';;', '(', ')'}
REDIRECTS = {'<', '>', '>>', '>|', '&>', '&>>', '>&', '<&', '<>'}
WRITES = {'>', '>>', '>|', '&>', '&>>'}
PUNCT = re.compile(r'<<<|<<|>>|>&|<&|<>|&>>|&>|>\||\|\||&&|\|&|;;|[\s\S]')
HEREDOC = re.compile(r'(?<!<)<<(?!<)-?\s*([\'"]?)([\w.-]+)\1')
SUBST = re.compile(r'\$\(([^\n]*)|`([^`]*)`')
READ_ONLY = {'grep', 'egrep', 'fgrep', 'rg', 'echo', 'printf', 'cat', 'head', 'tail', 'less', 'wc'}
GIT_READ = {'commit', 'log', 'diff', 'show', 'status'}
_ADDR = r'(\d+|\$|/[^/\n]*/)(,(\d+|\$|/[^/\n]*/))?'
_SED = rf'({_ADDR})?!?(p|s/[^/\n]*/[^/\n]*/[gpI\d]*)'
SED_PLAIN = re.compile(rf'{_SED}(;{_SED})*;?')      # print and s/// only: no e, w, r or other delimiters


def reads_only(argv):
    """True when this command only reads, searches or prints: none of its words can run anything or
    write a file."""
    if not argv:
        return False
    name, args = argv[0], argv[1:]
    if name == 'git':                   # no -c, --exec, --ext-diff, --textconv or --output
        while args[:1] == ['--no-pager'] or args[:1] == ['-C']:
            args = args[1 + (args[0] == '-C'):]
        return args[:1] != [] and args[0] in GIT_READ and not any(
            a.startswith(('-c', '--exec', '--ext-diff', '--textconv', '--output')) for a in args)
    if name in ('pytest', 'py.test') or (re.fullmatch(r'python[\d.]*', name) and args[:2] == ['-m', 'pytest']):
        return True
    if name == 'sed':                   # sed -n [-E|-r] SCRIPT FILE...: SCRIPT only prints and substitutes
        flags = [a for a in args if a.startswith('-')]
        rest = [a for a in args if not a.startswith('-')]
        return (all(re.fullmatch(r'-[nEr]+', a) for a in flags) and 'n' in ''.join(flags) and bool(rest)
                and args.index(rest[0]) == len(flags) and bool(SED_PLAIN.fullmatch(rest[0])))
    if name == 'rg':
        return not any(a.startswith('--pre') for a in args)     # --pre runs a program on each file
    if name == 'less':                  # +!cmd runs it; -o/-O/--log-file write a file
        return not any(a.startswith(('+', '--log', '--LOG')) or re.match(r'-[^-]*[oO]', a) for a in args)
    return name in READ_ONLY


def text_only(command):
    """Tier A, side-effect free: no $(...), backticks or <(...)/>(...) anywhere (here-strings and
    heredocs included), no file written (only /dev/null and 2>&1), and every command ``reads_only``."""
    if re.search(r'\$\(|`|[<>]\(', command):
        return False
    try:
        pipelines = parse(command)
    except ValueError:                  # an open quote: unreadable, so Tier B
        return False
    return bool(pipelines) and all(reads_only(cmd['argv']) and not cmd['writes'] and not cmd['subs']
                                   for pipeline in pipelines for cmd in pipeline)


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
            target = tokens[i] if i < len(tokens) else ''
            if token == '<<<':
                cmd['stdin'].append(target)
            elif token not in ('<', '<&') and target != '/dev/null' and not (
                    token == '>&' and re.fullmatch(r'\d|-', target)):
                cmd['writes'].append(target)                             # a file written (not 2>&1)
            i += 1
        else:
            cmd['argv'].append(token)
            nested(token)                                                # "$(...)" inside quotes
    end(False)
    return pipelines


def network_file():
    """The vessel's network env: ``$ASTRO_NETWORK_ENV`` (None when set but missing, so ``denied``
    fails closed), else the legacy config/boat-network.env of this workspace (``src/<repo>/<package>/``
    up three), of ``$ROS_WS`` or of ``~/astro_dock``."""
    explicit = os.environ.get('ASTRO_NETWORK_ENV')
    if explicit:
        path = Path(explicit).expanduser()
        return path if path.is_file() else None
    ws = os.environ.get('ROS_WS')
    for root in (Path(__file__).resolve().parents[3], ws if ws and os.path.isabs(ws) else None,
                 Path.home() / 'astro_dock'):
        if root and (Path(root) / NETWORK).is_file():
            return Path(root) / NETWORK
    return None


def boats(config):
    """Every name a boat computer goes by: ``hook.hosts`` and their vessel names (``blastoise-odroid`` ->
    ``blastoise``, the former ssh aliases: blocking more is the safe side), a boat ``$BOAT_AGENT_TARGET``, the hostnames
    and addresses of STATIC_LEASES in the vessel's network env (``network_file``; a checkout without it, such as CI,
    knows no boat addresses, so ``denied`` then treats any private-LAN address as a boat), and ~/.ssh/config
    aliases for any of them."""
    names = set(config['hosts']) | {host.split('-')[0] for host in config['hosts']} | {target_of(config)} - {'local'}
    try:
        leases = re.search(r'^STATIC_LEASES="?([^"\n]*)', network_file().read_text(), re.M).group(1)
        names |= {part for lease in leases.split() for part in lease.split('=')[1:]}
    except (AttributeError, OSError):
        pass
    try:
        aliases = []
        for line in (Path.home() / '.ssh' / 'config').read_text().splitlines():
            words = line.split()
            if words[:1] and words[0].lower() == 'host':
                aliases = words[1:]
            elif words[:1] and words[0].lower() == 'hostname' and len(words) > 1 and (
                    words[1] in names or words[1].split('.')[0] in names):
                names |= {a for a in aliases if not set('*?!') & set(a)}
    except OSError:
        pass
    return names


def denied(command, config):
    """``(what it blocks, the boat it names or None)``, else None. Tier A (``text_only``) is never
    denied; anything else is denied when ``hook.deny`` is found anywhere in its text, or
    ``hook.deny_boat`` when it names a boat (or runs on one)."""
    if text_only(command):
        return None
    text, names = plain(command), boats(config)
    alternatives = [re.escape(n) for n in sorted(names, key=len, reverse=True)]
    if network_file() is None:      # no network env (a checkout without astro_dock): fail closed, any private-LAN address
        alternatives.append(r'(?:192\.168|10|172\.(?:1[6-9]|2\d|3[01]))(?:\.\d{1,3}){2,3}')
    found = re.search(r'(?<![\w.-])(%s)(?![\w-])' % '|'.join(alternatives), text) if alternatives else None
    host = found.group(1) if found else (hostname().split('.')[0] if hostname().split('.')[0] in names else None)
    for pattern in config['deny'] + (config['deny_boat'] if host else []):
        match = re.search(pattern, text, re.S)
        if match:
            rest = re.compile(r'[\n;|&]').search(text, match.end())
            return ' '.join(text[match.start():rest.start() if rest else None].split()), host
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
