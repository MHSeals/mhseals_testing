"""The agent hook (spec agent-control section 6): playbook, raw-motion denies, notes to the TUI.

astro_dock's ``.agents/hook.py`` loads this file (``src/<repo>/<package>/agent_hook.py``) and calls
``agent_event`` for Claude Code and Codex alike with one neutral event (``agent``, ``event``
start|pre|post, ``session_id``, ``tool_id``, ``command``, ``exit``, ``duration_ms``, ``failed``).
Imported under ``python3 -I`` on the dev machine: no ROS. PyYAML, when present, reads
config/agent.yaml (about 15 ms per call); without it the built-in defaults apply.

* ``start`` returns a brief (the agent's boat name, the one rule, where the playbook is); the
  whole playbook (the ``boat_agent`` verbs, exit codes) in a boat session or for ``--playbook``.
* ``pre`` matches what the call runs, never its text: each command of each pipeline (``$(...)``, ``bash -c``,
  wrappers such as sudo or timeout, and the remote command of ``ssh HOST ...`` included) becomes one line, its
  argv joined (``commands``), and each file it writes one ``EDIT op live|dev path`` line. ``hook.deny`` patterns
  match a line from its start; ``hook.deny_boat`` too when the line runs on a boat (this host is one, or ssh to
  one). Heredoc bodies, quoted text and the arguments of grep, echo or git commit are never commands, unless a
  shell, python or ssh reads them on stdin; ``python -c`` counts its obvious literals (``python_runs``).
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
import glob
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import subprocess
import sys
import time
import types

PARTS = ('rules', 'shell', 'runs', 'hosts', 'notes', )      # hook/: one module per concern, re-exported here


def package():
    """hook/ beside this file: ``mhseals_testing.hook`` when imported with the package, else loaded by path under a
    private name (the dispatcher loads this file alone under ``python3 -I``; relative imports in hook/ need a parent)."""
    if __package__:
        return importlib.import_module(__package__ + '.hook')
    here = Path(__file__).resolve().parent / 'hook'
    name = '_agent_hook_' + hashlib.sha1(str(here).encode()).hexdigest()[:12]
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, here / '__init__.py', submodule_search_locations=[str(here)])
        sys.modules[name] = module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except BaseException:
            del sys.modules[name]
            raise
    return sys.modules[name]


_PKG = package()
_MODULES = [importlib.import_module(f'{_PKG.__name__}.{part}') for part in PARTS]
for _module in _MODULES:
    globals().update({k: v for k, v in vars(_module).items() if not k.startswith('__')})


class _Entry(types.ModuleType):
    """Setting a name on this module (a test's monkeypatch of ``hostname`` or ``network_file``) sets it in every hook/
    module that holds the same object, so the code that moved there sees it as it did here."""

    def __setattr__(self, name, value):
        old = self.__dict__.get(name, _Entry)
        for module in _MODULES:
            if old is not _Entry and module.__dict__.get(name, _Entry) is old:
                setattr(module, name, value)
        super().__setattr__(name, value)


if getattr(sys.modules.get(__name__), '__dict__', None) is globals():
    sys.modules[__name__].__class__ = _Entry

BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout


def cli_of(config):
    """How the agent calls boat_agent: this machine's checkout, which makes the ssh hop to a remote target itself
    (``$BOAT_AGENT_TARGET``) and so can say a boat is unreachable with the last operator state it saw."""
    host = remote(config)
    if host and os.environ.get('BOAT_AGENT_TARGET', '').strip() != host:
        return f'BOAT_AGENT_TARGET={host} {BOAT_AGENT}'
    return BOAT_AGENT


def who_of(event):
    """``$BOAT_AGENT``, else the agent CLI (claude, codex) and 4 hex digits: of the hook payload's ``agent_id`` (a
    Claude subagent shares its parent's session, not its agent id), else the first 4 of the session; with neither
    (``--playbook`` from a shell) 4 random ones, never a name another agent shares."""
    explicit = os.environ.get('BOAT_AGENT')
    if explicit:
        return explicit
    sub = str(event.get('agent_id') or '')
    digits = (hashlib.sha256(sub.encode()).hexdigest() if sub else
              re.sub(r'[^0-9a-f]', '', str(event.get('session_id', '')).lower()))[:4]
    return f'{event.get("agent") or "agent"}-{digits or secrets.token_hex(2)}'


# The id an agent calls boat_agent with (``--who X``, ``BOAT_AGENT=X``) is its own choice: the playbook from a shell
# gets 4 random digits, and a subagent's payload id differs from its parent's. The hook remembers the last one each
# agent used (``agent_event``), so a block names the id that holds its claim, not the one ``who_of`` derives.
# Only a boat_agent command that runs counts (``claimed_ids``): an echo, a grep or a commit message naming --who X
# does not, so text cannot pick the id a block names.


def claimed_ids(command):
    """The ids ``command`` calls boat_agent with: ``--who X``/``--who=X`` or a ``BOAT_AGENT=X`` prefix on a command
    whose program is boat_agent (after ``env``, ``nice`` or ``sudo``, or the remote command of ``ssh HOST ...``).
    A word the hook cannot read (``$X``) names nothing."""
    try:
        pipelines = parse(command)
    except ValueError:
        return []
    found = []

    def scan(argv):
        words, wrapped = list(argv), False
        while words and (re.fullmatch(r'\w+=\S*', words[0]) or words[0] in ('env', 'nice', 'sudo', 'exec')
                         or (wrapped and re.fullmatch(r'-\S*|\d+', words[0]))):
            wrapped = wrapped or words[0] in ('env', 'nice', 'sudo', 'exec')
            words = words[1:]
        if words[:1] == ['export'] or (words == [] and argv):        # export BOAT_AGENT=X, or BOAT_AGENT=X alone
            found.extend(w.split('=', 1)[1] for w in argv if w.startswith('BOAT_AGENT=')
                         and re.fullmatch(r'[\w.@:-]+', w.split('=', 1)[1]))
            return
        if words[:1] == ['ssh']:
            rest = [w for w in words[1:] if not w.startswith('-')][1:]
            for pipeline in (parse(' '.join(rest)) if rest else []):
                for cmd in pipeline:
                    scan(cmd['argv'])
            return
        if not words or os.path.basename(words[0]) != 'boat_agent':
            return
        prefix = [w.split('=', 1)[1] for w in argv[:len(argv) - len(words)] if w.startswith('BOAT_AGENT=')]
        given = [w.split('=', 1)[1] for w in words if w.startswith('--who=')]
        given += [b for a, b in zip(words, words[1:]) if a == '--who']
        found.extend(w for w in prefix + given if re.fullmatch(r'[\w.@:-]+', w))
    for pipeline in pipelines:
        for cmd in pipeline:
            try:
                scan(cmd['argv'])
            except ValueError:
                pass
    return found


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
        f'RoboBoat playbook: you are boat agent {who}. Keep this id for the whole session (export BOAT_AGENT={who});'
        ' every agent needs its own (a subagent too): the desk refuses a claim by an id that already holds one, and'
        " a claim works only with the id that made it. Ids are honour-based: never use another agent's --who.",
        f'Act on the boat or its sim only through `{cli}` (JSON out). On a boat host, ssh only to run boat_agent '
        '(claim, arm, run, start, reload, set, fcu, fence, ask, note, release, stop) or boat_tui; nothing else there. You may end your own '
        f'tmux sessions there (name them `agent-{who}-...` and end them with `tmux kill-session -t agent-{who}-...`), never `boat`, '
        "another agent's session or `-L boat`.",
        '  1. `status`: what runs and who drives.',
        '  2. `claim --reason "..."`: ask the operator for the boat.',
        '  3. `run KEY` (`run mission --task NAME`) or `goal X Y [YAW]` with `--claim ID --dry-run`, then for real.',
        '     `arm --claim ID [--countdown S]` arms through the TUI: a banner counts down (agent.yaml arm.countdown, '
        '5 s) and the operator may cancel it; the e-stop refuses it and arming never releases a hold. Sim only: '
        '`run KEY --sim-arm` arms a verified sim (its container sets MHSEALS_SIM_ARM=1) for that run and disarms '
        'after.',
        '     `run KEY --set K=V [--set K=V]` sets what the procedure measures before it starts (e.g. '
        '`run thrust_physics --set thrust_step=survey`); `set K=V --claim ID` sets any tunable key (the testing '
        'config schema; live Nav2 keys such as nav2.controller_server.FollowPath.vx_max too: mhseals_nav config/schema.yaml names them), within its range. '
        'While armed it refuses the keys reload.yaml marks armed-unsafe; never amplitude or command_mode while '
        'armed. The TUI shows and journals "agent ID set K=V". Exit 2: not_allowed (not tunable), bad_args '
        '(unknown key or value), busy (a procedure runs: never mid-segment), refused (armed-unsafe while armed).',
        '     `start COMPONENT --claim ID` / `stop COMPONENT --claim ID` starts or stops a stack part agent.yaml '
        '`start:` lists (odometry, EKFs, sensors, perception) as its TUI Status row does; its row shows your id; '
        '`reload COMPONENT --claim ID` respawns one. Never the driver, mux, Nav2 or watchdogs.',
        '     `reload config [NAME]` re-reads the TUI\'s YAML (hardware, settings, keymap, ...) as a hand edit does. '
        '`run f --claim ID` runs characterization (C applied live, read back).',
        '     `fcu get PARAM...` and `fcu rates` (IMU/odom rates, gap p99, FCU load, "Main loop slow") read only; '
        '`fcu set PARAM VALUE... --claim ID` (agent.yaml `fcu.set`: loop rate, SR0_/SR1_ stream rates, IMU filters; '
        'never yaw source, compass, servos, arming or failsafes), `fcu revert --to SNAPSHOT --claim ID` and '
        '`fcu restart --claim ID` (disarmed) go through the TUI\'s MAVROS link: snapshot, read-back and '
        'fcu_changes.md name you. SCHED_LOOP_RATE 400 only after `fcu rates` at 200 shows the FCU load low.',
        '     `fence corners LAT,LON LAT,LON LAT,LON ... --why "..."`, `fence on_exit alert|return --why "..."` and '
        '`fence anchor --why "..."` set fence_watchdog\'s course area at any time, no claim: validated, read back, '
        'kept in ~/.config/mhseals/fence.yaml and shown in the agent panel.',
        '     Start sims and heavy runs through the host guard: `.handoff/tools/heavy sim ...` (Unity headless).',
        '     No TUI (exit 4): `boat_tui up` (beside boat_agent) starts the stack and TUI in tmux and prints the line '
        'the operator attaches with; `boat_tui status` says who controls. Never type into its tmux session.',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  Every acting verb (claim, arm, run, goal, set, start, stop, reload, fcu set/restart, release) takes '
        '`--why "..."`: the TUI agent panel shows who, verb, why and result. `note --doing "..."` sets your "doing:" '
        'line there; `ask "..."` instead of guessing. `stop` any time: the operator and the agent it interrupts see '
        'your id; it cancels an arm countdown and keeps your claim (`release` it to claim again). `status` and '
        '`fcu get` work with no TUI controller.',
        '  `release HOLD --why "..."` (hold names in `status` .authority.record.holds) releases a lease or watchdog hold once its fault '
        'has cleared, at once; an operator hold (Space, stop, manual) only when ~/.config/mhseals/agent_grants.yaml '
        'grants it to you (then at once, e.g. a known bug the owner told you to clear). The e-stop is never '
        'releasable. When it is the best course of action and allowed, act yourself; say why.',
        'Exit codes: 0 ok, 2 refused (read hint), 3 revoked or overridden (the hint says who: '
        'the operator or another agent; claim again only if asked), 4 no TUI (`unreachable`: the boat is off or '
        'out of reach), 5 agents blocked, 6 timeout, 7 paused by the operator.',
        'Operator state (`status` .operator; the operator sets it with S, manual control sets manual): '
        '`manual` (the operator drives) or `battery_swap`/`power_cycle` (the boat goes down; ETA in the hint): '
        'changing verbs exit 7 and your claim lease is suspended, not lapsed. Pause: no retry loop, one '
        '`follow --claim ID --until operator_state:state=normal` keeps the claim and returns on `normal`, then '
        'carry on (a claim not kept alive during the pause ends cleanly: claim again). Exit 4 with a down state or '
        '`unreachable`: the boat is down; check `status` every few minutes. When status shows `restarted_after`, '
        're-check the stack, FCU params and RTK before anything else, and claim again (old claims are gone).',
        'Never publish /cmd_vel, /goal_pose, mux or MAVROS commands, change Nav2 lifecycles (params: '
        '`BOAT_AGENT_CLAIM=ID ros2 param set ...` or `set`), send raw test_pulses, start the mission runner or a '
        'procedure directly, never arm but through `arm`, release holds only with `release`: '
        'the hook blocks them and names the verb; kill only your own processes by exact PID; never answer a '
        'TUI popup. Run in sim first. For runs over 9 min use '
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands'
        + (f' ({remote(config)}).' if remote(config) else
           ' (the sim TUI on this machine, BOAT_AGENT_TARGET=sim).' if target_of(config) == SIM else
           ': this machine\'s TUI; a boat session sets BOAT_AGENT_TARGET=blastoise-odroid in your environment '
           '(`blastoise` still works for one release; it means blastoise-odroid).')))


def deny_reason(hit, host=None, who='agent', config=None):
    """One line: the command blocked, why, and the boat_agent verb (or operator action) that does it instead."""
    config = config or hook_config()
    what, verb = use_entry(hit)
    if verb == 'PID' and host:          # on a boat computer nothing is the agent's to kill: stop the boat instead
        what, verb = 'killing processes on a boat computer', 'stop'
    if verb == 'PID':
        return (f'Blocked `{hit[:80]}`: {what}. Instead {PID_ROUTE}. Never kill a boat process: '
                f'{cli_of(config)} stop --who {who} stops the boat.')
    where = f' on {host}' if host else ''
    head = f'Blocked `{hit[:80]}`{where}: {what} from a shell bypasses the boat claim and the operator.'
    here = not host or hostname().split('.')[0] == host
    # A bare vessel name (blastoise: a removed ssh alias) is reached through its first host.
    target = host if host in config['hosts'] else next(
        (h for h in config['hosts'] if h.split('-')[0] == host), host)
    cli = cli_of(config) if here else f'ssh {target} {BOAT_AGENT}'    # the boat it aimed at
    if verb is None:
        return f'{head} Those stay with the operator; ask them: {cli} ask "..." --who {who}'
    arms = ''
    if verb.startswith('arm '):
        arms = (' The TUI shows a countdown the operator can cancel; the e-stop refuses it. '
                'Then run KEY --claim ID.')
    return (f'{head} Use: {cli} {verb} --who {who}'
            + ('' if verb.startswith(NO_CLAIM) and '--claim' not in verb else f' (claim first: claim --reason "..." --who {who})')
            + f'.{arms} Verbs and exit codes: BOAT_AGENT={who} python3 .agents/hook.py --playbook')


def use_of(hit):
    """The short route a deny names, for the TUI row."""
    verb = use_entry(hit)[1]
    if verb == 'PID':
        verb = 'stop' if getattr(hit, 'host', None) else None
        if verb is None:
            return 'kill PID'
    return f'boat_agent {verb.split()[0]}' if verb else 'operator'


# A file is protected only by name (the deny rules), so only these names need a place (a git call for a worktree).
PROTECTED = re.compile(r'agent/stop|command_mux[^/]*\.json|hold_store|agent_hook\.py|/hook\.py|\.claude/settings'
                       r'|\.codex/hooks\.json|agent\.ya?ml|launch|sysrq')


def edit_lines(kind, given, base=None, remote=False):
    """``EDIT op where path`` for a file written (``write``) or removed (``delete``): its absolute path and, when it
    differs, its symlink target, a glob expanded. A remote or unresolvable name stays as given (``./`` before a
    relative one), live."""
    if base and (not os.path.isabs(base) or '$' in base) and not os.path.isabs(os.path.expanduser(given)):
        given, base = os.path.join(base, given), None           # after a cd the hook cannot resolve
    if remote or '$' in given or (not os.path.isabs(os.path.expanduser(given)) and base is None):
        return [f'EDIT {"delete" if kind == "delete" else "change"} live '
                f'{given if given.startswith(("/", "~")) else "./" + given}']
    path = os.path.join(base or '/', os.path.expanduser(given))
    out = []
    for each in (glob.glob(path) if re.search(r'[*?\[]', path) else []) or [path]:
        for full in dict.fromkeys([os.path.abspath(each), os.path.realpath(each)]):
            op = 'delete' if kind == 'delete' else 'change' if os.path.lexists(full) else 'create'
            out.append(f'EDIT {op} {place(full) if PROTECTED.search(full) else "live"} {full}')
    return out


class Scan:
    """``(text, line, host, claim)`` for each command a call runs and each file it writes (``lines``): ``text`` the
    command as the agent wrote it, ``line`` what the deny rules match, ``host`` the boat it runs on or None,
    ``claim`` a ``BOAT_AGENT_CLAIM=`` the command carries."""

    def __init__(self, names, cwd=None):
        self.names, self.cwd, self.out = names, cwd, []

    def text(self, command, host=None, remote=False, depth=0):
        try:
            pipelines = parse(command)
        except ValueError:                  # an open quote or an odd heredoc: each line's words, quotes dropped
            pipelines = [[{'argv': part.split(), 'writes': [], 'subs': []}]
                         for part in re.split(r'[;&|\n]+', plain(command)) if part.split()]
        self.pipelines(pipelines, host, remote, depth, self.cwd)

    def pipelines(self, pipelines, host, remote, depth, base):
        for cmd, before in ((c, p[k - 1] if k else None) for p in pipelines for k, c in enumerate(p)):
            argv = cmd['argv']
            if argv and depth <= 8:
                self.stdin(cmd, before, host, remote, depth, base)
            if argv[:1] in (['cd'], ['pushd']) and not remote:     # an unknown base stays a relative name
                to = os.path.expanduser(argv[1] if len(argv) > 1 else '~')
                known = base is not None and os.path.isabs(base) and '$' not in base
                base = (None if to == '-' else os.path.normpath(to) if os.path.isabs(to)
                        else os.path.normpath(os.path.join(base, to)) if known else os.path.join(base or '', to))
            for sub in cmd.get('subs', []):
                self.pipelines(sub, host, remote, depth + 1, base)
            text = ' '.join(argv)
            if argv:
                self.argv(argv, text, host, remote, depth, base)
            for target in cmd.get('writes', []):
                self.out += [(text, line, host, None) for line in edit_lines('write', target, base, remote)]

    def stdin(self, cmd, before, host, remote, depth, base):
        """A program a shell, python or ssh reads on stdin runs too: a heredoc or ``<<<`` body, or what an echo,
        printf or ``cat <<EOF`` before it in the pipeline prints."""
        bodies = list(cmd.get('stdin', []))
        feeder = core(before['argv']) if before else []
        if feeder[:1] in (['echo'], ['printf']):
            bodies.append(' '.join(w for w in feeder[1:] if not re.fullmatch(r'-[neE]+', w)).replace('\\n', '\n'))
        elif feeder[:1] == ['cat'] and before.get('stdin'):
            bodies += before['stdin']
        program = core(cmd['argv'])
        if not bodies or not program:
            return
        name, text = os.path.basename(program[0]), ' '.join(cmd['argv'])
        if name in SSH:
            _, command, target = runs(program)
            there = core(command.split()) if command else []
            if target is not None and (not there or os.path.basename(there[0]) in SHELLS and reads_stdin(there)):
                for body in bodies:
                    self.text(body, is_boat(target, self.names) or host, True, depth + 1)
        elif name in SHELLS and reads_stdin(program):
            for body in bodies:
                self.text(body, host, remote, depth + 1)
        elif PYTHON.match(name) and reads_stdin(program):
            for body in bodies:
                self.python(body, text, host, remote, depth, base)

    def python(self, code, text, host, remote, depth, base, claim=None):
        argvs, texts, files = python_runs(code)
        for argv in argvs:
            self.argv(argv, text, host, remote, depth + 1, base, claim)
        for command in texts:
            self.text(command, host, remote, depth + 1)
        for kind, path in files:
            self.out += [(text, line, host, claim) for line in edit_lines(kind, path, base, remote)]

    def argv(self, argv, text, host, remote, depth, base, claim=None):
        if depth > 8 or not argv:
            return
        count = 0
        while count < len(argv) and ASSIGN.fullmatch(argv[count]):
            count += 1
        lead = argv[:count]
        claim = next((w.split('=', 1)[1] for w in lead if w.startswith('BOAT_AGENT_CLAIM=')), claim)
        words = argv[len(lead):]
        self.out.append((text, ' '.join(lead + [os.path.basename(words[0])] + words[1:]) if words else ' '.join(lead),
                         host, claim))
        if not words:
            return
        if lead:
            self.out.append((text, ' '.join([os.path.basename(words[0])] + words[1:]), host, claim))
        for kind, path in written(words):
            self.out += [(text, line, host, claim) for line in edit_lines(kind, path, base, remote)]
        if PYTHON.match(os.path.basename(words[0])):
            flag = next((i for i, w in enumerate(words[1:], 1) if re.fullmatch(r'-[A-Za-z]*[cm]', w)), None)
            if flag is not None and words[flag].endswith('c') and flag + 1 < len(words):
                self.python(words[flag + 1], text, host, remote, depth, base, claim)
        inner, command, target = runs(words)
        if target is not None:
            host, remote = is_boat(target, self.names) or host, True
        if inner:
            self.argv(inner, text, host, remote, depth + 1, base, claim)
        if command:
            self.text(command, host, remote, depth + 1)


def lines(command, config, cwd=None):
    """Every ``(text, line, host, claim)`` of ``command`` (``Scan``); on a boat host every line runs here."""
    names = boats(config)
    here = hostname().split('.')[0]
    scan = Scan(names, cwd)
    scan.text(command, here if here in names else None)
    return scan.out


def denied(command, config, cwd=None, who=None):
    """``(Hit, the boat it runs on or None)`` for the first line of ``command`` a deny rule matches, else None."""
    for text, line, host, claim in lines(command, config, cwd):
        if host and who and re.match(OWN_TMUX % re.escape(who), line):
            continue
        for pattern in config['deny'] + (config['deny_boat'] if host else []):
            if re.match(pattern, line, re.S) and not (claim and pattern == NAV2_PARAM):
                return Hit(' '.join(text.split()), pattern, ' '.join(line.split()), host), host
    return None


def place(path):
    """``'dev'`` when ``path`` is in a linked git worktree (``git worktree add``: its .git file names a
    ``.../worktrees/...`` git dir) and not ignored there, else ``'live'``: the main checkouts the TUI and the stack
    read, install/ and build/, the data and runtime dirs, anything outside git. Any doubt is ``'live'``."""
    root = next((d for d in (Path(path), *Path(path).parents) if (d / '.git').exists()), None)
    try:
        gitdir = (root / '.git').read_text().strip() if root and (root / '.git').is_file() else ''
        if not re.match(r'gitdir: .*/worktrees/[^/]+$', gitdir):
            return 'live'
        done = subprocess.run(['git', '-C', str(root), 'check-ignore', '-q', '--no-index', str(path)],
                              stdin=subprocess.DEVNULL, capture_output=True, timeout=1)
    except Exception:
        return 'live'
    return 'dev' if done.returncode == 1 else 'live'


def edit_text(edits, cwd=None):
    """The ``EDIT op where path`` lines of an edit tool's ``[[write|delete, path]]`` (the path as given and,
    when it differs, its symlink target), or None when a path is missing (fail closed)."""
    lines = []
    for kind, given in edits or ():
        if not given or kind not in ('write', 'delete'):
            return None
        path = Path(os.path.expanduser(str(given)))
        path = path if path.is_absolute() else Path(cwd or os.getcwd()) / path
        for each in dict.fromkeys([os.path.abspath(path), os.path.realpath(path)]):
            op = 'delete' if kind == 'delete' else 'change' if os.path.lexists(each) else 'create'
            lines.append(f'EDIT {op} {place(each)} {each}')
    return '\n'.join(lines) or None


def edit_denied(event, config):
    """The deny reason for an edit tool's files (``event['edits']``), else None; on a boat host ``deny_boat`` too.
    Fails closed: a missing path or any error denies."""
    who = who_of(event)
    try:
        text = edit_text(event.get('edits'), event.get('cwd'))
        if text is None:
            return f'Blocked an edit whose file the hook cannot read (fail closed). Name the file. --who {who}'
        boat = hostname().split('.')[0] in boats(config)
        for line in text.split('\n'):
            for pattern in config['deny'] + (config['deny_boat'] if boat else []):
                if pattern.startswith('EDIT') and re.match(pattern, line):
                    why = next(w for p, w in EDIT_USE if re.search(p, line)).format(cli=cli_of(config))
                    return f'Blocked editing `{line.split(" ", 3)[3]}`: {why} (--who {who}). Reads stay allowed.'
    except Exception as error:
        return f'Blocked an edit the hook could not check ({error!r}; fail closed). --who {who}'
    return None


def deps_notice():
    """The workspace's missing host tools (devices/deps.sh check: declared in the repos' deps.yaml), '' when complete."""
    script = Path(__file__).resolve().parents[3] / 'devices' / 'deps.sh'
    try:
        done = subprocess.run([str(script), 'check'], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return ''
    return done.stderr.strip()


def agent_event(event, config=None, send=None, now=None, state_dir=None):
    """The answer to one neutral event (None, ``{'context'}`` or ``{'deny'}``); sends at most one note."""
    phase, agent = event.get('event'), event.get('agent') or 'agent'
    who = who_of(event)
    if phase not in ('start', 'pre', 'post'):
        return None
    config = config or hook_config()
    if phase == 'start':
        full = event.get('source') == 'playbook' or target_of(config) != 'local'
        text = start_text(who, agent, config) if full else brief_text(who, config)
        notice = deps_notice()
        return {'context': text + ('\n\n' + notice + '\nTell the owner; do not run sudo yourself.' if notice else '')}
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    command = str(event.get('command') or '')
    answer = blocked = None
    if phase == 'pre' and 'edits' in event:     # an edit tool, or a shell apply_patch: its files first
        reason = edit_denied(event, config)
        if reason:
            return {'deny': reason}
    if phase == 'pre':
        blocked = denied(command, config, event.get('cwd'), who)
    named = claimed_ids(command) if phase == 'pre' else []
    if named or blocked:
        with State(str(event.get('session_id') or 'none'), state_dir) as state:
            ids = state.data.setdefault('who', {})
            key = str(event.get('agent_id') or '')
            if named:
                ids[key] = named[-1]
            who = ids.get(key, who)
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
