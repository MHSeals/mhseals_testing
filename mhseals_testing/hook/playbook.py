"""What the agent reads: its id, the session-start brief, the playbook, and the one-line reason a deny gives with
the boat_agent verb to use instead. Stdlib only."""

import hashlib
import os
import re
import secrets

from . import hosts as _hosts
from .hosts import SIM, boats, remote, target_of
from .rules import NO_CLAIM, PID_ROUTE, UNREAD, hook_config, use_entry


def cli_of(config):
    """How the agent calls boat_agent: this machine's checkout, which makes the ssh hop to a remote target itself
    (``$BOAT_AGENT_TARGET``) and so can say a boat is unreachable with the last operator state it saw."""
    host = remote(config)
    if host and os.environ.get('BOAT_AGENT_TARGET', '').strip() != host:
        return f'BOAT_AGENT_TARGET={host} {config["boat_agent"]}'
    return config['boat_agent']


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
        'or boat_tui; nothing else there. You may end your own '
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
        '`run thrust_physics --set thrust_step=survey`); `set K=V --claim ID` sets any tunable key, within its range, '
        'in its layer: a session setting (amplitude, command_mode, direction, timings), a hardware YAML key '
        "(manual_amplitude, decoupling or one row of it: 'decoupling.yaw=[-0.0375, 0.1, 1.0]', command_mux.ramp), or a live "
        'Nav2 key (nav2.controller_server.FollowPath.vx_max: mhseals_nav config/schema.yaml names them). `set K` reads '
        'it (no claim). A write is validated, snapshotted, read back and logged; its reply has old, value, where and '
        'the revert line. Never tell the operator to change a setting you can set: set it. While armed it refuses the '
        'keys reload.yaml marks armed-unsafe (never amplitude or command_mode); C is live. The TUI shows and journals '
        '"agent ID set K=V". Exit 2: not_allowed (not tunable), bad_args (unknown key or value), busy (a procedure '
        'runs: never mid-segment), refused (armed-unsafe while armed). Every refusal carries the structured ask '
        '(.result.ask: key, current, target, where, why, when, command): relay exactly that.',
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
        '  Every acting verb takes `--why "..."` (`boat_agent VERB --help` marks it required): the TUI agent panel '
        'shows who, verb, why and result. `note --doing "..."` sets your "doing:" '
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
           f': this machine\'s TUI; {boat_session(config)}')))


def boat_session(config):
    """How a boat session names its boat: the registry's first boat computer and its vessel's legacy alias."""
    hosts = boats(config).hosts
    if not hosts:
        return ('a boat session sets BOAT_AGENT_TARGET to its boat computer (the device registry\'s) in your '
                'environment.')
    host = hosts[0]
    vessel = host.split('-')[0]
    alias = f' (`{vessel}` still works for one release; it means {host})' if vessel != host else ''
    return f'a boat session sets BOAT_AGENT_TARGET={host} in your environment{alias}.'


def deny_reason(hit, host=None, who='agent', config=None):
    """One line: the command blocked, why, and the boat_agent verb (or operator action) that does it instead."""
    config = config or hook_config()
    if getattr(hit, 'pattern', None) == UNREAD:
        return (f'Blocked `{hit[:80]}`: the hook cannot read the device registry ($ASTRO_REGISTRY or the profile '
                f"set's devices.yaml), so it cannot tell whether {host} is a boat computer and lets ssh there run only "
                f'boat_agent or boat_tui (fail closed). Use: ssh {host} {config["boat_agent"]} <verb> --who {who}, or '
                f'{cli_of(config)} <verb> --who {who} from here. Ask the owner to fix the registry (hw.sh whoami '
                '--registry names it) or to add the host to hook.ssh_allow.')
    what, verb = use_entry(hit)
    if verb == 'PID' and host:          # on a boat computer nothing is the agent's to kill: stop the boat instead
        what, verb = 'killing processes on a boat computer', 'stop'
    if verb == 'PID':
        return (f'Blocked `{hit[:80]}`: {what}. Instead {PID_ROUTE}. Never kill a boat process: '
                f'{cli_of(config)} stop --who {who} stops the boat.')
    where = f' on {host}' if host else ''
    head = f'Blocked `{hit[:80]}`{where}: {what} from a shell bypasses the boat claim and the operator.'
    here = not host or _hosts.hostname().split('.')[0] == host
    # A bare vessel name (a removed ssh alias) is reached through its first host (the registry's, in role order).
    hosts = boats(config).hosts
    target = host if host in hosts else next((h for h in hosts if h.split('-')[0] == host), host)
    cli = cli_of(config) if here else f'ssh {target} {config["boat_agent"]}'    # the boat it aimed at
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
