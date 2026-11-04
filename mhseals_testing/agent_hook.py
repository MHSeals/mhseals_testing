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
import json
import os
from pathlib import Path
import hashlib
import re
import secrets
import shlex
import subprocess
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
# Every pattern matches one line from its start (``re.match``): a command's argv joined by spaces, its program
# by base name (``ros2 topic pub /cmd_vel ...``), or ``EDIT create|change|delete live|dev PATH`` for a file it writes.
# What starts the thruster driver, the mux or the velocity controller: executables and their modules.
DRIVER = r'(thruster_pwm_node|command_mux|calibrated_velocity_controller|thruster_test|thruster_identify)'
MODULE = r'(thruster_pwm_node|command_mux|velocity_controller|identification|thruster_test)'


def ros2_run(package, executable):
    """ros2 run PKG ... EXE: any flags or --prefix value between the package and the executable."""
    return r'ros2\s+run\b.*?\s' + package + r'\s+(\S+\s+)*?' + executable + r'\b'


def started(names):
    """``names`` (executables) run directly (the program, after wrappers, by base name) or as a python script."""
    return names + r'(\.py)?(\s|$)|python[\d.]*\s+(-\S+\s+)*(\S*/)?' + names + r'(\.py)?(\s|$)'


# Starting the thruster driver, the mux or the velocity controller (arming) in any form: ros2 run, python -m the
# module, the executable or its script, a launch that starts the mux or a launch file by path.
ARMING = [ros2_run('mhseals_hardware', DRIVER), r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_hardware\.' + MODULE + r'\b',
          started(DRIVER), r'ros2\s+launch\b.*\bcommand_mux:=(?!false\b|False\b)',
          r'ros2\s+launch\s+(--?\S+\s+)*\S*mhseals_hardware\b', r'ros2\s+launch\s+(--?\S+\s+)*\S*(/|\.launch\.|\.py\b|\.xml\b|\.ya?ml\b)']
# Files no shell or edit tool writes, only live copies (a linked git worktree is development): the mux's saved
# holds (command_mux*.json, hold_store), changing or removing the agent stop file (creating or touching it stops
# the boat: fine), and the agent hook and its wiring (this file, .agents/hook.py, the agent-loop repo's
# src/*/hooks/hook.py, .claude/settings*.json, .codex/hooks.json).
HOLDS = [r'EDIT \w+ live .*(/command_mux[^/]*(\.json|\*)|/hold_store(/.*)?|/hold_store\.json)$']
STOP = [r'EDIT (change|delete) live .*/agent/stop$']
HOOKS = [r'EDIT \w+ live .*/(agent_hook\.py|\.agents/hook\.py|src/[^/]+/hooks/hook\.py|\.claude/settings(\.local)?\.json'
         r'|\.codex/hooks\.json)$']
# On a boat computer only: the runtime config and launch files the running boat reads.
BOAT_FILES = [r'EDIT \w+ live .*(/agent\.ya?ml|/launch/[^/]+\.(py|xml|ya?ml)|[^/]\.launch\.(py|xml|ya?ml))$']
# Nav2's servers: a lifecycle change breaks a claimed goal that is running. ``ros2 param`` on them is allowed with
# a claim (``BOAT_AGENT_CLAIM=ID ros2 param set ...``; boat_agent set does it for the live keys).
NAV2 = (r'(controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother'
        r'|collision_monitor|waypoint_follower|route_server|docking_server|lifecycle_manager\w*)')
NAV2_PARAM = r'ros2\s+param\s+(set|load|delete)\b.*\b' + NAV2 + r'\b'
# On a boat: a stack component started or stopped from a shell (any mhseals_nav/mhseals_hardware launch: the
# umbrella device launch or one part's, e.g. odom.launch.py; or its mhseals-<part> unit)
# instead of boat_agent start/stop COMPONENT, which goes through the TUI's Status row path under the claim.
BOAT_START = [r'ros2\s+launch\s+(--?\S+\s+)*mhseals_(nav|hardware)\b',
              r'systemctl\s+(--user\s+)?(start|restart)\s+(\S+\s+)*mhseals-']
BOAT_STOP_PART = [r'systemctl\s+(--user\s+)?(stop|kill)\s+(\S+\s+)*mhseals-']
BOAT_KILL = r'(pkill|killall|kill)\b|tmux\b.*\skill-(server|session|window|pane)\b'
# git with a hooks path of the caller's choosing skips the commit-msg hook (no AI attribution): set for the call,
# by git config or through the environment. Reading it stays allowed.
HOOKS_PATH = [r'(?i)git\b(?!.*\sconfig\s+(--get\S*|--list|-l)\s).*\bcore\.hookspath\b',
              r'(?i)\w+=.*\bcore\.hookspath\b']
# Typing into the boat's boat_test TUI: tools/boat_tui runs it on its own tmux server (tmux -L boat) on a boat host;
# keys sent to any of its panes would answer the operator's popups, arm or release a hold. A deny_boat rule:
# private sockets (-L/-S) for sim and dev stay free. Watching it (capture-pane, list, status) stays allowed.
TUI_KEYS = [r'''tmux\b.*(-L\s*boat\b|-S\s*\S*/tmux-\d+/boat\b).*\s(send-keys|send|paste-buffer|pasteb|load-buffer'''
            r'|loadb|pipe-pane|pipep)(\s|$)']
# The owner's human preview (tools/boat_preview, owner 11-01): agents never start, stop or type into it. Allowed: status,
# and an integration test's own preview, --test with --headless (or down/attach of it), which shares nothing with it.
_TEST_PREVIEW = r'(?!(?=.*\s--test\b)(?=.*\s(--headless|down|attach)\b))'
PREVIEW = [r'boat_tui\b(?=.*\spreview(\s|$))(?!.*\spreview\s+status\b)' + _TEST_PREVIEW,
           r'boat_preview\b(?!\s+status\b)' + _TEST_PREVIEW,
           r'tmux\b.*-L\s*preview(?=\s|$).*\s(send-keys|send|paste-buffer|pasteb|load-buffer|loadb|respawn-\w+'
           r'|kill-(server|session|window|pane))(\s|$)',
           r'(docker|podman)\b.*\s(stop|kill|restart|rm|pause)\b.*\broboboat_preview\b']
# Killing the boat's control: a kill (or pkill) whose arguments name it, $(pgrep ...) included. kill PID is fine.
_CONTROL = (r'\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller|mavros\w*'
            r'|boat_(trial|test)\w*)\b')
KILL_CONTROL = r'(kill|pkill|killall)\b.*' + _CONTROL
# fence_watchdog's course area: its corners and on_exit are already a watchdog parameter (denied above); /fence/anchor
# too. boat_agent fence sets them through the TUI, validated, read back and logged (D-20261101-23).
FENCE_CALL = r'ros2\s+service\s+call\b.*/fence/anchor\b'
FENCE = r'fence_watchdog\b.*\b(course\.corners|on_exit)\b|fence_watchdog/set_parameters|/fence/anchor\b'
NO_CLAIM = ('stop', 'fence ')      # verbs a deny names that need no claim first (stop COMPONENT --claim ID does)
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
             ros2_run('mavros', r'mav\w*'), r'ros2\s+run\b.*\steleop_twist_\w+\b',
             ros2_run('mhseals_hardware', 'keyboard_control'),
             ros2_run('mhseals_testing', 'operator_heartbeat')
             + r'|python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.ros\.operator_heartbeat\b',
             started('operator_heartbeat'),
             r'ros2\s+(topic\s+pub|service\s+call)\b.*\bthruster_pwm_node/test_pulses\b',
             NAV2_PARAM, r'ros2\s+lifecycle\s+set\b.*\b' + NAV2 + r'\b',
             r'ros2\s+service\s+call\b.*\b' + NAV2 + r'/(set_parameters\w*|load_parameters|change_state|manage_nodes)\b',
             FENCE_CALL, ros2_run('mhseals_nav', 'run_task'),
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_nav\.tasks\b',
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.(procedures|runner)\b',
             r'ros2\s+(topic\s+pub|service\s+call)\b.*(\boperator/heartbeat|\bmhseals_testing/authority'
             r'|\bthruster_pwm_node/estop|\bthrusters/)',
             r'ros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)|lifecycle\s+set|run)'
             r'\s+(--?[\w-]+\s+)*[^\s-]*\$',
             r'xargs\b.*\sros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)'
             r'|lifecycle\s+set|run)\b',
             r'(pkill|killall)\b', r'xargs\b.*\s(sudo\s+)?kill\b', KILL_CONTROL]
            + ARMING + HOLDS + STOP + HOOKS + HOOKS_PATH + PREVIEW,
    'hosts': ['blastoise-odroid', 'blastoise-jetson', 'squirtle-jetson'],
    'deny_boat': BOAT_START + BOAT_STOP_PART +
                 [BOAT_KILL, r'(docker|podman)\b.*\s(stop|kill|restart|rm|down|pause)\b',
                  r'systemctl\s+(--user\s+)?(stop|kill|restart|isolate|reboot|poweroff|halt)\b',
                  r'(reboot|poweroff|halt|shutdown)\b', r'(tel)?init\s+[0-6sS]\b',
                  r'loginctl\s+(terminate|kill)', r'EDIT \w+ \w+ .*sysrq'] + TUI_KEYS + BOAT_FILES}
NETWORK = 'config/boat-network.env'    # legacy path; STATIC_LEASES "MAC=hostname=address ...": the boat computers
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the route that does it instead: (regex on the matched line or None, what it
# blocks, the boat_agent verb (None: the operator's; PID: kill your own process by PID), the deny patterns it owns).
# The deny pattern that matched picks its entry (``use_entry``); patterns no entry owns fall back to the regex.
_DENY = DEFAULTS['deny']
USE = ((r'(pkill|killall|kill)\b.*' + _CONTROL, "killing the boat's control (the mux, a watchdog, the "
        'driver, MAVROS or a trial)', 'stop', (KILL_CONTROL,)),
       (None, 'pkill, killall and xargs kill (they can hit processes that are not yours)', 'PID',
        (r'(pkill|killall)\b', r'xargs\b.*\s(sudo\s+)?kill\b')),
       (None, 'starting a stack component on a boat (its device launch or unit)', 'start COMPONENT --claim ID',
        tuple(BOAT_START)),
       (None, 'stopping a stack component on a boat (its unit)', 'stop COMPONENT --claim ID', tuple(BOAT_STOP_PART)),
       (None, 'killing processes on a boat computer (a stack component: stop COMPONENT --claim ID; your own tmux '
        'session: tmux kill-session -t agent-<your id>...)', 'stop', (BOAT_KILL,)),
       (None, "typing into the operator's boat_test TUI (its tmux session): popups, arming and holds are the "
        "operator's; ask with ask \"...\", watch with status or tmux capture-pane", None, tuple(TUI_KEYS)),
       (None, "changing the running boat's config or launch files on a boat computer (develop them in a git "
        'worktree; hot changes: set K=V --claim ID or reload config)', None, tuple(BOAT_FILES)),
       (None, "stopping or restarting the boat's software or computer (the FCU: boat_agent fcu restart --claim ID)",
        None, tuple(p for p in DEFAULTS['deny_boat']
                    if p not in BOAT_START + BOAT_STOP_PART + [BOAT_KILL] + TUI_KEYS + BOAT_FILES)),
       (None, 'replaying a bag that can carry motion topics (play only --topics without cmd_vel, thruster or mux '
        'topics)', 'run KEY --claim ID', tuple(p for p in _DENY if p.startswith(r'ros2\s+bag\s+play'))),
       (None, "changing the mux's live saved holds (release a lease or watchdog hold whose fault cleared with "
        'release HOLD --why "..."; operator holds need a grant)', 'release HOLD --why "..."', tuple(HOLDS)),
       (None, 'removing or changing the live agent stop file (to stop, touch it or use stop; only the operator '
        'removes it)', None, tuple(STOP)),
       (None, 'changing the live agent hook or its wiring (it keeps agents on boat_agent; develop it in a git worktree '
        'and have the operator land it)', None, tuple(HOOKS)),
       (None, "the owner's preview (boat_tui preview: start, stop or keys; it is theirs, watch with boat_tui preview "
        'status). A test run of your own: boat_tui preview --headless --test, then boat_tui preview down --test', None,
        tuple(PREVIEW)),
       (None, 'git with a hooks path of its own (it skips the commit-msg hook; plain git, -c and GIT_* are fine)', None,
        tuple(HOOKS_PATH)),
       (None, 'arming (starting the thruster driver or the mux)', 'arm --claim ID', tuple(ARMING)),
       (None, "changing Nav2's servers (a lifecycle change breaks a running claimed goal; parameters: "
        'BOAT_AGENT_CLAIM=ID ros2 param set ..., or set K=V --claim ID)', 'set K=V --claim ID',
        tuple(p for p in _DENY if NAV2 in p)),
       (None, 'starting the mission runner directly (outside the claim)', 'run mission --task NAME --claim ID',
        tuple(p for p in _DENY if 'run_task' in p or r'mhseals_nav\.tasks' in p)),
       (None, 'starting a test procedure directly (outside the claim)', 'run KEY --claim ID',
        tuple(p for p in _DENY if r'mhseals_testing\.(procedures' in p)),
       (None, "raw thruster pulses (thruster_pwm_node's test_pulses)", 'run KEY --claim ID',
        tuple(p for p in _DENY if 'test_pulses' in p)),
       (r'cmd_vel|thrusters/|mavros/(rc/override|setpoint|manual_control)|teleop|keyboard_control',
        'raw motion', 'run KEY --claim ID', ()),
       (r'operator/heartbeat|operator_heartbeat|mhseals_testing/authority', "the operator's heartbeat or authority",
        None, ()),
       (r'estop', 'the thruster e-stop', 'stop', ()),
       (FENCE, "fence_watchdog's course area (corners, on_exit, anchor)",
        'fence corners LAT,LON LAT,LON LAT,LON ... | on_exit alert|return | anchor --why "..."', (FENCE_CALL,)),
       (r'mavros/param/set\b|param\s+set\b.*\bmavros', 'an FCU parameter change',
        'fcu set PARAM VALUE --claim ID', ()),
       (r'mavros', 'an FCU arming or mode change (parameters: boat_agent fcu set; an FCU restart: boat_agent fcu '
        'restart)', None, ()),
       (r'\$', 'a ros2 command whose target is a shell variable the hook cannot read (spell the name out)', None, ()),
       (r'ros2\s+(param|lifecycle)\b|/(set_parameters\w*|load_parameters|change_state)\b',
        'changing the thruster driver, the mux or a safety watchdog', None, ()),
       (r'command_mux', 'commanding the mux', 'stop', ()),
       (r'goal_pose|navigate|follow_\w*waypoints|follow_path|dock_robot|assisted_teleop', 'a raw Nav2 goal',
        'goal X Y [YAW] --claim ID', ()),
       (r'.', 'raw motion', 'run KEY --claim ID', ()))
PID_ROUTE = 'kill your own process by exact PID: kill PID (pgrep -af NAME lists them) or kill %JOB'


class Hit(str):
    """A denied command's text (``str``: the command that runs it), with the deny ``pattern`` that matched, ``tail``
    (the matched line, which names the reason) and ``host``."""
    def __new__(cls, text, pattern=None, tail=None, host=None):
        hit = super().__new__(cls, text)
        hit.pattern, hit.tail, hit.host = pattern, text if tail is None else tail, host
        return hit


def use_entry(hit):
    """``(what, verb)`` for a denied ``hit``: the first entry owning its pattern or whose regex finds its tail."""
    pattern, tail = getattr(hit, 'pattern', None), getattr(hit, 'tail', hit)
    return next((w, v) for r, w, v, owned in USE
                if (pattern is not None and pattern in owned) or (r is not None and re.search(r, tail)))


# Why an edit is denied, first match on the EDIT line wins.
EDIT_USE = ((r'command_mux|hold_store', "the mux's live saved holds. Release a lease or watchdog hold whose fault "
             'cleared with {cli} release HOLD --why "..."; an operator hold needs a grant (agent_grants.yaml)'),
            (r'/agent/stop$', 'the live agent stop file. To stop, create or touch it or use {cli} stop; only the '
             'operator removes it'),
            (r'/(agent_hook\.py|hook\.py|settings(\.local)?\.json|hooks\.json)$', 'the live agent hook or its '
             'wiring (it keeps agents on boat_agent). Develop it in a git worktree and have the operator land it'),
            (r'.', "the running boat's config or launch files on a boat computer. Develop them in a git worktree; hot "
             'changes: {cli} set K=V --claim ID or reload config'))


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


SIM = 'sim'          # $BOAT_AGENT_TARGET=sim: tools/boat_tui sim's TUI on this machine (its container), never a host


def remote(config):
    """The target host when it is another machine, else None (the sim runs here)."""
    target = target_of(config)
    return None if target in ('local', SIM) or hostname().split('.')[0] == target else target


def sim_channel():
    """The sim TUI's channel on this machine: config/sim/stack.yaml ``overlay`` (``$BOAT_SIM_OVERLAY`` wins), mounted
    as the container's /home/roboboat/autodemo, so its data/agent/channel.jsonl; read without PyYAML."""
    overlay = os.environ.get('BOAT_SIM_OVERLAY')
    if not overlay:
        try:
            text = (Path(__file__).resolve().parents[1] / 'config' / 'sim' / 'stack.yaml').read_text()
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


def plain(text):
    """``text`` with continuations joined and quotes and escapes dropped (the note filter, ``matches``)."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '').replace('\\', '')


def matches(patterns, text):
    """Any pattern found in ``plain(text)``; ``.`` spans newlines (the TUI-note filter)."""
    text = plain(text)
    return any(re.search(p, text, re.S) for p in patterns)


SEPARATORS = {';', '&', '&&', '||', '|', '|&', '\n', ';;', '(', ')'}
REDIRECTS = {'<', '>', '>>', '>|', '&>', '&>>', '>&', '<&', '<>'}
PUNCT = re.compile(r'<<<|<<|>>|>&|<&|<>|&>>|&>|>\||\|\||&&|\|&|;;|[\s\S]')
HEREDOC_OP = re.compile(r'(?<!<)<<(?!<)(-?)[ \t]*')
FIND_RUNS = ('-exec', '-execdir', '-ok', '-okdir', '-delete')


def delimiter(line, start):
    """The heredoc word at ``line[start:]`` as bash reduces it: quotes and backslashes removed, and any of them
    makes the body literal. ``(word, quoted)``; ValueError for a word the hook will not guess at."""
    word, quoted, i = '', False, start
    while i < len(line) and line[i] not in ' \t;&|()<>':
        c = line[i]
        if c in '\'"':
            end = line.find(c, i + 1)
            if end < 0 or (c == '"' and '\\' in line[i + 1:end]):
                raise ValueError('heredoc delimiter')
            word, quoted, i = word + line[i + 1:end], True, end + 1
        elif c == '\\':
            if i + 1 >= len(line):
                raise ValueError('heredoc delimiter')
            word, quoted, i = word + line[i + 1], True, i + 2
        else:
            word, i = word + c, i + 1
    if not word or re.search(r'[$`]', word):
        raise ValueError('heredoc delimiter')
    return word, quoted


def heredocs(text):
    """``(lines, bodies)``: the command lines of ``text`` (backslash-newline joined) with each heredoc body taken
    out, and ``bodies`` ``[[dash, word, quoted, body, line index, terminator]]``. ValueError on any doubt."""
    raw, lines, found, i = text.split('\n'), [], [], 0
    while i < len(raw):
        line = raw[i]
        i += 1
        while line.endswith('\\') and i < len(raw):
            line, i = line[:-1] + raw[i], i + 1
        lines.append(line)
        for op in HEREDOC_OP.finditer(line):
            word, quoted = delimiter(line, op.end())
            dash, body = bool(op.group(1)), []
            while i < len(raw) and (raw[i].lstrip('\t') if dash else raw[i]) != word:
                body.append(raw[i])
                i += 1
            if i >= len(raw):
                raise ValueError('unterminated heredoc')
            found.append([dash, word, quoted, '\n'.join(body), len(lines) - 1, raw[i]])
            i += 1
    return lines, found


def tokens_of(text):
    """Shell words and operators; a quoted word stays one word and ``\\;`` a word, not a separator (find -exec,
    tmux). Raises ValueError on an open quote."""
    lexer = shlex.shlex(text.replace('\\;', '\x01'), posix=True, punctuation_chars=';&|()<>`\n')
    lexer.whitespace, lexer.commenters, lexer.whitespace_split = ' \t\r', '', True
    out = []
    for token in lexer:
        out += PUNCT.findall(token) if set(token) <= set(';&|()<>`\n') else [token.replace('\x01', '\\;')]
    return out


def parse(text, bodies=None):
    """``[pipeline [command {argv, stdin, writes, subs, reads}]]`` of ``text``. Heredoc bodies come out of the
    text first (``bodies`` shared with nested substitutions); ``subs`` are ``$(...)``, backticks and ``<(...)``."""
    if bodies is None:
        lines, found = heredocs(text)
        bodies, text = iter([(f[3], f[2]) for f in found]), '\n'.join(lines)
    return build(tokens_of(text), bodies)


def substitutions(text):
    """The commands of each ``$(...)`` (parentheses balanced) and backtick pair in a word."""
    out, i = [], 0
    while i < len(text):
        if text.startswith('$(', i) and not text.startswith('$((', i):
            depth, j = 1, i + 2
            while j < len(text) and depth:
                depth += (text[j] == '(') - (text[j] == ')')
                j += 1
            out.append(text[i + 2:j - (depth == 0)])
            i = j
        elif text[i] == '`' and '`' in text[i + 1:]:
            end = text.index('`', i + 1)
            out.append(text[i + 1:end])
            i = end + 1
        else:
            i += 1
    return out


def build(tokens, bodies):
    pipelines, pipe = [], []
    cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': [], 'reads': []}

    def end(piped):
        nonlocal cmd, pipe
        if any(cmd.values()):
            pipe.append(cmd)
        cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': [], 'reads': []}
        if not piped and pipe:
            pipelines.append(pipe)
            pipe = []

    def nested(text):
        for inner in substitutions(text):
            try:
                cmd['subs'].append(parse(inner, bodies))
            except ValueError:
                cmd['subs'].append([[{'argv': inner.split(), 'writes': [], 'subs': []}]])

    i = 0
    while i < len(tokens):
        token = tokens[i]
        i += 1
        process = token in ('<', '>') and tokens[i:i + 1] == ['(']           # <(...) and >(...) run their command
        if token == '`' or process or (token == '(' and cmd['argv'] and cmd['argv'][-1].endswith('$')):
            i += process
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
            if cmd['argv'] and cmd['argv'][-1].isdigit():               # 2>/dev/null: the fd, not a word
                cmd['argv'].pop()
            target = tokens[i] if i < len(tokens) else ''
            if token == '<<<':
                cmd['stdin'].append(target)
                nested(target)                                           # <<< "$(...)" runs it
            elif token == '<':
                cmd['reads'].append(target)
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
    knows no boat addresses, so ``is_boat`` then treats any private-LAN address as a boat), and ~/.ssh/config
    aliases for any of them."""
    names = set(config['hosts']) | {host.split('-')[0] for host in config['hosts']} | {target_of(config)} - {'local', SIM}
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


PRIVATE_LAN = re.compile(r'(?:192\.168|10|172\.(?:1[6-9]|2\d|3[01]))(?:\.\d{1,3}){2,3}')


def is_boat(target, names):
    """The boat name an ssh destination (``user@host``, ``host:path``) reaches, else None; without a network env
    (``network_file``) any private-LAN address is a boat (fail closed). ``$H`` names nothing the hook can read: a
    boat."""
    host = target.rsplit('@', 1)[-1].split(':', 1)[0]
    if '$' in host or host in names or host.split('.')[0] in names:
        return host
    return host if network_file() is None and PRIVATE_LAN.fullmatch(host) else None


# --- what a call runs ----------------------------------------------------------------------------------------
# Shell keywords and wrappers run the words after them (their own options first: the letters in the value take the
# next word; POSITIONAL words more, such as timeout's duration). Shells (-c), su -c, script -c, watch, eval and tmux
# new/run take a command string; ssh and kin take a remote one; docker/podman run|exec and pixi/uv/conda run argv.
KEYWORDS = {'!', '{', '}', 'if', 'then', 'else', 'elif', 'do', 'while', 'until', 'coproc'}
WRAP = {'nohup': '', 'setsid': '', 'exec': 'a', 'command': '', 'builtin': '', 'time': 'fo', 'nice': 'n', 'env': 'uCS',
        'sudo': 'ugUhpCDrtTR', 'doas': 'uC', 'stdbuf': 'ioe', 'ionice': 'cnp', 'timeout': 'sk', 'chrt': '',
        'taskset': '', 'flock': 'wEc', 'xargs': 'aEdIiLlnPs', 'strace': 'eoOpPsuS', 'unbuffer': '', 'chroot': '',
        'systemd-run': 'pupE', 'firejail': '', 'nsenter': 'tSG', 'unshare': '', 'catchsegv': '', 'chronic': ''}
POSITIONAL = {'timeout': 1, 'chrt': 1, 'taskset': 1, 'flock': 1, 'chroot': 1}
SHELLS = {'bash', 'sh', 'dash', 'zsh', 'ksh', 'fish'}
SSH = {'ssh': 'bcDEeFIiJLlmOoPpQRSWw', 'autossh': 'MbcDEeFIiJLlmOoPpQRSWw', 'mosh': '', 'et': 'cpt'}
DOCKER = 'eEvlpuwhmc'     # docker/podman run|exec: short options that take a value (long ones: '--x=v' or the next word)
DOCKER_LONG = {'--env', '--volume', '--name', '--workdir', '--user', '--net', '--network', '--publish', '--entrypoint',
               '--env-file', '--mount', '--label', '--device', '--ipc', '--pid', '--gpus', '--runtime', '--add-host',
               '--hostname', '--cap-add', '--cap-drop', '--security-opt', '--shm-size', '--ulimit', '--memory',
               '--cpus', '--platform', '--restart', '--log-driver', '--detach-keys', '--group-add', '--tmpfs'}
TMUX_RUNS = {'new', 'new-session', 'new-window', 'neww', 'split-window', 'splitw', 'respawn-pane', 'respawnp',
             'respawn-window', 'respawnw', 'run-shell', 'run', 'display-popup', 'popup'}
ASSIGN = re.compile(r'[A-Za-z_]\w*=.*', re.S)


def options(words, takes, positional=0, assignments=False):
    """``words`` after a wrapper's options (``-x``; a letter in ``takes`` also takes the next word; ``--`` ends them),
    its VAR=x words (``assignments``: env, sudo) and ``positional`` words."""
    i = 0
    while i < len(words):
        word = words[i]
        if word == '--':
            i += 1
            break
        if word.startswith('-') and len(word) > 1:
            i += 1 + (len(word) == 2 and word[1] in takes)
        elif assignments and ASSIGN.fullmatch(word):
            i += 1
        else:
            break
    return words[i + positional:]


def runs(argv):
    """``(argv, text, remote)``: what ``argv`` runs as argv, as a command string, or on the ssh ``remote``."""
    name, rest = os.path.basename(argv[0]), argv[1:]
    if name in KEYWORDS:
        return rest, None, None
    if name in WRAP:
        return options(rest, WRAP[name], POSITIONAL.get(name, 0), name in ('env', 'sudo', 'doas')), None, None
    if name in SHELLS or name in ('su', 'script'):
        flag = next((i for i, w in enumerate(rest) if re.fullmatch(r'-[A-Za-z]*c|--command', w)), None)
        return None, rest[flag + 1] if flag is not None and flag + 1 < len(rest) else None, None
    if name in ('eval', 'watch'):
        return None, ' '.join(options(rest, 'nd') if name == 'watch' else rest), None
    if name == 'tmux':
        at = next((i for i, w in enumerate(rest) if w in TMUX_RUNS), None)
        if at is not None:
            words = options(rest[at + 1:], 'censtxyFlLSTb')
            return None, ' '.join(words) or None, None
        return None, None, None
    if name == 'sshpass':
        return options(rest, 'pfde'), None, None
    if name in SSH:
        words = options(rest, SSH[name])
        if name == 'et' and words:
            flag = next((i for i, w in enumerate(rest) if w == '-c'), None)
            return None, rest[flag + 1] if flag is not None and flag + 1 < len(rest) else None, words[0]
        if words[1:2] == ['--']:
            words = words[:1] + words[2:]
        return None, ' '.join(words[1:]) or None, words[0] if words else None
    if name in ('docker', 'podman') and rest[:1] in (['run'], ['exec'], ['container']):
        words, i = rest[1 + (rest[:1] == ['container']):], 0
        while i < len(words) and words[i].startswith('-'):
            i += 1 + ((len(words[i]) == 2 and words[i][1] in DOCKER) or words[i] in DOCKER_LONG)
        return words[i + 1:], None, None
    if name in ('pixi', 'uv', 'poetry', 'conda', 'mamba', 'pipenv', 'rye') and rest[:1] == ['run']:
        return options(rest[1:], 'nep'), None, None
    return None, None, None


def written(argv):
    """``[(op, path)]`` the files ``argv`` writes (op ``write`` or ``delete``): removers, movers, copies (the
    destination, and the file a copy into a directory makes), tee, truncate, chmod, dd of=, sed/perl -i and
    find -delete/-exec roots."""
    name, rest = os.path.basename(argv[0]), argv[1:]
    words = options(rest, {'cp': 'tS', 'mv': 'tS', 'ln': 'tS', 'install': 'tmogS', 'rsync': 'eTfB', 'truncate': 'sr',
                           'tee': '', 'sed': 'ef', 'perl': 'eEIM'}.get(name, ''))
    words = [w for w in words if not w.startswith('-')]
    if name in ('rm', 'unlink', 'shred', 'rmdir'):
        return [('delete', w) for w in words]
    if name in ('tee', 'truncate'):
        return [('write', w) for w in words]
    if name in ('chmod', 'chown', 'chgrp', 'chattr'):
        return [('write', w) for w in words[1:]]
    if name in ('cp', 'mv', 'ln', 'install', 'rsync'):
        into = next((w.split('=', 1)[1] if '=' in w else rest[i + 1] for i, w in enumerate(rest[:-1])
                     if w in ('-t', '--target-directory') or w.startswith('--target-directory=')), None)
        sources, dest = (words, into) if into else (words[:-1], words[-1] if words else None)
        if dest is None:
            return []
        out = [('delete', s) for s in sources] if name == 'mv' else []
        return out + [('write', dest)] + [('write', os.path.join(dest, os.path.basename(s.rstrip('/'))))
                                          for s in sources]
    if name == 'dd':
        return [('write', w[3:]) for w in rest if w.startswith('of=')]
    if name in ('sed', 'perl') and any(re.fullmatch(r'-[A-Za-z]*i\S*|--in-place\S*', w) for w in rest):
        script = any(w in ('-e', '-f', '-E') or re.fullmatch(r'-[A-Za-z]*e', w) for w in rest)
        return [('write', w) for w in (words if script else words[1:])]
    if name == 'find' and any(w in FIND_RUNS for w in rest):
        return [('delete', w) for w in (rest[:next((i for i, w in enumerate(rest) if w[:1] in '-(!'), len(rest))]
                                        or ['.'])]
    return []


PYTHON = re.compile(r'python[\d.]*$')
# Python calls that run a command (a literal string or list) and that write or remove a file (a literal path).
PY_RUNS = {'system', 'popen', 'run', 'call', 'check_call', 'check_output', 'Popen', 'getoutput', 'getstatusoutput',
           'create_subprocess_exec', 'create_subprocess_shell'}
PY_DELETES = {'remove', 'unlink', 'rmdir', 'rmtree', 'removedirs'}
PY_WRITES = {'write_text', 'write_bytes', 'truncate', 'touch', 'chmod'}
PY_MOVES = {'rename', 'replace', 'move', 'copy', 'copyfile', 'copy2', 'symlink', 'link'}


def python_runs(code):
    """``(argvs, texts, files)``: what Python ``code`` (``-c`` or on stdin) runs and writes, from its obvious
    literals only: ``subprocess.run([...])`` an argv, ``os.system('...')`` a command string, ``os.remove('...')``,
    ``Path('...').write_text`` or ``open('...', 'w')`` a ``(write|delete, path)``. A variable names nothing."""
    import ast
    try:
        tree = ast.parse(code)
    except (SyntaxError, ValueError):
        return [], [], []

    def text(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.Call) and name(node.func) in ('Path', 'PosixPath', 'PurePath') and node.args:
            return text(node.args[0])
        return None

    def name(func):
        return func.attr if isinstance(func, ast.Attribute) else func.id if isinstance(func, ast.Name) else ''

    argvs, texts, files = [], [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called, args = name(node.func), node.args
        own = text(node.func.value) if isinstance(node.func, ast.Attribute) else None   # Path('x').unlink()
        paths = [p for p in [own] + [text(a) for a in args] if p is not None]
        if called in PY_RUNS and args:
            if isinstance(args[0], (ast.List, ast.Tuple)) and all(text(a) is not None for a in args[0].elts):
                argvs.append([text(a) for a in args[0].elts])
            elif called == 'create_subprocess_exec' and all(text(a) is not None for a in args):
                argvs.append([text(a) for a in args])
            elif text(args[0]) is not None:
                texts.append(text(args[0]))
        elif re.fullmatch(r'exec[lv]p?e?|spawn[lv]p?e?', called):
            lists = [[text(e) for e in a.elts] for a in args
                     if isinstance(a, (ast.List, ast.Tuple)) and all(text(e) is not None for e in a.elts)]
            words = [text(a) for a in args if text(a) is not None]       # execlp('ros2', 'ros2', 'topic', ...)
            argvs += lists[:1] or ([words[1:]] if len(words) > 1 else [])
        elif called in PY_DELETES:
            files += [('delete', p) for p in paths[:1]]
        elif called in PY_WRITES:
            files += [('write', p) for p in paths[:1]]
        elif called in PY_MOVES:
            files += [('delete', p) for p in paths[:1] if called in ('rename', 'replace', 'move')]
            files += [('write', p) for p in paths[1:2]]
        elif called == 'open' and paths:
            mode = next((text(k.value) for k in node.keywords if k.arg == 'mode'), None)
            mode = mode if mode is not None else (text(args[1]) if len(args) > 1 else 'r')
            files += [('write', paths[0])] if mode is None or set(mode) & set('wax+') else []
    return argvs, texts, files


def reads_stdin(argv):
    """``argv`` (a shell or python) takes its program from stdin: no ``-c``, no script, no ``-m``."""
    for word in argv[1:]:
        if word in ('-', '--', '-s') or not word.startswith('-'):
            return word in ('-', '-s')
        if re.fullmatch(r'-[A-Za-z]*[cm]', word):
            return False
    return True


def core(argv):
    """The program ``argv`` runs once its wrappers (sudo, env, timeout, ...) and VAR=x words are taken off."""
    words = list(argv)
    for _ in range(8):
        while words and ASSIGN.fullmatch(words[0]):
            words = words[1:]
        inner = runs(words)[0] if words else None
        if not inner:
            return words
        words = inner
    return words


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


# An agent's own tmux session on a boat host (a boatops-<who>*/agent-<who>* name: it carries the calling agent's id):
# ending it is not stopping the boat's software. Never `boat`, another agent's session, -L/-S sockets or -a.
OWN_TMUX = r'tmux\s+kill-session\s+-t\s*=?(?:boatops|agent)-%s(?:[-_.][\w.-]*)?$'


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
            path = {'local': local_channel, SIM: sim_channel}.get(target, lambda: config['path'])()  # or the target
            if target in ('local', SIM) and not Path(path).parent.is_dir():
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
