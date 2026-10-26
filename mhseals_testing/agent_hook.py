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
# What starts the thruster driver, the mux or the velocity controller: executables and their modules.
DRIVER = r'(thruster_pwm_node|command_mux|calibrated_velocity_controller|thruster_test|thruster_identify)'
MODULE = r'(thruster_pwm_node|command_mux|velocity_controller|identification|thruster_test)'
# Starting the thruster driver, the mux or the velocity controller (arming) in any form: ros2 run, python -m
# the module or the installed path in command position, an import of it (reading or editing its source is fine), a bare or ./ executable in command position, a launch that starts the mux.
ARMING = [r'ros2\s+run\b[^;&|\n]*\bmhseals_hardware\s+(--?\S+\s+)*' + DRIVER + r'\b',
          r'\bpython[\d.]*\s+(-\S+\s+)*-m\s*mhseals_hardware\.' + MODULE + r'\b',
          r'(\bimport\s+|\b(run_module|import_module|__import__)\(\s*)mhseals_hardware\.' + MODULE + r'\b',
          r'\bfrom\s+mhseals_hardware(\.' + MODULE + r'\b|(\.\w+)?\s+import\b[^\n]*\b' + MODULE + r'\b)',
          r'(^|[;&|(`\n]|\b(exec|nohup|setsid|sudo|env|time|nice|stdbuf|xargs|strace|timeout|python[\d.]*|bash|sh|dash'
          r'|zsh)\b)\s*((-\S+|\w+=\S*|\d+\w*)\s+)*(/|~/|\.{1,2}/)?([\w.-]+/)*' + DRIVER + r'(\.py)?(?=\s|$|[;&|)`])',
          r'ros2\s+launch\b.*\bcommand_mux:=(?!false\b|False\b)', r'ros2\s+launch\s+(--?\S+\s+)*\S*mhseals_hardware\b',
          r'ros2\s+launch\s+(--?\S+\s+)*\S*(/|\.launch\.|\.py\b|\.xml\b|\.ya?ml\b)']   # a launch file by path
# Turning sim-arm on: the switch's env name anywhere (MHSEALS_SIM_ARM=1 tui, docker exec -e ...), or a shell write to
# agent.yaml at any path (sim_arm.enabled; the TUI re-reads it live); cp/ln/install/rsync only when agent.yaml is
# the last word or the destination is a directory. Only the owner turns sim-arm on.
# Shell writers of a protected file: editors, movers and filters with an output file (uniq IN OUT, sort -o, tar,
# find -delete/-exec, git checkout/restore/...). agent.yaml adds python and pytest: running a launch file or the
# hook (python3 .agents/hook.py --playbook) is not a write.
WRITERS = (r'rm|mv|dd|tee|sed|perl|awk|gawk|ed|ex|vi|vim|nano|yq|patch|truncate|chmod|unlink|shred|rsync|ln|uniq|sort'
           r'|tar|find(?=[^;&|\n]*\s-(delete|exec\w*|ok\w*|fprint\w*|fls)\b)'
           r'|git\s+(-C\s+\S+\s+)?(checkout|restore|apply|reset|stash|mv|rm)')


def writes(name, more=''):
    """Shell writes to a file ``name`` (a regex): a writer naming it, a copy into it (a copy *from* it to a named
    file is a read), a redirection onto it, a list naming it fed to xargs."""
    span = '.*' if more else r'[^;&|\n]*'
    return [rf'(?<![\w-])({WRITERS}{more})(?![\w.-]){span}({name})',
            rf'(?<![\w-])(cp|install)\b[^;&|\n]*({name})(?![^;&|\n]*\s(?!\S*({name}))\S*[^/\s]\.\w+\s*($|[;&|)\n]))',
            rf'>\|?\s*\S*({name})', rf'({name})[^;&\n]*\|\s*xargs\b']


AGENT_YAML = r'\bagent\.ya?ml\b'
SIM_ARM = [r'\bMHSEALS_SIM_ARM\b'] + writes(AGENT_YAML, r'|python[\d.]*|pytest|py\.test')
# rsync, tar or git checkout/restore into a directory holding a live agent.yaml (``into_live``: d/agent.yaml,
# d/config/agent.yaml or d/*/config/agent.yaml): a config/ restore overwrites it without naming it.
INTO = [r'(?<![\w-])(rsync|tar|git\s+(-C\s+\S+\s+)?(checkout|restore))\b[^;&|\n]*']
# Live launch files (ros2 launch starts the driver and mux from these) and the live agent hook and its wiring.
_START = r'(?:(?<=/)|(?<![^\s;&|<>()=]))'
LAUNCH_SH = _START + r'launch/[^\s;&|<>()/]+\.(?:py|xml|ya?ml)\b|[^\s;&|<>()=/]\.launch\.(?:py|xml|ya?ml)\b'
HOOK_SH = (_START + r'(?:agent_hook\.py|\.agents/hook\.py|\.claude/settings(?:\.local)?\.json|\.codex/hooks\.json)'
           r'(?![\w.-])')
LAUNCHES, HOOKS = writes(LAUNCH_SH), writes(HOOK_SH)
# The mux's saved holds (a sim session's command_mux_holds.json, the boat's command_mux_<domain>.json and their
# directory) and the agent stop file: no shell write, truncation, move or delete. touch on the stop file stays.
# The stop file is <data root>/agent/stop: a path ending in agent/stop anywhere; a whole agent dir or a glob in it
# only under bags/, a $VAR, ~ or bare (a relative agent dir may be the data root's): build/.../agent is not it.
HOLDS = [r'command_mux[\w*?\[\]-]*\.json|\bcommand_mux_\*|\bhold_store\b(?!\.py)',
         r'(?<![\w-])(rm|mv|rmdir|unlink|shred|find|rsync)\b.*/mhseals(/\*?)?(?=\s|$|[;&|)])']
AGENT_DIR = r'((?<![\w./~$}-])|(?<=\bbags/)|(?<=[}~]/)|\$\w+/)agent'
STOP = [r'(?<![\w-])(rm|mv|unlink|shred|truncate|cp|ln|install|dd|tee|sed|perl|python[\d.]*|find|rsync|chmod|chattr'
        r'|rmdir)\b.*(\bagent/stop\b|' + AGENT_DIR + r'/(\*|\?|\['  r'))',
        r'>\|?\s*\S*\bagent/stop\b', r'(?<![\w-])(rm|mv|rmdir|find|rsync)\b.*' + AGENT_DIR + r'/?(?=\s|$|[;&|)])',
        # a bare stop after a cd into the agent dir: cd .../agent && rm stop
        r'\b(cd|pushd)\s+(-\S+\s+)*\S*\bagent/?(?=\s|$|[;&|)]).*((?<![\w-])(rm|mv|unlink|shred|truncate|cp|ln|install'
        r'|dd|tee|sed|perl|python[\d.]*|find|rsync|chmod|chattr)\b.*|>\|?\s*)(?<![\w/.-])(\./)?(stop\b|\*|\?|\[)',
        r'\bstop_path\(\)\.(unlink|rename|replace|write_\w+|open)\b']
# The agent's file tools (Claude Edit/Write/NotebookEdit, Codex apply_patch): each file becomes one line
# ``EDIT create|change|delete live|dev <absolute path>`` (``edit_text``), searched by the deny rules that start
# with ``(?m)^EDIT``. Only live copies are denied (``place``): what the running stack and the TUI read. A linked
# git worktree (~/.cache/wt, ~/worktrees) is where agent.yaml, launch files and the hook are developed: allowed.
# Live: agent.yaml, the mux's saved holds (command_mux*.json, hold_store, the runtime mhseals dir), an existing stop
# file (creating it is fine) and any launch file (ros2 launch starts the driver or mux from an installed one).
LAUNCH = r'(/launch/[^/\n]+\.(py|xml|ya?ml)|[^/\n]\.launch\.(py|xml|ya?ml))$'
EDITS = [r'(?m)^EDIT \w+ live [^\n]*/agent\.ya?ml$',
         r'(?m)^EDIT \w+ live [^\n]*(/command_mux[^/\n]*\.json$|/hold_store(/|$|\.json$)|/mhseals(/|$))',
         r'(?m)^EDIT (change|delete) live [^\n]*/agent/stop$',
         r'(?m)^EDIT \w+ live [^\n]*' + LAUNCH,
         r'(?m)^EDIT \w+ live [^\n]*/(agent_hook\.py|\.agents/hook\.py|\.claude/settings(\.local)?\.json|\.codex/hooks\.json)$']
# A shell file rule (SIM_ARM's agent.yaml writes, HOLDS, STOP) is skipped when every file it names (FILES) is a
# dev copy; a relative name after a cd, a $variable or a glob counts as live (fail closed).
FILES = re.compile(r'[^\s;&|<>()=]*(agent\.ya?ml|command_mux[^\s;&|<>()/]*\.json|agent/stop|\.launch\.(py|xml|ya?ml)'
                   r'|' + LAUNCH_SH + '|' + HOOK_SH + r')(?![\w.-])')
# Nav2's servers: a lifecycle or parameter change breaks a claimed goal that is running.
NAV2 = (r'(controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother'
        r'|collision_monitor|waypoint_follower|route_server|docking_server|lifecycle_manager\w*)')
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
             r'ros2\s+run\b[^;&|\n]*\b(mavros\s+(--?\S+\s+)*mav\w*|teleop_twist_\w+|mhseals_hardware\s+(--?\S+\s+)*'
             r'keyboard_control)\b',
             r'ros2\s+run\b[^;&|\n]*\bmhseals_testing\s+(--?\S+\s+)*operator_heartbeat\b'
             r'|\bpython[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.ros\.operator_heartbeat\b',
             r'ros2\s+(topic\s+pub|service\s+call)\b.*\bthruster_pwm_node/test_pulses\b',
             r'ros2\s+(param\s+(set|load|delete)|lifecycle\s+set)\b.*\b' + NAV2 + r'\b',
             r'ros2\s+service\s+call\b.*\b' + NAV2 + r'/(set_parameters\w*|load_parameters|change_state|manage_nodes)\b',
             r'ros2\s+run\b[^;&|\n]*\bmhseals_nav\s+(--?\S+\s+)*run_task\b',
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_nav\.tasks\b',
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.(procedures|runner)\b',
             r'ros2\s+(topic\s+pub|service\s+call)\b.*(\boperator/heartbeat|\bmhseals_testing/authority'
             r'|\bthruster_pwm_node/estop|\bthrusters/)',
             r'python[\d.]*(\s+-\w+)*\s+(-[A-Za-z]*c|-)\s.*\bcreate_publisher\b',
             r'ros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)|lifecycle\s+set|run)'
             r'\s+(--?[\w-]+\s+)*[^\s-]*\$',
             r'\bxargs\b[^|;&\n]*\bros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)'
             r'|lifecycle\s+set|run)\b',
             r'\b(pkill|killall)\b', r'\bxargs\s+(-\S+\s+)*(sudo\s+)?kill\b',
             r'\bkill\b.*\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller'
             r'|mavros\w*|boat_(trial|test)\w*)\b'] + SIM_ARM + INTO + ARMING + HOLDS + STOP + LAUNCHES + HOOKS + EDITS,
    'hosts': ['blastoise-odroid', 'blastoise-jetson', 'squirtle-jetson'],
    'deny_boat': [r'\b(pkill|killall|kill)\b', r'\b(docker|podman)\b.*\s(stop|kill|restart|rm|down|pause)\b',
                  r'\bsystemctl\s+(--user\s+)?(stop|kill|restart|isolate|reboot|poweroff|halt)\b',
                  r'\b(reboot|poweroff|halt|shutdown)\b', r'\b(tel)?init\s+[0-6sS]\b',
                  r'\bloginctl\s+(terminate|kill)', r'sysrq']}
NETWORK = 'config/boat-network.env'    # legacy path; STATIC_LEASES "MAC=hostname=address ...": the boat computers
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the route that does it instead: (regex on the hit's matched line or None,
# what it blocks, the boat_agent verb (None: the operator's; PID: kill your own process by PID), the deny patterns
# it owns). The deny pattern that matched picks its entry (``use_entry``); patterns no entry owns fall back to the
# regex, searched in the matched text's last line, so a heredoc can't borrow a later line's reason.
_DENY = DEFAULTS['deny']
_KILL_CONTROL = (r'\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller|mavros\w*'
                 r'|boat_(trial|test)\w*)\b')
USE = ((r'\b(pkill|killall|kill)\b.*' + _KILL_CONTROL, "killing the boat's control (the mux, a watchdog, the "
        'driver, MAVROS or a trial)', 'stop', (r'\bkill\b.*' + _KILL_CONTROL,)),
       (None, 'pkill, killall and xargs kill (they can hit processes that are not yours)', 'PID',
        (r'\b(pkill|killall)\b', r'\bxargs\s+(-\S+\s+)*(sudo\s+)?kill\b')),
       (None, 'killing processes on a boat computer', 'stop', (DEFAULTS['deny_boat'][0],)),
       (None, "stopping or restarting the boat's software or computer", None, tuple(DEFAULTS['deny_boat'][1:])),
       (None, 'replaying a bag that can carry motion topics (play only --topics without cmd_vel, thruster or mux '
        'topics)', 'run KEY --claim ID', tuple(p for p in _DENY if p.startswith(r'ros2\s+bag\s+play'))),
       (None, 'turning sim-arm on or changing the live agent.yaml (develop agent.yaml in a git worktree; sim-arm is '
        'off unless the sim container sets MHSEALS_SIM_ARM=1: ask the operator to enable sim-arm in the sim container, '
        'then run KEY --sim-arm)', None,
        tuple(SIM_ARM + INTO)),
       (None, "changing the mux's live saved holds (only the operator releases a hold, with U in the TUI)", None,
        tuple(HOLDS)),
       (None, 'removing or changing the live agent stop file (to stop, touch it or use stop; only the operator '
        'removes it)', None, tuple(STOP)),
       (None, 'changing a live launch file (the running stack starts the thruster driver and mux from these; '
        'develop it in a git worktree, and to run the stack use run KEY --claim ID)', None, tuple(LAUNCHES)),
       (None, 'changing the live agent hook or its wiring (it keeps agents on boat_agent; develop it in a git worktree '
        'and have the operator land it)', None, tuple(HOOKS)),
       (None, 'arming (starting the thruster driver or the mux)', 'run KEY --sim-arm --claim ID', tuple(ARMING)),
       (None, "changing Nav2's servers (a lifecycle or parameter change breaks a running claimed goal)", None,
        tuple(p for p in _DENY if NAV2 in p)),
       (None, 'starting the mission runner directly (outside the claim)', 'run mission --task NAME --claim ID',
        tuple(p for p in _DENY if 'run_task' in p or r'mhseals_nav\.tasks' in p)),
       (None, 'starting a test procedure directly (outside the claim)', 'run KEY --claim ID',
        tuple(p for p in _DENY if r'mhseals_testing\.(procedures' in p)),
       (None, "raw thruster pulses (thruster_pwm_node's test_pulses)", 'run KEY --claim ID',
        tuple(p for p in _DENY if 'test_pulses' in p)),
       (r'cmd_vel|thrusters/|mavros/(rc/override|setpoint|manual_control)|teleop|keyboard_control|create_publisher',
        'raw motion', 'run KEY --claim ID', ()),
       (r'operator/heartbeat|operator_heartbeat|mhseals_testing/authority', "the operator's heartbeat or authority",
        None, ()),
       (r'estop', 'the thruster e-stop', 'stop', ()),
       (r'mavros', 'an FCU arming, mode or parameter change', None, ()),
       (r'\$', 'a ros2 command whose target is a shell variable the hook cannot read (spell the name out)', None, ()),
       (r'ros2\s+(param|lifecycle)\b|/(set_parameters\w*|load_parameters|change_state)\b',
        'changing the thruster driver, the mux or a safety watchdog', None, ()),
       (r'command_mux', 'commanding the mux', 'stop', ()),
       (r'goal_pose|navigate|follow_\w*waypoints|follow_path|dock_robot|assisted_teleop', 'a raw Nav2 goal',
        'goal X Y [YAW] --claim ID', ()),
       (r'.', 'raw motion', 'run KEY --claim ID', ()))
PID_ROUTE = 'kill your own process by exact PID: kill PID (pgrep -af NAME lists them) or kill %JOB'


class Hit(str):
    """A denied command's text (``str``: from the match to the next separator), with the deny ``pattern`` that
    matched, ``tail`` (the last line of the matched text, which names the reason) and ``host``."""
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
EDIT_USE = ((r'/agent\.ya?ml$', 'the live agent.yaml (hook rules, sim-arm switch). Develop it in a git worktree and '
             'have the operator land it; to arm a sim use {cli} run KEY --sim-arm --claim ID, and ask the operator '
             'to enable sim-arm in the sim container: {cli} ask "..."'),
            (r'command_mux|hold_store|/mhseals(/|$)', "the mux's live saved holds. Only the operator releases a hold "
             '(U in the TUI); ask them: {cli} ask "..."'),
            (r'/agent/stop$', 'the live agent stop file. To stop, create or touch it or use {cli} stop; only the '
             'operator removes it'),
            (r'/(agent_hook\.py|hook\.py|settings(\.local)?\.json|hooks\.json)$', 'the live agent hook or its '
             'wiring (it keeps agents on boat_agent). Develop it in a git worktree and have the operator land it'),
            (r'.', 'a live launch file (the running stack starts the thruster driver and mux from these). Develop it '
             'in a git worktree; to run the stack use {cli} run KEY --claim ID (a sim: --sim-arm)'))


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
        '(claim, run, ask, note, release, stop); nothing else there.',
        '  1. `status`: what runs and who drives.',
        '  2. `claim --reason "..."`: ask the operator for the boat.',
        '  3. `run KEY` (`run mission --task NAME`) or `goal X Y [YAW]` with `--claim ID --dry-run`, then for real.',
        '     Sim only: `run KEY --sim-arm` arms a verified sim for that run and disarms after; it needs the sim '
        'container started with MHSEALS_SIM_ARM=1 (the owner turns it on; ask), and a real boat refuses it '
        '(the operator arms).',
        '     Start sims and heavy runs through the host guard: `.handoff/tools/heavy sim ...` (Unity headless).',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  `stop` any time: the operator and the agent it interrupts see your id. `ask "..."` instead of guessing. '
        '`note "..."` writes a line in the TUI log.',
        'Exit codes: 0 ok, 2 refused (read hint), 3 revoked or overridden (the hint says who: '
        'the operator or another agent; claim again only if asked), 4 no TUI, 5 agents blocked, 6 timeout.',
        'Never publish /cmd_vel, /goal_pose, mux or MAVROS commands, change Nav2 lifecycles or params, send raw '
        'test_pulses, start the mission runner or a procedure directly, never arm (but --sim-arm) or release a hold: '
        'the hook blocks them and names the verb; kill only your own processes by exact PID; never answer a '
        'TUI popup. Run in sim first. For runs over 9 min use '
        f'{background} (the CLI keeps the lease). The TUI shows your boat commands'
        + (f' ({remote(config)}).' if remote(config) else
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
    if '--sim-arm' in verb and (host or remote(config)):   # a real boat refuses --sim-arm
        verb = verb.replace(' --sim-arm', '')
        arms = ' On a real boat the operator arms from the arm panel; --sim-arm is sim only.'
    elif '--sim-arm' in verb:
        arms = ' --sim-arm needs a verified sim whose container sets MHSEALS_SIM_ARM=1; a real boat is armed by the operator.'
    return (f'{head} Use: {cli} {verb} --who {who}'
            + ('' if verb == 'stop' else f' (claim first: claim --reason "..." --who {who})')
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
    """The command as the shell reads its words: continuations joined, quotes and escapes dropped,
    so ``/cmd''_vel``, ``/cmd\\_vel`` or ``ros2 'topic' pub`` cannot hide a match."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '').replace('\\', '')


def matches(patterns, text):
    """Any pattern found in ``plain(text)``; ``.`` spans newlines (the TUI-note filter)."""
    text = plain(text)
    return any(re.search(p, text, re.S) for p in patterns)


# Two tiers. Tier A: every command in the call (each part of a pipe or list, each $(...)) only reads,
# searches or prints text (READ_ONLY, find without -exec/-delete, sort/uniq without an output file,
# git commit/log/diff/show/status, pytest): its arguments are data,
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
READ_ONLY = {'grep', 'egrep', 'fgrep', 'rg', 'echo', 'printf', 'cat', 'head', 'tail', 'less', 'wc', 'cut', 'nl',
             'tr', 'column', 'ls', 'stat', 'diff', 'jq', 'basename', 'dirname', 'realpath', 'readlink', 'pwd', 'du',
             'comm', 'true'}
FIND_RUNS = ('-exec', '-execdir', '-ok', '-okdir', '-delete', '-fprint', '-fprint0', '-fprintf', '-fls')
PYTEST_WRITES = ('--junit', '--basetemp', '--result-log', '--report-log', '--debug', '--html', '--json', '--cov',
                 '--log-file', '-o', '--override-ini', '--lf', '--last-failed', '--ff', '--failed-first', '--sw',
                 '--stepwise', '--cache-clear', '--pastebin', '--rootdir', '--confcutdir', '-p', '-c', '--inifile',
                 '--config-file')
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
        return not any(a.startswith(PYTEST_WRITES) for a in args)       # --junitxml=F writes F, --basetemp=D wipes D
    if name == 'sed':                   # sed -n [-E|-r] SCRIPT FILE...: SCRIPT only prints and substitutes
        flags = [a for a in args if a.startswith('-')]
        rest = [a for a in args if not a.startswith('-')]
        return (all(re.fullmatch(r'-[nEr]+', a) for a in flags) and 'n' in ''.join(flags) and bool(rest)
                and args.index(rest[0]) == len(flags) and bool(SED_PLAIN.fullmatch(rest[0])))
    if name == 'rg':
        return not any(a.startswith('--pre') for a in args)     # --pre runs a program on each file
    if name == 'find':                  # no -exec/-ok (runs), -delete or -fprint/-fls (writes)
        return not any(a in FIND_RUNS for a in args)
    if name == 'sort':                  # -o writes, --compress-program runs
        return not any(a.startswith(('-o', '--output', '--compress')) or re.match(r'-[^-]*o', a) for a in args)
    if name == 'uniq':                  # uniq IN OUT writes OUT; - is stdin, a file argument too
        return len([a for a in args if a == '-' or not a.startswith('-')]) <= 1
    if name == 'file':                  # -C compiles a magic file
        return not any(re.match(r'-[^-]*C', a) or a == '--compile' for a in args)
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


def denied(command, config, cwd=None):
    """``(what it blocks, the boat it names or None)``, else None. Tier A (``text_only``) is never
    denied; anything else is denied when ``hook.deny`` is found anywhere in its text, or
    ``hook.deny_boat`` when it names a boat (or runs on one)."""
    if text_only(command):
        return None
    text, names = links(plain(command), cwd), boats(config)
    alternatives = [re.escape(n) for n in sorted(names, key=len, reverse=True)]
    if network_file() is None:      # no network env (a checkout without astro_dock): fail closed, any private-LAN address
        alternatives.append(r'(?:192\.168|10|172\.(?:1[6-9]|2\d|3[01]))(?:\.\d{1,3}){2,3}')
    found = re.search(r'(?<![\w.-])(%s)(?![\w-])' % '|'.join(alternatives), text) if alternatives else None
    host = found.group(1) if found else (hostname().split('.')[0] if hostname().split('.')[0] in names else None)
    live_only = set(SIM_ARM[1:] + HOLDS + STOP + LAUNCHES + HOOKS)   # file rules: a dev copy (a worktree) is development
    for pattern in config['deny'] + (config['deny_boat'] if host else []):
        for match in re.finditer(pattern, text, re.S):
            rest = re.compile(r'[\n;|&]').search(text, match.end())
            hit = ' '.join(text[match.start():rest.start() if rest else None].split())
            if pattern in INTO and not into_live(hit, cwd, text):
                continue
            if pattern in live_only and all_dev(hit, cwd, text):
                continue
            tail = text[match.start():rest.start() if rest else None].rsplit('\n', 1)[-1]   # the matched line
            return Hit(hit, pattern, ' '.join(tail.split()), host), host
    return None


def links(text, cwd=None):
    """``text`` with each literal path that is (or runs through) a symlink to a protected file (``FILES``: agent.yaml,
    holds, stop, launch, hook) replaced by its target, so ``echo x >> wt/link`` to a live agent.yaml is a write to it."""
    base = where(text, cwd)

    def real(match):
        paths = resolve(match.group(0), base)
        target = paths[-1] if paths else None
        return target if target and target != paths[0] and FILES.fullmatch(target) else match.group(0)
    return re.sub(r'(?<![^\s;&|<>()=])[~/.\w][^\s;&|<>()=$`*?]*', real, text)


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


def where(whole, cwd):
    """The directory relative names resolve in: the call's ``cwd``, or the literal directory of a leading
    ``cd DIR &&`` when that is the call's only cd; None when it changes directory otherwise (fail closed)."""
    moves = re.findall(r'(?<![\w-])(?:cd|pushd)\b', whole)
    lead = re.match(r'\s*(?:cd|pushd)\s+([/~][^\s;&|$`*?\[{]*)\s*&&', whole)
    if lead and len(moves) == 1:
        return os.path.expanduser(lead.group(1))
    return None if moves else cwd


def resolve(name, base):
    """``name`` as absolute paths (as given and its symlink target), or None for a $variable, a glob or a relative
    name with no ``base``."""
    if re.search(r'[$*?\[{`]', name) or (not name.startswith(('/', '~')) and not base):
        return None
    path = Path(os.path.expanduser(name))
    path = os.path.abspath(path if path.is_absolute() else Path(base) / path)
    return list(dict.fromkeys([path, os.path.realpath(path)]))


def targets(text):
    """Paths a command in ``text`` writes that the protected names do not show: the file a copy into a directory
    makes (cp/mv/ln/install/rsync SRC... DIR, -t DIR) and find's roots."""
    words = text.split()
    if not words:
        return []
    if words[0] == 'find':
        roots = []
        for word in words[1:]:
            if word.startswith(('-', '(', '!')):
                break
            roots.append(word)
        return roots or ['.']
    if words[0] not in ('cp', 'mv', 'ln', 'install', 'rsync'):
        return []
    args, into, skip = [], None, False
    for i, word in enumerate(words[1:], 1):
        if skip:
            skip = False
        elif word in ('-t', '--target-directory'):
            into, skip = (words[i + 1] if i + 1 < len(words) else None), True
        elif word.startswith('--target-directory='):
            into = word.split('=', 1)[1]
        elif not word.startswith('-'):
            args.append(word)
    if into is None and len(args) >= 2 and (args[-1].endswith('/') or os.path.isdir(os.path.expanduser(args[-1]))):
        into, args = args[-1], args[:-1]
    return [os.path.join(into, os.path.basename(a.rstrip('/'))) for a in args
            if into and FILES.fullmatch('/' + os.path.basename(a.rstrip('/')))]


def all_dev(text, cwd=None, whole=''):
    """Every file ``FILES`` finds in ``text``, and every file it copies into a directory or root it finds under
    (``targets``), is a dev copy (literal and symlink target), and it names at least one. A relative name is live
    when the call (``whole``) changes directory other than by one leading ``cd DIR &&``."""
    base = where(whole or text, cwd)
    names, made = [m.group(0) for m in FILES.finditer(text)], targets(text)
    for name in names + made:
        paths = resolve(name, base)
        if paths is None:
            return False
        if name in made and not any(any((d / '.git').exists() for d in Path(path).parents) for path in paths):
            continue                    # a copy into a directory outside git (/tmp/): nothing reads it
        if any(place(path) != 'dev' for path in paths):
            return False
    return bool(names + made)


def into_live(text, cwd=None, whole=''):
    """An rsync, tar -x or git checkout/restore in ``text`` writes into a directory holding a live agent.yaml (or
    one it cannot read: a $variable, a remote ...config/ dir). Its destination, -C dir or pathspecs."""
    words, base = text.split(), where(whole or text, cwd)
    if words[:1] == ['rsync']:
        args = [w for w in words[1:] if not w.startswith('-')]
        dirs = args[-1:] if len(args) >= 2 else []
    elif words[:1] == ['tar']:
        if not (any(re.fullmatch(r'-?[A-Za-z]*x[A-Za-z]*', w) for w in words[1:2]) or any(
                re.fullmatch(r'-[A-Za-z]*x[A-Za-z]*|--extract|--get', w) for w in words[1:])):
            return False                                                # only an extract writes
        found = re.search(r'(?:\s-C\s*|--directory[= ])(\S+)', text)
        dirs = [found.group(1) if found else '.']
    else:
        found = re.search(r'\s-C\s+(\S+)', text)                       # git -C DIR: pathspecs are relative to DIR
        if found:
            base = (resolve(found.group(1), base) or [None])[0]
        rest = re.split(r'\b(?:checkout|restore)\b', text, maxsplit=1)[-1].split()
        dirs, skip = [], False
        for word in rest:
            if skip:
                skip = False
            elif word in ('-s', '--source', '-b', '-B'):
                skip = True
            elif not word.startswith('-'):
                dirs.append(word)
    for name in dirs:
        if re.match(r'[\w.-]+@?[\w.-]*:', name):                    # a remote destination: a config dir is live
            if re.search(r'(^|[:/])config/?$', name):
                return True
            continue
        paths = resolve(name, base)
        if paths is None:
            return True
        for path in paths:
            for found in [Path(path)] + [Path(path) / 'agent.yaml', Path(path) / 'config' / 'agent.yaml'] + list(
                    Path(path).glob('*/config/agent.yaml')) if os.path.isdir(path) else [Path(path)]:
                if found.name in ('agent.yaml', 'agent.yml') and found.exists() and place(str(found)) != 'dev':
                    return True
    return False


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
    """The deny reason for an edit tool's files (``event['edits']``), else None. Fails closed: a missing path or
    any error denies."""
    who = who_of(event)
    try:
        text = edit_text(event.get('edits'), event.get('cwd'))
        if text is None:
            return f'Blocked an edit whose file the hook cannot read (fail closed). Name the file. --who {who}'
        for pattern in config['deny']:
            match = re.search(pattern, text) if pattern.startswith('(?m)^EDIT') else None
            if match:
                line = match.group(0)
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
    if phase == 'pre' and 'edits' in event:     # an edit tool, or a shell apply_patch: its files first
        reason = edit_denied(event, config)
        if reason:
            return {'deny': reason}
    if phase == 'pre':
        blocked = denied(command, config, event.get('cwd'))
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
