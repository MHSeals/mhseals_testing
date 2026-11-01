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
# ros2 run PKG ... EXE: any flags or --prefix value between the package and the executable.
def ros2_run(package, executable):
    return r'ros2\s+run\b[^;&|\n]*\b' + package + r'\s+([^;&|\n]*\s)?' + executable + r'\b'


def started(names):
    """``names`` (executables) started in command position: bare, by path (./, ~/, /, install/.../lib/...) or after
    exec, env, nice, python and the like."""
    return (r'(^|[;&|(`\n]|\b(exec|nohup|setsid|sudo|env|time|nice|stdbuf|xargs|strace|timeout|python[\d.]*|bash|sh|dash'
            r'|zsh)\b)\s*((-\S+|\w+=\S*|\d+\w*)\s+)*(/|~/|\.{1,2}/)?([\w.-]+/)*' + names + r'(\.py)?(?=\s|$|[;&|)`])')


ARMING = [ros2_run('mhseals_hardware', DRIVER),
          r'\bpython[\d.]*\s+(-\S+\s+)*-m\s*mhseals_hardware\.' + MODULE + r'\b',
          r'(\bimport\s+|\b(run_module|import_module|__import__)\(\s*)mhseals_hardware\.' + MODULE + r'\b',
          r'\bfrom\s+mhseals_hardware(\.' + MODULE + r'\b|(\.\w+)?\s+import\b[^\n]*\b' + MODULE + r'\b)',
          started(DRIVER),
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


# A python one-liner (-c or stdin) that writes, moves or deletes a file: open(..., w|a|x), write_*, unlink, ...
PY_WRITE = (r'\bpython[\d.]*\s+(-\S+\s+)*(-[A-Za-z]*c\s|(-\s+)?<<)(?=.*(\b(write_\w+|unlink|remove|rename|replace|copy\w*|move'
            r'|truncate|chmod|symlink_to|rmtree)\(|\bopen\([^)\n]*[,(]\s*(mode=)?[wax]))')


# xargs running a reader (wc, grep, cat, head ...) on the files a list names reads them; any other program may write.
XARGS_READS = (r'(?!(\s+(-[nPLsI]\s*\S+|-\S+))*\s+(wc|grep|egrep|fgrep|rg|cat|head|tail|ls|stat|file|md5sum|sha\d*sum'
               r'|du|nl|less)(?=[\s;&|)]|$))')


def writes(name, more=''):
    """Shell writes to a file ``name`` (a regex): a writer naming it, a copy into it (a copy *from* it to a named
    file is a read), a redirection onto it, a list naming it fed to xargs, a python one-liner writing it."""
    span = '.*' if more else r'[^;&|\n]*'
    return [rf'(?<![\w-])({WRITERS}{more})(?![\w.-]){span}({name})', PY_WRITE + rf'.*({name})',
            rf'(?<![\w-])(cp|install)\b[^;&|\n]*({name})(?![^;&|\n]*\s(?!\S*({name}))\S*[^/\s]\.\w+\s*($|[;&|)\n]))',
            rf'>\|?\s*\S*({name})', rf'({name})[^;&\n]*\|\s*xargs\b' + XARGS_READS]


AGENT_YAML = r'\bagent\.ya?ml\b'
SIM_ARM = [r'\bMHSEALS_SIM_ARM\b'] + writes(AGENT_YAML, r'|python[\d.]*|pytest|py\.test')
# Writes into a directory that is or holds a live protected dir (``into_live``: a config dir with agent.yaml, a
# launch dir, the hook's dirs) without naming the file: rsync, tar -x, git checkout/restore/apply, patch (-d),
# cp -r/-a, and find whose root is a protected dir with -delete or -exec/-ok of a writer.
FIND_WRITE = (r'find(?=[^;&|\n]*\s(-delete|-fprint\w*|-fls|-(exec|execdir|ok|okdir)\s+(\S*/)?(rm|rmdir|mv|sed|perl|cp'
              r'|tee|truncate|chmod|shred|unlink|ln|dd|install|rsync|sh|bash)\b))')
INTO = [r'(?<![\w-])(rsync|tar|patch|git\s+(-C\s+\S+\s+)?(checkout|restore|apply)'
        r'|cp(?=[^;&|\n]*\s(-[A-Za-z]*[raR][A-Za-z]*|--recursive|--archive)(?=\s))|' + FIND_WRITE + r')\b[^;&|\n]*']
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
# On a boat: a stack component started or stopped from a shell (any mhseals_nav/mhseals_hardware launch: the
# umbrella device launch or one part's, e.g. odom.launch.py; or its mhseals-<part> unit)
# instead of boat_agent start/stop COMPONENT, which goes through the TUI's Status row path under the claim.
BOAT_START = [r'\bros2\s+launch\s+(--?\S+\s+)*mhseals_(nav|hardware)\b',
              r'\bsystemctl\s+(--user\s+)?(start|restart)\s+(\S+\s+)*mhseals-']
BOAT_STOP_PART = [r'\bsystemctl\s+(--user\s+)?(stop|kill)\s+(\S+\s+)*mhseals-']
BOAT_KILL = r'\b(pkill|killall|kill)\b'
# git that runs a program of the caller's choosing: a config or environment set for the call (-c with any key but
# identity, colour or advice; --config-env, --exec-path; GIT_* other than identity, GIT_PAGER=cat|less or
# GIT_TERMINAL_PROMPT), or a hooks, fsmonitor, alias, filter, diff or merge driver setting (git config included).
GIT_RUNS = [r'(?<![\w/.-])git\b[^;&|\n]*\s(-c\s*(?![\'"]?(user\.(name|email)|color\.\S+|advice\.\S+|commit\.gpgsign)=)\S'
            r'|--config-env\b|--exec-path\b)',
            r'(?<![\w-])GIT_(?!(AUTHOR|COMMITTER)_(NAME|EMAIL|DATE)=|TERMINAL_PROMPT=|PAGER=([\'"]?(cat|less)?[\'"]?)(\s|$))\w+=',
            r'(?<![\w/.-])git\b[^;&|\n]*\b(core\.(hooksPath|fsmonitor|sshCommand|pager|editor|askPass|gitProxy)|alias\.'
            r'|filter\.|diff\.\S*(textconv|command)|merge\.\S*driver|credential\.helper|include(If)?\.)']
# Typing into the boat's boat_test TUI: tools/boat_tui runs it on its own tmux server (tmux -L boat) on a boat host;
# keys sent to any of its panes (-t boat, -t %3, no -t, or chained after \;, which the shell hands tmux as ;) would
# answer the operator's popups, arm or release a hold. A deny_boat rule: only on a boat host or via ssh to one;
# private sockets (-L/-S) for sim and dev stay free. Watching it (capture-pane, list, status) stays allowed.
TUI_KEYS = [r'\btmux\b(?:(?!\btmux\b)[^&|\n])*?' + r'''(-L\s*['"]?boat\b|-S\s*['"]?\S*/tmux-\d+/boat\b)'''
            r'(?:(?!\btmux\b)[^&|\n])*?(?<![\w-])(send-keys|send|paste-buffer|pasteb|load-buffer|loadb|pipe-pane|pipep)'
            r'(?![\w-])']
# Killing the boat's control: a kill whose own arguments (with their $(...) and backticks) name it, or a kill of a
# $variable in a call that names it anywhere (pid=$(pgrep mavros); kill $pid). A kill of a literal PID is not, whatever
# else the call says (kill 4242; echo stopped the mavros sim).
_CONTROL = (r'\b(command_mux|\w*watchdog|approach_guard|thruster_pwm\w*|\w*velocity_controller|mavros\w*'
            r'|boat_(trial|test)\w*)\b')
_KILL_ARGS = r'(?<![\w-])kill\b(?:[^;&|\n$`]|\$\([^)]*\)|`[^`]*`)*?(?:\$\([^)]*?|`[^`]*?)?'
_KILL_VAR = r'(?<![\w-])kill\b(?:[^;&|\n]*[$`]|\(\s*(?!\d)\w)'      # kill $pid, kill $(cat f), os.kill(pid, 9)
KILL_CONTROL = f'{_KILL_ARGS}{_CONTROL}|{_KILL_VAR}.*{_CONTROL}|{_CONTROL}.*{_KILL_VAR}'
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
             ros2_run('mavros', r'mav\w*'), r'ros2\s+run\b[^;&|\n]*\bteleop_twist_\w+\b',
             ros2_run('mhseals_hardware', 'keyboard_control'),
             ros2_run('mhseals_testing', 'operator_heartbeat')
             + r'|\bpython[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.ros\.operator_heartbeat\b',
             started('operator_heartbeat'),
             r'ros2\s+(topic\s+pub|service\s+call)\b.*\bthruster_pwm_node/test_pulses\b',
             r'ros2\s+(param\s+(set|load|delete)|lifecycle\s+set)\b.*\b' + NAV2 + r'\b',
             r'ros2\s+service\s+call\b.*\b' + NAV2 + r'/(set_parameters\w*|load_parameters|change_state|manage_nodes)\b',
             ros2_run('mhseals_nav', 'run_task'),
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_nav\.tasks\b',
             r'python[\d.]*\s+(-\S+\s+)*-m\s*mhseals_testing\.(procedures|runner)\b',
             r'ros2\s+(topic\s+pub|service\s+call)\b.*(\boperator/heartbeat|\bmhseals_testing/authority'
             r'|\bthruster_pwm_node/estop|\bthrusters/)',
             r'python[\d.]*(\s+-\w+)*\s+(-[A-Za-z]*c|-)\s.*\bcreate_publisher\b',
             r'ros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)|lifecycle\s+set|run)'
             r'\s+(--?[\w-]+\s+)*[^\s-]*\$',
             r'\bxargs\b[^|;&\n]*\bros2\s+(topic\s+pub|action\s+send_goal|service\s+call|param\s+(set|load|delete)'
             r'|lifecycle\s+set|run)\b',
             r'\b(pkill|killall)\b', r'\bxargs\s+((-[nPLsIdEa]\s*\S+|-\S+)\s+)*(sudo\s+)?kill\b', KILL_CONTROL]
            + SIM_ARM + INTO + ARMING + HOLDS + STOP + LAUNCHES + HOOKS + EDITS + GIT_RUNS,
    'hosts': ['blastoise-odroid', 'blastoise-jetson', 'squirtle-jetson'],
    'deny_boat': BOAT_START + BOAT_STOP_PART +
                 [r'\b(pkill|killall|kill)\b', r'\b(docker|podman)\b.*\s(stop|kill|restart|rm|down|pause)\b',
                  r'\bsystemctl\s+(--user\s+)?(stop|kill|restart|isolate|reboot|poweroff|halt)\b',
                  r'\b(reboot|poweroff|halt|shutdown)\b', r'\b(tel)?init\s+[0-6sS]\b',
                  r'\bloginctl\s+(terminate|kill)', r'sysrq'] + TUI_KEYS}
NETWORK = 'config/boat-network.env'    # legacy path; STATIC_LEASES "MAC=hostname=address ...": the boat computers
WINDOW_S = 10.0
BOAT_AGENT = '~/astro_dock/src/mhseals_testing/tools/boat_agent'   # the boat hosts' checkout
# Why a denied command is denied and the route that does it instead: (regex on the hit's matched line or None,
# what it blocks, the boat_agent verb (None: the operator's; PID: kill your own process by PID), the deny patterns
# it owns). The deny pattern that matched picks its entry (``use_entry``); patterns no entry owns fall back to the
# regex, searched in the matched text's last line, so a heredoc can't borrow a later line's reason.
_DENY = DEFAULTS['deny']
USE = ((r'\b(pkill|killall|kill)\b.*' + _CONTROL, "killing the boat's control (the mux, a watchdog, the "
        'driver, MAVROS or a trial)', 'stop', (KILL_CONTROL,)),
       (None, 'pkill, killall and xargs kill (they can hit processes that are not yours)', 'PID',
        (r'\b(pkill|killall)\b', r'\bxargs\s+((-[nPLsIdEa]\s*\S+|-\S+)\s+)*(sudo\s+)?kill\b')),
       (None, 'starting a stack component on a boat (its device launch or unit)', 'start COMPONENT --claim ID',
        tuple(BOAT_START)),
       (None, 'stopping a stack component on a boat (its unit)', 'stop COMPONENT --claim ID', tuple(BOAT_STOP_PART)),
       (None, 'killing processes on a boat computer (a stack component: stop COMPONENT --claim ID; your own tmux '
        'session: tmux kill-session -t agent-<your id>...)', 'stop',
        (BOAT_KILL,)),
       (None, "stopping or restarting the boat's software or computer (the FCU: boat_agent fcu restart --claim ID)",
        None,
        tuple(p for p in DEFAULTS['deny_boat'] if p not in BOAT_START + BOAT_STOP_PART + [BOAT_KILL] + TUI_KEYS)),
       (None, 'replaying a bag that can carry motion topics (play only --topics without cmd_vel, thruster or mux '
        'topics)', 'run KEY --claim ID', tuple(p for p in _DENY if p.startswith(r'ros2\s+bag\s+play'))),
       (None, 'turning sim-arm on or changing the live agent.yaml (develop agent.yaml in a git worktree; sim-arm is '
        'off unless the sim container sets MHSEALS_SIM_ARM=1: ask the operator to enable sim-arm in the sim container, '
        'then run KEY --sim-arm)', None,
        tuple(SIM_ARM)),
       (None, 'writing into a live config, launch or hook dir (it holds agent.yaml, launch files or the agent hook; '
        'develop in a git worktree and have the operator land it)', None, tuple(INTO)),
       (None, "changing the mux's live saved holds (only the operator releases a hold, with U in the TUI)", None,
        tuple(HOLDS)),
       (None, 'removing or changing the live agent stop file (to stop, touch it or use stop; only the operator '
        'removes it)', None, tuple(STOP)),
       (None, 'changing a live launch file (the running stack starts the thruster driver and mux from these; '
        'develop it in a git worktree, and to run the stack use run KEY --claim ID)', None, tuple(LAUNCHES)),
       (None, 'changing the live agent hook or its wiring (it keeps agents on boat_agent; develop it in a git worktree '
        'and have the operator land it)', None, tuple(HOOKS)),
       (None, 'git with a config, environment, hook, alias or filter that runs a program (plain git add, commit, diff, '
        'log, show and status are fine)', None, tuple(GIT_RUNS)),
       (None, "typing into the operator's boat_test TUI (its tmux session): popups, arming and holds are the "
        "operator's; ask with ask \"...\", watch with status or tmux capture-pane", None, tuple(TUI_KEYS)),
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
        '(claim, run, start, fcu, ask, note, release, stop) or boat_tui; nothing else there. You may end your own '
        f'tmux sessions there (name them `agent-{who}-...` and end them with `tmux kill-session -t agent-{who}-...`), never `boat`, '
        "another agent's session or `-L boat`.",
        '  1. `status`: what runs and who drives.',
        '  2. `claim --reason "..."`: ask the operator for the boat.',
        '  3. `run KEY` (`run mission --task NAME`) or `goal X Y [YAW]` with `--claim ID --dry-run`, then for real.',
        '     Sim only: `run KEY --sim-arm` arms a verified sim for that run and disarms after; it needs the sim '
        'container started with MHSEALS_SIM_ARM=1 (the owner turns it on; ask), and a real boat refuses it '
        '(the operator arms).',
        '     `start COMPONENT --claim ID` / `stop COMPONENT --claim ID` starts or stops a stack part agent.yaml '
        '`start:` lists (odometry, EKFs, sensors, perception) as its TUI Status row does; its row shows your id. '
        'Never the driver, mux, Nav2 or watchdogs; arming stays the operator\'s.',
        '     `fcu get PARAM...` and `fcu rates` (IMU/odom rates, gap p99, FCU load, "Main loop slow") read only; '
        '`fcu set PARAM VALUE... --claim ID` (agent.yaml `fcu.set`: loop rate, SR0_/SR1_ stream rates, IMU filters; '
        'never yaw source, compass, servos, arming or failsafes), `fcu revert --to SNAPSHOT --claim ID` and '
        '`fcu restart --claim ID` (disarmed) go through the TUI\'s MAVROS link: snapshot, read-back and '
        'fcu_changes.md name you. SCHED_LOOP_RATE 400 only after `fcu rates` at 200 shows the FCU load low.',
        '     Start sims and heavy runs through the host guard: `.handoff/tools/heavy sim ...` (Unity headless).',
        '     No TUI (exit 4): `boat_tui up` (beside boat_agent) starts the stack and TUI in tmux and prints the line '
        'the operator attaches with; `boat_tui status` says who controls. Never type into its tmux session.',
        '  4. `follow --claim ID --until ...`: watch it.  5. `release --claim ID`: hand it back.',
        '  `stop` any time: the operator and the agent it interrupts see your id. `ask "..."` instead of guessing. '
        '`note "..."` writes a line in the TUI log.',
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
# git commit/log/diff/show/status, pytest, cd): its arguments are data,
# so a grep pattern, an echo, a commit message or a heredoc into a file never denies. Tier B, anything
# else: the whole command text is searched (quotes and escapes dropped, ``.`` spanning newlines), so
# whatever wraps or hides a denied command is still denied, except a heredoc body nothing can run (a codex-task
# prompt, ``code_of``). Variables, strings built at run time and
# scripts from earlier calls still get through: advisory, the mux and the claim are the boundary.
SEPARATORS = {';', '&', '&&', '||', '|', '|&', '\n', ';;', '(', ')'}
REDIRECTS = {'<', '>', '>>', '>|', '&>', '&>>', '>&', '<&', '<>'}
WRITES = {'>', '>>', '>|', '&>', '&>>'}
PUNCT = re.compile(r'<<<|<<|>>|>&|<&|<>|&>>|&>|>\||\|\||&&|\|&|;;|[\s\S]')
HEREDOC = re.compile(r'(?<!<)<<(?!<)-?\s*([\'"]?)([\w.-]+)\1')
HEREDOC_OP = re.compile(r'(?<!<)<<(?!<)(-?)[ \t]*')

SUBST = re.compile(r'\$\(([^\n]*)|`([^`]*)`')
READ_ONLY = {'grep', 'egrep', 'fgrep', 'rg', 'echo', 'printf', 'cat', 'head', 'tail', 'less', 'wc', 'cut', 'nl',
             'tr', 'column', 'ls', 'stat', 'diff', 'jq', 'basename', 'dirname', 'realpath', 'readlink', 'pwd', 'du',
             'comm', 'true', 'cd', 'pushd', 'popd', 'mkdir'}    # cd or mkdir changes nothing: the rest must only read
FIND_RUNS = ('-exec', '-execdir', '-ok', '-okdir', '-delete', '-fprint', '-fprint0', '-fprintf', '-fls')
PYTEST_WRITES = ('--junit', '--basetemp', '--result-log', '--report-log', '--debug', '--html', '--json', '--cov',
                 '--log-file', '-o', '--override-ini', '--lf', '--last-failed', '--ff', '--failed-first', '--sw',
                 '--stepwise', '--cache-clear', '--pastebin', '--rootdir', '--confcutdir', '-p', '-c', '--inifile',
                 '--config-file')
GIT_READ = {'add', 'commit', 'log', 'diff', 'show', 'status'}      # add only stages: git add && git commit -m is text
_ADDR = r'(\d+|\$|/[^/\n]*/)(,(\d+|\$|/[^/\n]*/))?'
_SED = rf'({_ADDR})?!?(p|s/[^/\n]*/[^/\n]*/[gpI\d]*)'
SED_PLAIN = re.compile(rf'{_SED}(;{_SED})*;?')      # print and s/// only: no e, w, r or other delimiters


def delimiter(line, start):
    """The heredoc word at ``line[start:]`` as bash reduces it: quotes and backslashes removed, and any of them
    makes the body literal. ``(word, quoted)``; ValueError for a word the hook will not guess at ($, `, an
    empty or open-quoted word, a backslash inside double quotes)."""
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
    """``(lines, bodies)``: the command lines of ``text`` (backslash-newline joined, as bash reads them) with
    each heredoc body taken out after its line, and ``bodies`` ``[[dash, word, quoted, body, line index, terminator]]``.
    Bodies follow bash: a quoted delimiter keeps the body literal (a trailing backslash joins nothing), the
    terminator is the exact line. Any doubt raises ValueError so the caller treats the whole call as code: an
    odd delimiter, an unterminated body, or a line ending in a backslash inside an unquoted body."""
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
                if not quoted and raw[i].endswith('\\'):
                    raise ValueError('heredoc continuation')
                body.append(raw[i])
                i += 1
            if i >= len(raw):
                raise ValueError('unterminated heredoc')
            found.append([dash, word, quoted, '\n'.join(body), len(lines) - 1, raw[i]])
            i += 1
    return lines, found


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


def text_only(command, cwd=None):
    """Tier A, side-effect free: no $(...), backticks or <(...)/>(...) anywhere (here-strings and
    heredocs included), no file written but data files (``data_target``: notes, reports, logs; /dev/null and 2>&1),
    and every command ``reads_only``: nothing in the call can run what it writes."""
    if re.search(r'\$\(|`|[<>]\(', command):
        return False
    try:
        pipelines = parse(command)
    except ValueError:                  # an open quote: unreadable, so Tier B
        return False
    return bool(pipelines) and all(reads_only(cmd['argv']) and not cmd['subs']
                                   and all(data_target(t, where(command, cwd)) for t in cmd['writes'])
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
        lines, found = heredocs(text)
        bodies, text = iter([(f[3], f[2]) for f in found]), '\n'.join(lines)
    return build(tokens_of(text), bodies)


# A heredoc body is data only when every command of its pipeline is one of these readers (an allowlist: an
# unlisted interpreter, debugger or database shell such as ipython, gdb -batch or sqlite3 may run it).
READS_STDIN = re.compile(r'codex-task|codex-ideate|cat|head|tail|wc|sort|uniq|cut|tr|grep|true|:')
STDIN_SINKS = re.compile(r'\(\s*\)|\balias\b|\bfunction\b')    # a function or alias can hide the reader


def code_of(command, cwd=None):
    """``command`` without the heredoc bodies that are only data: the body of ``codex-task N <<'EOF'`` is a
    prompt, not shell, and ``cat > notes.md <<EOF`` writes a file. A body stays when anything could run it: a
    command in its pipeline that is not a known data reader (``READS_STDIN``), a file written there that is not
    plain data (``data_file``: an allowlist of data files in notes, scratch and worktree dirs, written by a call that
    does nothing else), a pipe
    continued past the line, a ``$(...)``, backtick or ``<(...)`` on the line or (unquoted delimiter) in
    the body, a word the hook cannot read (``$X``), or a function or alias in the call."""
    text = command.replace('\\\n', '')
    try:
        lines, found = heredocs(command)
    except ValueError:                  # a heredoc the hook cannot read as bash would: all of it is code
        return command
    if STDIN_SINKS.search('\n'.join(lines)):     # a function or alias defined in the call (bodies are text)
        return command
    out = []
    for n, line in enumerate(lines):
        out.append(line)
        mine = [f for f in found if f[4] == n]
        if not mine:
            continue
        try:
            pipelines = parse(line, iter([(f'\0{k}', True) for k in range(len(mine))]))
        except ValueError:
            pipelines = []
        risky = bool(re.search(r'\$\(|`|[<>]\(', line) or re.search(r'\|\s*$', line))
        for k, (dash, word, quoted, body, _, end) in enumerate(mine):
            pipe = next((p for p in pipelines if any(f'\0{k}' in c['stdin'] for c in p)), None)
            data = pipe is not None and not risky and ((not (not quoted and re.search(r'\$\(|`', body)) and not any(
                (c['writes'] and not data_file(c, text, cwd)) or any('$' in w for w in c['argv'])
                or not READS_STDIN.fullmatch(os.path.basename(next(
                    (w for w in c['argv'] if not re.fullmatch(r'\w+=\S*', w)), '')))
                for c in pipe)) or (py_script(pipe, where(text, cwd), text)
                                    and (quoted or not re.search(r'[$`\\]', body)) and py_inert(body, where(text, cwd))))
            out += ([] if data else body.split('\n') + list(filter(None, [py_text(body)]))) + [end]
    return '\n'.join(out)


def py_text(code):
    """The string literals of python ``code`` joined on one line ('' unless it parses), so the deny rules see a list
    argv (``subprocess.run(['ros2', 'topic', 'pub', ...])``, ``os.execvp('ros2', [...])``) as the command it runs."""
    import ast
    try:
        tree = ast.parse(code)
    except (SyntaxError, ValueError):
        return ''
    return ' '.join(' '.join(n.value.split()) for n in ast.walk(tree)
                    if isinstance(n, ast.Constant) and isinstance(n.value, str))


def py_inline(command):
    """``py_text`` of each ``python -c CODE`` in the call, one line each."""
    try:
        pipelines = parse(command)
    except ValueError:
        return ''
    out, todo = [], [c for p in pipelines for c in p]
    while todo:
        cmd = todo.pop()
        todo += [c for sub in cmd['subs'] for p in sub for c in p]
        words = cmd['argv']
        if any(re.fullmatch(r'(\S*/)?python[\d.]*', w) for w in words):     # sudo, env -i, /usr/bin/python3 ...
            out += [py_text(words[i + 1]) for i, w in enumerate(words[:-1]) if re.fullmatch(r'-[A-Za-z]*c', w)]
    return '\n'.join(out)


WRAPPERS = {'nohup', 'setsid', 'nice', 'env', 'timeout', 'stdbuf', 'time', 'exec'}


def program(argv):
    """The program a command runs: its first word after ``VAR=x`` prefixes, wrappers (nohup, env, nice ...) and their
    flags."""
    words = list(argv)
    while words and (re.fullmatch(r'\w+=\S*', words[0]) or words[0] in WRAPPERS
                     or (len(words) < len(argv) and re.fullmatch(r'-\S*|\d+\w*', words[0]))):
        words = words[1:]
    return os.path.basename(words[0]) if words else ''


def data_file(cmd, command, cwd=None):
    """``cat > T <<EOF`` only stores its body, an allowlist that fails closed: T is a data file (``data_target``) that
    nothing else in the call touches (``alone``; a reader such as codex-task may take it on stdin). Anything else keeps
    its body, searched as shell."""
    return alone(cmd, command) and data_target(cmd['writes'][0], cwd)


def alone(cmd, command, readers=True):
    """``cmd`` is ``cat > T`` and nothing else in the call can name, copy, glob, chmod or run T: no other command,
    heredoc or redirection names it (with ``readers``, a reader in READS_STDIN taking it on stdin aside), and no word
    holds a ``$``, a glob or a substitution."""
    try:
        pipelines, lines = parse(command), heredocs(command)[0]
    except ValueError:
        return False
    if cmd['argv'] not in (['cat'], ['cat', '-']) or len(cmd['writes']) != 1 or cmd['subs'] or re.search(
            r'\$\(|`|[<>]\(', '\n'.join(lines)):          # a substitution on a command line (bodies: code_of)
        return False
    target = cmd['writes'][0]
    name = os.path.basename(target)
    others = [c for p in pipelines for c in p if c['writes'] != cmd['writes'] or c['argv'] != cmd['argv']]
    for c in [c for p in pipelines for c in p]:
        if any(re.search(r'[$*?\[]', w) for w in c['argv'] + c['writes'] + c['reads']):
            return False
    for c in others:
        if any(name in w for w in c['argv'] + c['writes'] + c['stdin']) or (
                any(name in w for w in c['reads']) and not (readers and READS_STDIN.fullmatch(program(c['argv'])))):
            return False
    return True


def data_target(target, cwd=None):
    """A file a call may write as data: a plain literal name with a data extension (``DATA_EXT``), not a protected file
    (``FILES``), resolving (realpath, after ``~`` and ``cwd``) under ``data_roots`` and outside any ``/.git/``."""
    if target == '/dev/null':
        return True
    if not re.fullmatch(r'~?[\w./+-]+', target):
        return False
    if target.startswith('~'):
        if not target.startswith('~/'):
            return False
        target = str(Path.home()) + target[1:]
    if not os.path.isabs(target) and not cwd:
        return False
    real = os.path.realpath(os.path.join(cwd or '/', target))
    if '/.git/' in real + '/' or '/.git/' in target or any(
            FILES.search(p) or re.search(r'\bhold_store\b|command_mux|/mhseals(/|$)', p) for p in (real, target)):
        return False
    if os.path.splitext(real)[1].lower() not in DATA_EXT:
        return False
    return any(real.startswith(root) for root in data_roots())


# A python script (``python3 - <<EOF``, or ``cat > x.py <<EOF``: what it does when run) is data when nothing in it can
# act on the boat or a protected file (``py_inert``): it parses, imports only PY_SAFE modules, calls nothing that runs a
# program, signals a process, changes directory or reaches past the names it shows (PY_OPAQUE), and every file it
# writes, moves or deletes is a literal path (constants, Path(...) / ..., os.path.join, f-strings of constants) that is
# a data file (``data_target``). Anything else keeps its body, searched as shell.
PY_SAFE = {'json', 're', 'os', 'sys', 'pathlib', 'collections', 'itertools', 'math', 'statistics', 'csv',
           'glob', 'fnmatch', 'datetime', 'time', 'textwrap', 'hashlib', 'yaml', 'typing', 'dataclasses', 'shutil',
           'difflib', 'pprint', 'string', 'bisect', 'heapq', 'decimal', 'fractions', 'enum', 'copy',
           'tomllib', 'configparser', 'argparse', 'ast', 'unicodedata', 'statistics', '__future__'}
PY_OPAQUE = {'exec', 'eval', 'compile', '__import__', 'getattr', 'setattr', 'delattr', 'globals', 'locals', 'vars',
             'breakpoint', 'system', 'popen', 'kill', 'killpg', 'fork', 'forkpty', 'chdir', 'fchdir', 'startfile',
             'putenv', 'pidfd_send_signal', 'modules', 'unsafe_load', 'full_load', 'Loader', 'UnsafeLoader', 'FullLoader',
             'unpack_archive', 'make_archive', 'help'}
PY_WRITES = {'write_text', 'write_bytes', 'unlink', 'rmdir', 'touch', 'chmod', 'lchmod', 'chown', 'lchown', 'symlink_to',
             'hardlink_to', 'link_to', 'rename', 'renames', 'replace', 'truncate', 'rmtree', 'remove', 'removedirs',
             'copy', 'copy2', 'copyfile', 'copytree', 'copymode', 'copystat', 'move', 'utime', 'symlink', 'link',
             'mkfifo', 'mknod'}
# A Path's (or a handle's) own writes: the receiver and, for a link or move, its argument.
PY_PATH_WRITES = {'write_text', 'write_bytes', 'unlink', 'rmdir', 'touch', 'chmod', 'lchmod', 'symlink_to', 'hardlink_to',
                  'link_to', 'rename', 'copy_into', 'move_into'}
# What a script may take from a module, an allowlist: a module named here gives only these names (its writes and open
# only as path-checked ``module.name(...)`` calls, never from-imported); any other PY_SAFE module gives its public names
# that are not modules. A module is only ever an attribute's receiver (no alias, no bare value), and the only hops to
# another module are PY_SUBMODULES, so no other module, and none of its calls, is reachable.
PY_ATTRS = {
    'os': PY_WRITES | {'open', 'path', 'environ', 'getenv', 'getcwd', 'listdir', 'scandir', 'walk', 'stat', 'lstat', 'sep',
                       'linesep', 'pathsep', 'curdir', 'pardir', 'devnull', 'name', 'getpid', 'cpu_count', 'fspath',
                       'fsencode', 'fsdecode', 'makedirs', 'mkdir', 'access', 'F_OK', 'R_OK', 'W_OK', 'X_OK', 'PathLike',
                       'get_terminal_size', 'O_RDONLY', 'O_WRONLY', 'O_RDWR', 'O_CREAT', 'O_APPEND', 'O_TRUNC', 'O_EXCL'},
    'shutil': PY_WRITES | {'which', 'disk_usage', 'get_terminal_size', 'copyfileobj', 'Error', 'SameFileError'},
    'sys': {'argv', 'stdin', 'stdout', 'stderr', 'exit', 'version', 'version_info', 'platform', 'executable', 'maxsize',
            'byteorder', 'getsizeof', 'getrecursionlimit', 'setrecursionlimit', 'getdefaultencoding', 'float_info'},
    'yaml': {'safe_load', 'safe_load_all', 'safe_dump', 'safe_dump_all', 'dump', 'dump_all', 'SafeLoader', 'CSafeLoader',
             'SafeDumper', 'CSafeDumper', 'Dumper', 'YAMLError'},
    'pathlib': {'Path', 'PurePath', 'PosixPath', 'PurePosixPath'},
    'string': {'ascii_letters', 'ascii_lowercase', 'ascii_uppercase', 'digits', 'hexdigits', 'octdigits', 'punctuation',
               'printable', 'whitespace', 'Template', 'capwords'},
    'argparse': {'ArgumentParser', 'Namespace', 'SUPPRESS', 'ArgumentError', 'ArgumentTypeError', 'HelpFormatter',
                 'RawDescriptionHelpFormatter', 'RawTextHelpFormatter', 'ArgumentDefaultsHelpFormatter',
                 'BooleanOptionalAction'},
    'typing': {'Any', 'Optional', 'Union', 'List', 'Dict', 'Tuple', 'Set', 'FrozenSet', 'Iterable', 'Iterator', 'Sequence',
               'Mapping', 'MutableMapping', 'Callable', 'NamedTuple', 'TypedDict', 'Literal', 'TYPE_CHECKING', 'cast',
               'TypeVar', 'Generic', 'Protocol', 'ClassVar', 'Final', 'IO', 'TextIO', 'overload', 'Annotated', 'Type',
               'DefaultDict', 'Counter', 'Deque', 'Generator', 'Self', 'NoReturn', 'Never'}}
PY_SUBMODULES = {'os.path', 'collections.abc'}
# A write call's keywords that name no path; any other keyword (dst=, target=, **kw) is a path, checked.
PY_KW_OK = {'encoding', 'errors', 'newline', 'exist_ok', 'missing_ok', 'parents', 'follow_symlinks', 'dirs_exist_ok',
            'ignore_errors', 'mode'}
# Attributes that reach a module from an object that is not one (Path('x').parser.os): never.
PY_HOPS = {'os', 'posix', 'sys', 'builtins', 'bltns', 'subprocess', 'shutil', 'importlib', 'inspect', 'operator',
           'functools', 'types', 'signal', 'socket', 'ctypes', 'pty', 'annotationlib', 'io', 'codecs', 'parser'}


def py_attr(module, name):
    """``module.name`` is one a script may use (PY_ATTRS above)."""
    import inspect
    if f'{module.__name__}.{name}' in PY_SUBMODULES:
        return True
    if module.__name__ in PY_ATTRS:
        return name in PY_ATTRS[module.__name__]
    return not name.startswith('_') and hasattr(module, name) and not inspect.ismodule(getattr(module, name))


def py_modules(tree):
    """``{name: module}`` the script's imports bind, or None unless each is plain: a PY_SAFE module or PY_SUBMODULES,
    no alias of a module, no ``*``, from-imports only of names ``py_attr`` allows, writes and open aside."""
    import ast
    import importlib
    import inspect
    mods = {}
    try:
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.asname or alias.name not in PY_SAFE | PY_SUBMODULES:
                        return None
                    importlib.import_module(alias.name)
                    mods[alias.name.split('.')[0]] = importlib.import_module(alias.name.split('.')[0])
            elif isinstance(node, ast.ImportFrom):
                if node.level or node.module not in PY_SAFE | PY_SUBMODULES:
                    return None
                module = importlib.import_module(node.module)
                for alias in node.names:
                    if alias.name == '*' or alias.name in PY_WRITES | {'open'} or not py_attr(module, alias.name):
                        return None
                    if inspect.ismodule(getattr(module, alias.name)):
                        mods[alias.asname or alias.name] = getattr(module, alias.name)
    except ImportError:
        return None
    return mods


def py_script(pipe, cwd=None, command=''):
    """The heredoc of this pipeline is a python program: ``python3 -`` (or no script; flags -u -B -I -E -s -S only, stdout
    only into data files), or ``cat > NAME.py`` (a literal name, not a protected file) that nothing else in the call
    names (``alone``: a shell cannot run it as shell)."""
    if len(pipe) != 1:
        return False
    cmd = pipe[0]
    words = [w for w in cmd['argv'] if not re.fullmatch(r'\w+=\S*', w)]
    if words and re.fullmatch(r'python3?(\.\d+)?', os.path.basename(words[0])):
        return (all(w in ('-', '-u', '-B', '-I', '-E', '-s', '-S') for w in words[1:]) and not cmd['reads']
                and all(data_target(t, cwd) for t in cmd['writes']))
    return (words in (['cat'], ['cat', '-']) and len(cmd['writes']) == 1 and cmd['writes'][0].endswith('.py')
            and bool(re.fullmatch(r'~?[\w./+-]+', cmd['writes'][0])) and not FILES.search(cmd['writes'][0])
            and '.git/' not in cmd['writes'][0] and alone(cmd, command, readers=False))


def py_inert(code, cwd=None):
    """True when the python ``code`` can only read and write data files (see PY_SAFE above); any doubt is False."""
    import ast
    try:
        tree = ast.parse(code)
    except (SyntaxError, ValueError):
        return False
    mods = py_modules(tree)
    if mods is None:
        return False

    def module_of(node):            # the module an expression names (os, os.path), else None
        if isinstance(node, ast.Name):
            return mods.get(node.id)
        base = module_of(node.value) if isinstance(node, ast.Attribute) else None
        value = getattr(base, node.attr, None) if base is not None else None
        return value if type(value) is type(os) else None
    stores, called = {}, {id(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    receivers = {id(n.value) for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            base = module_of(node.value)
            if not py_attr(base, node.attr) if base is not None else node.attr in PY_HOPS:
                return False
        if module_of(node) is not None and id(node) not in receivers:     # a module as a value: m = os; f(shutil)
            return False
        if id(node) not in called and (isinstance(node, ast.Name) and node.id == 'open' or isinstance(
                node, ast.Attribute) and (node.attr in PY_WRITES | {'open'} or ast.unparse(node) == 'sys.path')):
            return False
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or '']
            if node.__class__ is ast.ImportFrom and node.level or any(n.split('.')[0] not in PY_SAFE for n in names):
                return False
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            stores[node.id] = stores.get(node.id, 0) + 1
        elif isinstance(node, ast.Attribute) and (node.attr in PY_OPAQUE or node.attr.startswith(('__', 'exec', 'spawn',
                                                                                                    'posix_spawn'))):
            return False
        elif isinstance(node, ast.Name) and (node.id in PY_OPAQUE or node.id.startswith('__') and node.id != '__name__'):
            return False
        # every other binding: a parameter, except/match name or def/class shadows a top-level name (def f(P): ...)
        name = (node.arg if isinstance(node, ast.arg) else node.rest if isinstance(node, ast.MatchMapping)
                else getattr(node, 'name', None) if isinstance(node, (ast.ExceptHandler, ast.MatchAs, ast.MatchStar,
                                                                    ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
                else None)
        if name:
            stores[name] = stores.get(name, 0) + 1
    if any(stores.get(name) for name in ('Path', 'PurePath', 'str', 'open')):    # what py_path and open read as builtins
        return False
    env = {}
    for node in tree.body:              # names bound once, at the top, to a value the hook can read
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                and stores.get(node.targets[0].id) == 1):
            value = py_path(node.value, env)
            if value is not None:
                env[node.targets[0].id] = value
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func, targets = node.func, None
        mod = isinstance(func, ast.Attribute) and module_of(func.value) is not None
        keywords = [k.value for k in node.keywords if k.arg not in PY_KW_OK]           # dst=, target=, **kw
        if isinstance(func, (ast.Name, ast.Attribute)) and (func.id if isinstance(func, ast.Name) else func.attr) == 'open':
            method = isinstance(func, ast.Attribute) and not mod            # Path.open(mode)
            args = node.args[0:] if method else node.args[1:]
            mode = next((k.value for k in node.keywords if k.arg in ('mode', 'flags')), args[0] if args else None)
            if any(k.arg is None for k in node.keywords) or any(isinstance(a, ast.Starred) for a in node.args):
                mode = ast.Name('flags')                                    # open(p, **kw), open(*a): a write
            if isinstance(func, ast.Attribute) and ast.unparse(func.value) == 'os':
                mode = None if mode is None else ast.Name('flags')       # os.open(path, flags): always a write
            if mode is None or (isinstance(mode, ast.Constant) and isinstance(mode.value, str)
                                and not set(mode.value) & set('wax+')):
                continue
            targets = [func.value] if method else node.args[:1]
        elif isinstance(func, ast.Attribute) and mod and func.attr in PY_WRITES:      # os.remove, shutil.copy ...
            targets = (node.args[1:2] if func.attr.startswith('copy') else node.args) + keywords
        elif isinstance(func, ast.Attribute) and (func.attr in PY_PATH_WRITES or (      # a Path's own writes
                func.attr in ('replace', 'copy', 'move') and len(node.args) + len(node.keywords) == 1)):   # not str.replace(a, b), dict.copy()
            targets = [func.value] + (node.args if func.attr in PY_PATH_WRITES - {'write_text', 'write_bytes'}
                                      or func.attr in ('replace', 'copy', 'move') else []) + keywords
        if targets is not None:
            for target in targets or [None]:
                path = py_path(target, env) if target is not None else None
                if path is None or not data_target(path, cwd):
                    return False
    return True


def py_path(node, env):
    """The literal path a python expression names, or None: str constants, f-strings of them, names bound once to one,
    ``+``, ``Path(...)``/``PurePath``, ``/``, ``os.path.join``, ``str()`` and ``.expanduser()``/``.resolve()``."""
    import ast
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name):
        return env.get(node.id)
    if isinstance(node, ast.JoinedStr):
        parts = [py_path(v.value if isinstance(v, ast.FormattedValue) else v, env) for v in node.values]
        return None if None in parts else ''.join(parts)
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Div)):
        left, right = py_path(node.left, env), py_path(node.right, env)
        if left is None or right is None:
            return None
        return left + right if isinstance(node.op, ast.Add) else os.path.join(left, right)
    if isinstance(node, ast.Call) and not node.keywords:
        name = ast.unparse(node.func)
        args = [py_path(a, env) for a in node.args]
        if None in args:
            return None
        if name in ('Path', 'PurePath', 'pathlib.Path', 'os.path.join', 'str') and args:
            return os.path.join(*args)
        if isinstance(node.func, ast.Attribute) and node.func.attr in ('expanduser', 'resolve', 'absolute') and not args:
            inner = py_path(node.func.value, env)
            return None if inner is None else os.path.expanduser(inner)
        if name in ('Path.home', 'pathlib.Path.home') and not args:
            return str(Path.home())
    return None


SCRATCH = '/tmp'
DATA_EXT = {'.md', '.txt', '.json', '.yaml', '.yml', '.csv', '.log', '.tex', '.patch', '.diff'}


def data_roots():
    """Where a call may write data: astro_dock's .handoff (notes and briefs), /tmp and ~/.cache (scratch, its linked git
    worktrees included; protected names stay out by ``FILES``), each as its realpath with a trailing /."""
    home = Path.home()
    roots = [home / 'astro_dock' / '.handoff', Path(SCRATCH), home / '.cache']
    try:
        roots += [d for d in (home / '.cache' / 'wt').iterdir() if (d / '.git').is_file()]
    except OSError:
        pass
    return [os.path.realpath(r).rstrip('/') + '/' for r in roots]


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


# An agent's own tmux session on a boat host (default server, a boatops-<who>*/agent-<who>* name: it carries the
# calling agent's id; nothing after the target): ending it is not stopping the boat's software. Never `boat` (the TUI's
# session), another agent's session, -L/-S sockets or -a.
OWN_TMUX = r"""\btmux\s+kill-session\s+-t\s*(['"]?)=?(?:boatops|agent)-%s(?:[-_.][\w.-]*)?\1(?=[ \t]*($|[;&|)\n'"]))"""
# The free text of boat_agent ask/note (the question or note) is data. Shell commands nest (ssh "...", bash -c "..."),
# so its quotes may be literal at one level and quotes at the next: the text counts as data only when it holds nothing
# any level could act on (no quote of either kind, $, backtick, backslash, operator or newline), and only when the
# whole command has no command or parameter substitution anywhere ($(, backticks, ${): otherwise it is checked as code.
AGENT_TEXT = re.compile(r"""(\bboat_agent[ \t]+(?:ask|note)[ \t]+)("[^"'$`\\;&|<>()\n]*"|'[^"'$`\\;&|<>()\n]*')""")
EXPANDS = re.compile(r'\$\(|`|\$\{')


def own_text(command, who=None):
    """``command`` with the calling agent's (``who``) own boat tmux session ends and its boat_agent ask/note text
    blanked (``AGENT_TEXT``)."""
    if who:
        command = re.sub(OWN_TMUX % re.escape(who), 'tmux-end-own-session', command)
    return command if EXPANDS.search(command) else AGENT_TEXT.sub(r'\1TEXT', command)


# ``-c core.editor=true`` or ``GIT_EDITOR=true`` (sequence.editor, GIT_SEQUENCE_EDITOR; ``true`` or ``:``), its value one
# whole shell word: the non-interactive continue of a cherry-pick, rebase or merge. Any other value (true;x, "true $(x)")
# stays a program of the caller's choosing (GIT_RUNS).
BENIGN_GIT = re.compile(r'''(?<![\w$-])(?:-c[ \t]*(['"]?)(?:core|sequence)\.editor=(['"]?)(?:true|:)\2\1'''
                        r'''|GIT_(?:SEQUENCE_)?EDITOR=(['"]?)(?:true|:)\3)(?=[ \t])''')
# A command runs on a boat computer when this is one, or when the call reaches one: a boat named in a call with a
# remote shell or transfer (ssh and kin, a remote docker or podman; BOAT_AGENT_TARGET=HOST boat_agent, which makes the
# ssh hop itself, as ssh HOST boat_agent). A boat name in a path, a profile or a container's arguments (docker exec
# crane_autodemo ... blastoise) does not move the command off this machine.
REMOTE = re.compile(r'(?<![\w.-])(ssh|mosh|autossh|sshpass|dbclient|plink|pdsh|pssh|parallel-ssh|clush|ansible[\w-]*'
                    r'|salt[\w-]*|telnet|nc|ncat|socat|curl|wget|scp|sftp|rsync|rsh|rexec|rlogin|paramiko|fabric'
                    r'|asyncssh|pexpect|pxssh|netmiko|telnetlib|socket|sshfs|et|kubectl|tsh)(?![\w.-])|ssh://'
                    r'|\bBOAT_AGENT_TARGET='
                    r'|\b(docker\s+context\s+use|podman\s+system\s+connection\s+default)\b'
                    r'|\b(DOCKER_HOST|DOCKER_CONTEXT|CONTAINER_HOST|CONTAINER_CONNECTION)='
                    r'|\b(docker|podman)\b[^;&|\n]*\s(-H|--host|--context|-c|--url|--connection|--remote)\b')


def denied(command, config, cwd=None, who=None):
    """``(what it blocks, the boat it names or None)``, else None. Tier A (``text_only``) is never
    denied, and a heredoc body that is only data (``code_of``) is not searched;
    anything else is denied when ``hook.deny`` is found anywhere in its text, or
    ``hook.deny_boat`` when it names a boat (or runs on one)."""
    command = BENIGN_GIT.sub('', command)
    if text_only(command, cwd):
        return None
    text = '\n'.join(filter(None, [code_of(command, cwd), py_inline(command)]))    # python list argv as words
    text, names = links(plain(failed_cd(own_text(text, who))), cwd), boats(config)
    alternatives = [re.escape(n) for n in sorted(names, key=len, reverse=True)]
    if network_file() is None:      # no network env (a checkout without astro_dock): fail closed, any private-LAN address
        alternatives.append(r'(?:192\.168|10|172\.(?:1[6-9]|2\d|3[01]))(?:\.\d{1,3}){2,3}')
    found = re.search(r'(?<![\w.-])(%s)(?![\w-])' % '|'.join(alternatives), text) if alternatives else None
    here = hostname().split('.')[0]
    host = found.group(1) if found and REMOTE.search(text) else here if here in names else None
    live_only = set(SIM_ARM[1:] + HOLDS + STOP + LAUNCHES + HOOKS)   # file rules: a dev copy (a worktree) is development
    for pattern in config['deny'] + (config['deny_boat'] if host else []):
        for match in re.finditer(pattern, text, re.S):
            one_liner = re.match(r'python[\d.]*\s', text[match.start():])    # its ; are python's: to the line end
            body = one_liner and HEREDOC.search(text[match.start():].split('\n', 1)[0])     # python - <<EOF: its body
            rest = re.compile(r'\n%s(?=\n|$)' % re.escape(body.group(2)) if body else r'\n' if one_liner
                              else r'[\n;|&]').search(text, match.end())
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


MOVES = re.compile(r'(?<![\w-])(?:cd|pushd)\b|\benv\b[^;&|\n]*\s(?:-[A-Za-z]*C|--chdir\b)|\bchdir\(')
_HOME = r'(?:~|\$HOME\b|\$\{HOME\})(?=/|[\s;&|)]|$)'      # quotes are gone (plain): "$HOME/x", '~'/x
_DIR = r'((?:[/~]|\$HOME\b|\$\{HOME\})[^\s;&|`*?\[()]*)'


_LEAD_CD = re.compile(r'(\s*(?:cd|pushd)\s+)(\S+?)[ \t]*(?:;|\n)')


def failed_cd(command):
    """``command`` with a NUL before the target of a leading ``cd DIR ;`` (or newline) that fails in the shell, so
    ``where`` keeps the old directory: a DIR that does not exist (after unquoted ``~``/``$HOME`` expansion) or one
    quoted with a literal ``~`` or ``$`` (``cd '~/x' ;``). After ``&&`` a failed cd stops the chain, so it is left."""
    lead = _LEAD_CD.match(command)
    if not lead:
        return command
    word = lead.group(2)
    if not re.search(r"'[^']*[~$]|\"~", word):
        found = re.sub('^' + _HOME, lambda m: str(Path.home()), plain(word))
        if '$' in found or not found.startswith('/') or os.path.isdir(found):
            return command                              # a variable or relative name: where fails closed as before
    return command[:lead.end(1)] + '\0' + command[lead.end(1):]


def where(whole, cwd):
    """The directory relative names resolve in: the call's ``cwd``, or the literal directory of its only change
    of directory when that is a leading ``cd DIR`` then ``&&``, ``;`` or a newline (``cwd`` when ``failed_cd`` marked
    it), a leading ``env -C DIR`` or a python ``os.chdir(DIR)`` (``~``, ``$HOME`` and ``${HOME}`` expanded, any
    other ``$`` fails closed); None when it changes directory otherwise (fail closed)."""
    moves = MOVES.findall(whole)
    if len(moves) == 1 and re.match(r'\s*(?:cd|pushd)\s+\0', whole):             # failed_cd: it never moved
        return cwd
    lead = (re.match(r'\s*(?:cd|pushd)\s+' + _DIR + r'[ \t]*(?:&&|;|\n)', whole)
            or re.match(r'\s*env\s+(?:-\S+\s+)*?(?:-C\s*|--chdir[= ])' + _DIR + r'\s', whole)
            or re.search(r'\bchdir\(\s*' + _DIR + r'\s*\)', whole))
    if lead and len(moves) == 1:
        found = re.sub('^' + _HOME, lambda m: str(Path.home()), lead.group(1))
        return None if re.search(r'[${}~]', found) else found
    return None if moves else cwd


def resolve(name, base):
    """``name`` as absolute paths (as given and its symlink target), or None for a $variable, a glob or a relative
    name with no ``base``."""
    if re.search(r'[$*?\[{`]', name) or (not name.startswith(('/', '~')) and not base):
        return None
    path = Path(os.path.expanduser(name))
    if not path.is_absolute() and not base:                         # ~other/x with no such user
        return None
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


# What makes a directory protected (``holds_live``): the file itself, a protected file in it, a config dir with
# agent.yaml, a launch dir, a package's launch dir, the hook's wiring.
PROTECTED = ('', '*', 'config/agent.y*ml', '*/config/agent.y*ml', 'launch/*', '*/launch/*', '.agents/hook.py',
             '.claude/settings*.json', '.codex/hooks.json')


def holds_live(path, globs=PROTECTED):
    """``path`` is, or holds (``globs``), a live protected file (``FILES``: agent.yaml, a launch file, the hook)."""
    for glob in globs:
        for found in (Path(path).glob(glob) if glob else [Path(path)]):
            if FILES.fullmatch(str(found)) and found.exists() and place(str(found)) != 'dev':
                return True
    return False


def into_live(text, cwd=None, whole=''):
    """A write in ``text`` into a directory that is or holds a live protected dir (or one it cannot read: a
    $variable, a remote ...config/ dir): rsync's or cp -r's destination, tar -x's or git's -C dir and pathspecs,
    git apply's and patch's dir, find's roots (only a protected dir itself: find LIVE -name x.pyc -delete is fine)."""
    words, base = text.split(), where(whole or text, cwd)
    globs = PROTECTED
    if words[:1] in (['rsync'], ['cp']):
        into = next((w.split('=', 1)[1] if '=' in w else words[i + 1] for i, w in enumerate(words[:-1])
                     if re.fullmatch(r'-[A-Za-z]*t|--target-directory(=.*)?', w)), None)
        args = [w for w in words[1:] if not w.startswith('-')]
        dirs = [into] if into else args[-1:] if len(args) >= 2 else []
    elif words[:1] == ['find']:
        dirs, globs = targets(text), ('', '*')
    elif words[:1] == ['patch']:
        found = re.search(r'(?:\s-d\s*|--directory[= ])(\S+)', text)
        dirs = [found.group(1) if found else '.']
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
        verb, rest = re.split(r'\b(checkout|restore|apply)\b', text, maxsplit=1)[-2:]
        dirs, skip = ([] if verb != 'apply' else ['.']), False
        for word in rest.split() if verb != 'apply' else ():
            if skip:
                skip = False
            elif word in ('-s', '--source', '-b', '-B'):
                skip = True
            elif not word.startswith('-'):
                dirs.append(word)
    for name in dirs:
        if re.match(r'[\w.-]+@?[\w.-]*:', name):                    # a remote destination: a config dir is live
            if re.search(r'(^|[:/])(config|launch)/?$', name):
                return True
            continue
        paths = resolve(name, base)
        if paths is None or any(holds_live(path, globs) for path in paths):
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
