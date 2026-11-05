"""The rule table: what the hook denies (``DEFAULTS``: deny, deny_boat), the TUI-note filters, and the route each
deny names instead (``USE``, ``EDIT_USE``); ``hook_config`` lays config/agent.yaml ``hook:`` over them. Stdlib only."""

from pathlib import Path
import re

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'agent.yaml'


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
# the boat: fine), and the agent hook and its wiring (agent_hook.py and its hook/ package, .agents/hook.py, the agent-loop repo's
# src/*/hooks/hook.py, .claude/settings*.json, .codex/hooks.json).
HOLDS = [r'EDIT \w+ live .*(/command_mux[^/]*(\.json|\*)|/hold_store(/.*)?|/hold_store\.json)$']
STOP = [r'EDIT (change|delete) live .*/agent/stop$']
HOOKS = [r'EDIT \w+ live .*/(agent_hook\.py|\w+/hook/[^/]+\.py|\.agents/hook\.py|src/[^/]+/hooks/hook\.py|\.claude/settings(\.local)?\.json'
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
            (r'/(agent_hook\.py|hook/[^/]+\.py|hook\.py|settings(\.local)?\.json|hooks\.json)$', 'the live agent hook or its '
             'wiring (it keeps agents on boat_agent). Develop it in a git worktree and have the operator land it'),
            (r'.', "the running boat's config or launch files on a boat computer. Develop them in a git worktree; hot "
             'changes: {cli} set K=V --claim ID or reload config'))


# An agent's own tmux session on a boat host (a boatops-<who>*/agent-<who>* name: it carries the calling agent's id):
# ending it is not stopping the boat's software. Never `boat`, another agent's session, -L/-S sockets or -a.
OWN_TMUX = r'tmux\s+kill-session\s+-t\s*=?(?:boatops|agent)-%s(?:[-_.][\w.-]*)?$'


def hook_config(path=CONFIG):
    """``hook:`` of config/agent.yaml over the built-in defaults."""
    try:
        import yaml
        found = (yaml.safe_load(Path(path).read_text()) or {}).get('hook') or {}
    except Exception:          # no PyYAML, or a missing or broken file: the defaults are the shipped values
        return dict(DEFAULTS)
    return {**DEFAULTS, **{k: v for k, v in found.items() if k in DEFAULTS}}
