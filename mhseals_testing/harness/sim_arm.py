"""Sim-only auto-arm (owner decision 2026-10-26): the checks that prove a stack is a simulator. ROS-free.

``boat_agent run KEY --sim-arm --claim ID`` asks the desk (ui/feeds/agent.py) to arm for that run
instead of raising the human arm panel. The desk arms only when every check here passes, through
``App.agent_arm`` (the arm panel's own path, without its popup), never releases a hold, and disarms
when the run ends for any reason. Each check is one a real boat cannot pass by accident:

===========  ===================================================================================
enabled      config/agent.yaml ``sim_arm.enabled``: the owner's switch
profile      ``use_sim_time``, topic outputs (PWM to the simulator, no pins), no FCU, a command mux,
             no remote (ssh) driver: a boat profile has pin outputs or an FCU
domain       ROS_DOMAIN_ID in ``sim_arm.domains``, never the fleet's 42 (hard-coded, not configurable)
discovery    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST and no ROS_STATIC_PEERS: a boat spans computers
container    /.dockerenv or /run/.containerenv
private_ipc  /proc/self/ns/ipc is not the host's initial IPC namespace (docker ``--ipc private``);
             the dev and boat containers run ``--ipc host``
clock        /clock has a publisher and the node's sim time is past 0
player       the profile's ground truth is a ``/crane/`` topic and something publishes it
no_hardware  no node named like real hardware (MAVROS, ZED, the lidar driver, the FCU watchdog, a
             thruster driver or mux already up): ``sim_arm.real_nodes``
holds        the mux hold file is the session's own (arming.session_hardware's sim branch) and holds
             nothing: a saved hold stays until the operator releases it
stop_file    ``<data root>/agent/stop`` is absent (touch it to end every sim-armed run)
===========  ===================================================================================

Anything missing or unreadable fails its check. ``refusal`` names the first failed check.
"""

from dataclasses import dataclass, field
import os
from pathlib import Path
import re

from mhseals_testing import data

BOAT_DOMAIN = 42                    # the fleet's domain: never a sim, whatever agent.yaml says
INIT_IPC = 'ipc:[4026531839]'       # PROC_IPC_INIT_INO: the host's initial IPC namespace
CONTAINER_MARKERS = ('/.dockerenv', '/run/.containerenv')
PLAYER_PREFIX = '/crane/'
HOLD_FILE = 'command_mux_holds.json'   # arming.session_hardware's per-session file
CHECKS = ('enabled', 'profile', 'domain', 'discovery', 'container', 'private_ipc', 'clock', 'player',
          'no_hardware', 'holds', 'stop_file')
DEFAULTS = {'enabled': False, 'max_s': 1800, 'domains': [101, 232],
            'real_nodes': [r'mavros\w*', r'zed_\w+', r'velodyne\w*', 'lidar_relay_publish', 'fcu_watchdog',
                           'host_diagnostic', r'ntrip\w*', r'ublox\w*', 'thruster_pwm_node', 'command_mux']}


def stop_path():
    """The stop file: while it exists nothing sim-arms, and a sim-armed run ends."""
    return data.path('agent', 'stop')


@dataclass(frozen=True)
class Facts:
    """What the checks judge; ``gather`` reads them from the live TUI."""
    use_sim_time: object = None
    outputs: str = ''
    uses_fcu: bool = True
    muxed: bool = False
    remote: bool = True
    ground_truth: str = ''
    env: dict = field(default_factory=dict)
    container: bool = False
    ipc: str = ''
    clock_publishers: tuple = ()
    sim_now: float = 0.0
    truth_publishers: tuple = ()
    nodes: tuple = ()
    hold_file: str = ''
    session: str = ''
    holds: tuple = ()
    stop_file: bool = True


def ipc_namespace():
    try:
        return os.readlink('/proc/self/ns/ipc')
    except OSError:
        return ''


def in_container():
    return any(os.path.exists(marker) for marker in CONTAINER_MARKERS)


def graph_facts(ros, ground_truth):
    """(clock publishers, sim now, ground-truth publishers, nodes) from ``ros`` (ros/node.Ros or alike)."""
    return (tuple(ros.publishers('/clock')), float(ros.now()),
            tuple(ros.publishers(ground_truth)) if ground_truth else (), tuple(ros.nodes()))


def gather(app, env=None):
    """Facts from the live app: its profile, arming, ROS graph and this process's environment."""
    from mhseals_hardware import hold_store
    from mhseals_testing.ros.arming import session_hardware
    profile, arming = app.profile, app.arming
    remote = getattr(arming, 'remote', None)
    hardware = session_hardware(profile, getattr(app.args, 'hardware_config', None) or {},
                                arming.config_path, remote)
    hold_file = str((hardware.get('command_mux') or {}).get('state_file') or '')
    truth = getattr(profile, 'ground_truth', None) or ''
    clock, now, truth_pubs, nodes = graph_facts(app.ros, truth)
    return Facts(use_sim_time=profile.use_sim_time, outputs=profile.outputs, uses_fcu=bool(profile.uses_fcu),
                 muxed=bool(profile.muxed), remote=remote is not None, ground_truth=truth,
                 env=dict(os.environ if env is None else env), container=in_container(), ipc=ipc_namespace(),
                 clock_publishers=clock, sim_now=now, truth_publishers=truth_pubs, nodes=nodes,
                 hold_file=hold_file, session=str(Path(arming.config_path).parent.parent),
                 holds=tuple(sorted(hold_store.load(hold_file))) if hold_file else (),
                 stop_file=stop_path().exists())


def domain_check(env, domains):
    raw = str(env.get('ROS_DOMAIN_ID', ''))
    if not raw.isdigit():
        return False, f'ROS_DOMAIN_ID {raw!r} is not a private sim domain'
    low, high = domains
    domain = int(raw)
    if domain == BOAT_DOMAIN or not low <= domain <= high:
        return False, f'ROS_DOMAIN_ID {domain} is outside the sim domains {low}..{high} (the fleet is {BOAT_DOMAIN})'
    return True, f'domain {domain}'


def real_nodes(nodes, patterns):
    """The nodes whose name (namespace dropped) fully matches a real-hardware pattern."""
    return sorted(n for n in nodes if any(re.fullmatch(p, n.rsplit('/', 1)[-1]) for p in patterns))


def holds_check(facts):
    """(ok, detail): the mux will load the session's own hold file, and it holds nothing."""
    own = str(Path(facts.session) / HOLD_FILE) if facts.session else None
    if facts.hold_file != own:
        return False, f"hold file {facts.hold_file or 'unset'} is not the session's own"
    if facts.holds:
        return False, f"held: {', '.join(facts.holds)}; the operator must release it (U)"
    return True, 'none held'


def checks(facts, config):
    """[(name, ok, detail)] for every check in CHECKS, in order."""
    config = {**DEFAULTS, **(config or {})}
    f, env = facts, facts.env
    profile_bad = [why for bad, why in (
        (f.use_sim_time is not True, 'use_sim_time is off'), (f.outputs != 'topic', f'outputs are {f.outputs!r}'),
        (f.uses_fcu, 'the profile uses an FCU'), (not f.muxed, 'no command mux'),
        (f.remote, 'the driver runs on a remote computer')) if bad]
    peers = env.get('ROS_STATIC_PEERS', '')
    discovery = env.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')
    found = real_nodes(f.nodes, config['real_nodes'])
    out = [('enabled', config['enabled'] is True, 'agent.yaml sim_arm.enabled'),
           ('profile', not profile_bad, '; '.join(profile_bad) or 'sim profile, topic outputs, muxed'),
           ('domain', *domain_check(env, config['domains'])),
           ('discovery', discovery == 'LOCALHOST' and not peers,
            f'ROS_AUTOMATIC_DISCOVERY_RANGE={discovery or "unset"}' + (f', ROS_STATIC_PEERS={peers}' if peers else '')),
           ('container', f.container, 'in a container' if f.container else 'not in a container'),
           ('private_ipc', bool(f.ipc) and f.ipc != INIT_IPC,
            "the host's IPC namespace" if f.ipc == INIT_IPC else f'private IPC {f.ipc}' if f.ipc else 'unreadable'),
           ('clock', bool(f.clock_publishers) and f.sim_now > 0,
            f'/clock from {", ".join(f.clock_publishers)}' if f.clock_publishers and f.sim_now > 0
            else 'no /clock publisher' if not f.clock_publishers else 'sim time not started'),
           ('player', f.ground_truth.startswith(PLAYER_PREFIX) and bool(f.truth_publishers),
            f'{f.ground_truth or "no ground truth"} from {", ".join(f.truth_publishers) or "nobody"}'),
           ('no_hardware', not found, f'real hardware nodes: {", ".join(found)}' if found else 'none'),
           ('holds', *holds_check(f)),
           ('stop_file', not f.stop_file, f'{stop_path()} exists' if f.stop_file else 'absent')]
    return [(name, bool(ok), str(detail)) for name, ok, detail in out]


def refusal(results):
    """'' when every check passed, else 'sim-arm refused: <check>: <detail>' for the first that failed."""
    failed = next(((name, detail) for name, ok, detail in results if not ok), None)
    return f'sim-arm refused: {failed[0]}: {failed[1]}' if failed else ''


def passed(results):
    return [name for name, ok, _ in results if ok]
