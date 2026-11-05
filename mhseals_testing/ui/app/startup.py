"""Startup: arguments, the session and review entry points, restarts and detaching."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import socket
import sys
import time

import yaml
from rich.console import Console
from rich.text import Text

from mhseals_hardware.configuration import config_path, configure_args, load_config, save_config
from mhseals_testing.nav2 import limits as nav2_limits
from mhseals_testing.harness import control, session as session_module, settings
from mhseals_testing.harness.components import Stack, components
from mhseals_testing.harness.remote import Units
from mhseals_testing.harness.unit import probe as unit_probe
from mhseals_testing.nav2 import Nav2File
from mhseals_testing.harness.registry import Registry
from mhseals_testing.health.probes import Facts
from mhseals_testing.health.network import NetworkRow
from mhseals_testing.history import adapters
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.devices import DeviceError
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.ros.arming import command_profile, muxed
from mhseals_testing.ui import theme
from mhseals_testing.ui.feeds.agent_desk import desk as agent_feed
from mhseals_testing.ui.feeds import authority as authority_feed, control as control_feed
from mhseals_testing.ui.feeds.issues import IssuesFeed, settings as issue_settings
from mhseals_testing.ui.review import Offline, find_session, load_session
from mhseals_testing.ui.takeover import decide

from mhseals_testing.ui.app.core import App
from mhseals_testing.ui.app.shared import Detach


@dataclass(frozen=True)
class SessionPaths:
    """Where one session keeps its files: all inside ``sessions/<id>/`` (data_layout)."""
    directory: Path
    bag: Path       # bag/NN/ segments
    report: Path
    logs: Path
    hardware: Path  # snapshot of the hardware config the driver runs with
    nav2: Path      # Nav2 params with measured limits, written by characterization
    journal: Path   # append-only events: markers, phases, results
    runs: Path      # one runs/<run_id>.json per procedure run
    session: Path   # session.json: identity and clock domain
    truth: Path     # simulator course truth, copied at start

    @classmethod
    def from_output(cls, directory):
        """The paths of the session directory ``directory``."""
        d = Path(directory)
        return cls(d, d / 'bag', d / 'analysis' / 'report.json', d / 'logs',
                   d / 'snapshots' / 'hardware.yaml', d / 'snapshots' / 'nav2.yaml',
                   d / 'events.jsonl', d / 'runs', d / 'session.json', d / 'course_truth.json')


def settle_discovery(nodes, least=0.5, most=2.0, step=0.25, clock=time.monotonic, sleep=time.sleep):
    """Wait until the ROS graph stops growing (two equal reads ``step`` apart, after ``least``), at most ``most``
    seconds; returns the nodes seen last. A quiet graph costs ``least``, not the old fixed 2 s."""
    start, seen = clock(), set(nodes())
    while clock() - start < most:
        sleep(step)
        now = set(nodes())
        if now == seen and clock() - start >= least:
            return now
        seen = now
    return seen


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    parser.add_argument('--profile', default='boat',
                        help=f'plant profile name ({", ".join(available_profiles())}) or YAML path; '
                             'boat_trial profiles explains each path; ? shows the active profile')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                        help='override a setting for this run (see harness/settings.py), e.g. '
                             'amplitude=0.4, command_mode=velocity, bench_override=on')
    parser.add_argument('--review', nargs='?', const='', default=None, metavar='SESSION',
                        help='play back a past session (path or name under the data root; '
                             'default the newest) with no ROS node and no outputs')
    parser.add_argument('--output', type=Path, default=None,
                        help='session directory (default a new sessions/<time>_<host>_<profile> under the data root)')
    seat = parser.add_mutually_exclusive_group()
    seat.add_argument('--view', action='store_true',
                      help='start as a viewer of the live session: no commands but Space and E; O takes control')
    seat.add_argument('--take', action='store_true',
                      help='take control of the live session: it stops at neutral, disarms and hands over')
    return parser


def parse_args(argv=None, *, defer_hardware=False):
    parser = build_parser()
    try:
        if defer_hardware:
            parser.add_argument('--config', help='hardware YAML (or MHSEALS_HARDWARE_CONFIG)')
            parser.add_argument('--save-config', action='store_true')
            args = parser.parse_args(argv)
            args.hardware_config = None
        else:
            args = configure_args(parser, argv)  # explicit unmanaged/parser callers
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        # Refuse to start on a broken hardware YAML, naming the file instead of a traceback.
        probe = argparse.ArgumentParser(add_help=False)
        probe.add_argument('--config')
        path = config_path(probe.parse_known_args(argv)[0].config)
        parser.error(f'{path}: {error}')
    args.settings = settings.load()
    try:
        for assignment in args.set:
            args.settings.set(assignment)
    except ValueError as error:
        parser.error(f'--set {error}')
    args.stale_after, args.bench_override = args.settings.stale_after, args.settings.bench_override
    args.base_profile = load_profile(args.profile)     # the command_mode hot switch (set_setting) starts here
    profile = command_profile(args.base_profile, args.settings.command_mode)
    if profile.components and args.review is None:
        try:            # where each part runs: refused here, before a session directory exists
            profile.placed
        except DeviceError as error:
            raise SystemExit(f'boat_test: {error}') from None
    return args, profile


def hang_up(*_):
    """SIGHUP and SIGTERM stop like Ctrl+C: neutral, disarm, flush, then exit."""
    raise KeyboardInterrupt


def restart_as(app):
    """(argv, notice) to exec after this session: O in a viewer restarts it as the owner (``--take``); a
    human takeover restarts the old owner as a viewer, told who took over. None otherwise."""
    if app is None:
        return None
    if app.restart_as == 'take':
        mode, notice = '--take', 'Taking control'
    elif app.detaching and app.handed_by:
        mode, notice = '--view', f'Control taken by {control.name(app.handed_by)}' + (
            f" ({app.handed_by['from']})" if app.handed_by.get('from') else '') + '; you are viewing'
    else:
        return None
    argv = [arg for arg in sys.orig_argv if arg not in ('--view', '--take')]
    return [*argv, mode], notice


def detach(*_):
    """SIGUSR1: the next instance is taking over."""
    raise Detach


def nav2_recorder(app):
    """Return the history writer for a live apply: the content Nav2File handed to it.

    Not the file as it is now, so an edit saved during the apply is not recorded as applied.
    """
    def record():
        applying = getattr(app().nav2_file, 'applying', None)
        if applying:
            adapters.record_value(app().history, 'nav2', 'tui:nav2-live', *applying,
                                  app().warn_history, applied='live-session')
    return record


def build_nav2_live(ros, registry, profile, app):
    """Build the live Nav2 applier for this session owner (``app``: a callable, built later)."""
    from mhseals_testing.nav2 import Nav2Live
    from mhseals_testing.nav2 import RosPorts
    ports = RosPorts(ros.node, registry, lambda phase, **fields: ros.emit(phase, **fields),
                     lambda: bool(app().armed) and muxed(profile))
    return Nav2Live(ports, me=os.getpid(), record=nav2_recorder(app))


def review_main(args, profile):
    """--review: the dashboard over a past session, with no ROS, bag recorder or outputs."""
    try:
        found = find_session(args.review)
        review = load_session(found)
    except ValueError as error:
        raise SystemExit(f'boat_test --review: {error}')
    console = Console()
    offline = Offline()
    app = App(args, profile, BoatMonitor(args.stale_after), offline, offline,
              SessionPaths.from_output(found), console, review=review)
    app.run()


def main(argv=None):
    """boat_test: a device registry the harness refuses ends with its message, not a traceback."""
    try:
        return session_main(argv)
    except DeviceError as error:
        raise SystemExit(f'boat_test: {error}') from None


def session_main(argv=None):
    args, profile = parse_args(argv, defer_hardware=True)
    if args.review is not None:
        args.hardware_config = load_config(args.config)
        return review_main(args, profile)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.harness.simsession import start_session
    from mhseals_testing.ros import arm_link
    from mhseals_testing.ros.arming import Arming
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.perception import subscribe_detections
    from mhseals_testing.ros.preflight import subscribe_tui as subscribe_preflight
    from mhseals_testing.ros.mission import TaskWatcher
    from mhseals_testing.nav2 import Nav2
    from mhseals_testing.ros.node import RosIO, node_name
    from mhseals_testing.ros.plant import Plant
    from mhseals_testing.harness.processes import ProcessManager

    from rclpy.signals import SignalHandlerOptions

    settings.save(args.settings)  # the last used values (not command mode or bench override)
    console = Console()
    registry = Registry()
    me = dict(pid=os.getpid(), node=node_name(), host=socket.gethostname(), started=time.time(),
              profile=profile.name, tty=os.ttyname(0) if os.isatty(0) else '', **control.identity())
    choice = decide(registry, console, me, mode='view' if args.view else 'take' if args.take else None)
    if choice.mode == 'quit':
        return
    watching = choice.watching if choice.mode == 'watch' else 0
    # Adopted sessions keep the recording's directory, so the report and bag stay together.
    if choice.session and choice.mode != 'own':
        paths = SessionPaths.from_output(choice.session)
    else:
        paths = SessionPaths.from_output(session_module.create(
            profile.name, directory=args.output,
            clock={'domain': 'sim' if profile.use_sim_time else 'wall', 'time_scale': 1.0},
            git=session_module.running_sources(),
            kind=session_module.kind(profile)).directory)
    if args.save_config:
        if choice.mode != 'own':
            raise ValueError('--save-config requires a fresh owner session')
        save_config(load_config(args.config), args.config)
    args.hardware_config = load_config(args.config)
    # rclpy's own SIGINT handler would shut ROS down before the graceful close could
    # send neutral and the final events; without it Ctrl+C is a KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for hangup in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(hangup, hang_up)
    signal.signal(signal.SIGUSR1, detach)
    gyros = [sensor.name for sensor in profile.sensors if sensor.kind == 'imu']
    monitor = BoatMonitor(args.stale_after, gyro_sensor=gyros[0] if gyros else None,
                          history_sensors=tuple(
                              sensor.name for sensor in profile.sensors
                              if sensor.kind == 'odometry' or sensor.name == (gyros or [None])[0]),
                          deadlines={sensor.name: sensor.deadline for sensor in profile.sensors
                                     if sensor.deadline is not None},
                          future=profile.stamp_future)
    facts = Facts()
    ros = RosIO(profile, monitor, args.stale_after, paths.journal, facts)
    # The keepers follow the registry owner, so a handover keeps the children for the adopter.
    stack_items = components(profile)
    processes = ProcessManager(paths.logs, registry=None if watching else registry)
    plant = Plant(ros, profile)
    # Parts on other computers run as units there; a watcher reads them but keeps none alive.
    # Every computer reports its own parts (its process table), so DDS discovery never sets a state.
    status = issue_settings()
    units = Units(stack_items, by=dict(host=profile.placed[0].name if stack_items else '',
                                       pid=os.getpid(), session=str(paths.directory)),
                  renew=not watching, period=float(status['period'])).begin()
    stack = Stack(stack_items, processes, time.monotonic, units, scan=unit_probe)
    ros.ghosts = stack.ghosts              # node.unique ignores the lingering names of a child we stopped
    # Off the thrusters host, arming targets it over DDS + ssh (ros/arm_link.py); a watcher never arms.
    driver_host = None if watching else arm_link.target(profile)
    remote_driver = None if driver_host is None else arm_link.RemoteDriver(
        driver_host[0], arm_link.RosLink(ros.node), driver_host[1],
        by=dict(host=profile.placed[0].name, pid=os.getpid(), session=str(paths.directory)))
    if remote_driver is not None:   # one hardware.yaml: the boat's, read now and again at each arm
        args.hardware_config = remote_driver.fetch_config() or args.hardware_config   # shown only if unread

        def adopt(config):
            args.hardware_config = config
        remote_driver.adopt = adopt
    arming = Arming(profile, ros, processes, args.hardware_config, paths.hardware, remote=remote_driver)
    app = authority = desk = None
    try:
        for name, child in choice.children.items():
            processes.adopt(name, child)
        if choice.mode == 'own':               # a stale owner's leftovers: stop, start fresh
            processes.stop_all()
        subscribe_detections(ros, facts, profile)
        if profile.uses_fcu:
            subscribe_fcu(ros, monitor, facts, profile.battery)
        subscribe_preflight(ros, facts, profile, lambda: args.hardware_config, registry)
        if watching:   # known to the owner, so it is not flagged as a foreign instance
            registry.add_guest(os.getpid(), me['node'], control.VIEW,
                               **{k: me[k] for k in ('host', 'started', 'tty', 'client', 'from')})
        else:
            registry.update(lambda state: state.update(session=str(paths.directory)))
            start_session(profile, paths.session, paths.truth)
            # The profile's components marked start come up with the session; whatever of
            # them already runs is attached to (Stack.autostart). Off the simulator nothing
            # that moves the boat auto-starts.
            if stack.start_order():
                settle_discovery(ros.nodes)   # let discovery find an already-running stack
                sim = profile.use_sim_time is True and profile.outputs == 'topic'
                for line in stack.autostart(ros.nodes(), ros.publishers(profile.odometry), sim):
                    console.print(Text(line, style=theme.WARN))
                    ros.emit('autostart', detail=line)
            registry.set_children(processes.records())
        app = App(args, profile, monitor, ros, processes, paths, console, arming, plant,
                  nav2=lambda: Nav2(ros, profile.map_frame or profile.odom_frame,
                                    profile.base_frame),
                  task_watcher=lambda: TaskWatcher(ros), pulses=plant.test_pulses,
                  registry=registry, watching=watching, facts=facts,
                  nav2_file=Nav2File(nav2_limits.path()), network=NetworkRow(),
                  nav2_live=lambda: build_nav2_live(ros, registry, profile, lambda: app), stack=stack,
                  issues=IssuesFeed(status['grace'], status['discovery_grace']))
        if not watching:
            ros.emit('takeover' if choice.mode == 'adopt' else 'session_start',
                     profile=profile.name, bag=str(paths.bag))
            app.observe_config()
            if processes.running('bag'):               # adopted: the recorder keeps going
                app.recorder.mode = 'lean'
            else:
                app.set_recording('lean')              # lean is the default at session start
            registry.set_children(processes.records())
        authority = None if watching else authority_feed.start(
            app, ros, subscribe=authority_feed.subscribe_cmd_vel, registry=registry)
        app.arbiter = authority and authority.arbiter   # the one arbiter the popups, banner and Agents setting read
        app.desk = desk = None if watching else agent_feed.start(app, ros, registry=registry)
        if not watching and not app.review:
            app.operator_start()                   # after the arbiter and desk: they pause with it
        app.authority_view = authority_feed.viewer(app, ros)   # a viewer shows the owner's record; None for the owner
        control_feed.start(app, registry, emit=None if watching else ros.emit)
        if os.environ.get('BOAT_TUI_NOTICE'):      # why this client restarted (a takeover), once
            app.say(os.environ.pop('BOAT_TUI_NOTICE'))
        app.run()
    except KeyboardInterrupt:  # before the page opened, or a second Ctrl+C while closing
        console.print('\n[yellow]Interrupted; stopping at neutral.[/]')
    finally:
        try:
            if app is not None:
                app.close()  # already done by run() unless the page never opened
            else:
                plant.neutral()
                arming.disarm()
        finally:
            if desk is not None:        # an act never outlives its TUI, nor passes to a successor
                desk.close()
            if authority is not None:   # claims end with the session unless a successor adopts them
                authority.arbiter.close(keep_claims=app.detaching)
            plant.close()
            processes.stop_all()   # a detach already released the children it hands on
            if watching:
                registry.remove_guest(os.getpid())
            else:
                registry.release(os.getpid(), keep_children=False)
            ros.close()
            rclpy.try_shutdown()
    if app is not None and app.detaching:
        console.print(f'Handed over to pid {getattr(app, "handed_to", "?")}')
    restart = restart_as(app)
    if restart:
        console.print(restart[1])
        os.environ['BOAT_TUI_NOTICE'] = restart[1]
        os.execv(sys.executable, restart[0])
    if app is not None:
        table = theme.grid({'header': '', 'style': f'bold {theme.ACCENT}'},
                                     {'header': '', 'ratio': 1}, padding=(0, 2))
        table.show_header = False
        for label, value in app.summary():
            table.add_row(label, value)
        console.print(theme.block(table, 'Session closed', expand=False))
