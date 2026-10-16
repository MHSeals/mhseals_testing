"""boat_test: the interactive test suite.

Starts passive: it watches sensors, TF and advisories and records the whole
session to one bag. Motion needs an explicit arm (H, then typing ARM) and
passes the interlocks in readiness.py on every tick; any error or Space stops
at neutral and disarms. Procedures are chosen with P (see procedures/).
"""

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import socket
import subprocess
import time

from rich.console import Console
from rich.live import Live
from rich.panel import Panel

from mhseals_hardware.configuration import configure_args, load_config
from mhseals_hardware.fcu import default_fcu_url
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled, select_option
from mhseals_hardware.thruster_mixer import validate_mixer
from mhseals_testing import health
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES, Context, Settings, unavailable
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.runner import Aborted, Runner
from mhseals_testing.ui.dashboard import VIEWS, render_dashboard

KEYS = ('Tab views  H arm/disarm  P procedures  M manual  +/- amplitude\n'
        'E export  Space STOP  Q quit')
ADVISORY_PERIOD = 1.0
STOP_KEYS = ('space',)
# Keys that leave the live dashboard to talk to the operator.
INTERACTIVE = {'h': 'arm', 'p': 'procedures', 'm': 'm', 'i': 'i'}
# Errors that stop a procedure: the plant goes neutral and outputs disarm.
STOPPING_ERRORS = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)


@dataclass(frozen=True)
class SessionPaths:
    bag: Path
    report: Path
    logs: Path
    hardware: Path  # snapshot of the hardware config the driver runs with
    nav2: Path      # Nav2 params with measured limits, written by characterization

    @classmethod
    def from_output(cls, output):
        output = Path(output)
        return cls(output, output.with_suffix('.report.json'),
                   output.parent / 'logs' / output.name,
                   output.with_suffix('.hardware.yaml'), output.with_suffix('.nav2.yaml'))


class App:
    """Dashboard and procedure loop.

    ROS, processes, arming, the plant, keys and the live view are injected so
    tests drive the whole loop with fakes.
    """

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 arming=None, plant=None, open_keys=KeyReader, live=Live,
                 clock=time.monotonic, foreground=subprocess.call, nav2=None,
                 task_watcher=None):
        self.args = args
        self.profile = profile
        self.monitor = monitor
        self.ros = ros
        self.processes = processes
        self.paths = paths
        self.console = console or Console()
        self.arming = arming
        self.plant = plant
        self.open_keys = open_keys
        self.live = live
        self.clock = clock
        self.foreground = foreground
        self.nav2_factory, self._nav2 = nav2, None
        self.watcher_factory, self._watcher = task_watcher, None
        self.settings = Settings(args.amplitude, args.baseline, args.command_duration,
                                 args.settle, args.repeats, args.observe, args.nav_distance,
                                 args.station_hold, args.station_tolerance)
        self.view = 'overview'
        self.notice = 'Passive monitor; H arms the outputs'
        self.command = (0.0, 0.0, 0.0)
        self.manual = False
        self.last_advisory = float('-inf')
        self.pwm_readback = ['not sampled'] * 4

    @property
    def armed(self):
        return self.arming is not None and self.arming.armed

    def snapshot(self):
        return self.monitor.snapshot(self.profile.required_tf)

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        return reasons

    def supervision_blockers(self):
        """What must hold while anything drives the boat, the suite or Nav2."""
        if not self.armed:
            return ['outputs are not armed']
        fault = self.arming.fault()
        return ([fault] if fault else []) + self.blockers()

    def motion_blockers(self):
        """When the suite commands, nothing else may publish the command topic."""
        return self.supervision_blockers() + (self.plant.conflicts() if self.armed else [])

    # --- advisories and rendering -------------------------------------------------

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.last_advisory < ADVISORY_PERIOD:
            return
        self.last_advisory = self.clock()
        snapshot = self.snapshot()
        if self.profile.use_sim_time and self.ros.now() == 0:
            # Without /clock every stamp looks stale; say why instead.
            self.monitor.advise('clock', 'FAIL', 'use_sim_time but no /clock received')
        else:
            self.monitor.advise('clock', *health.clock(snapshot))
        self.monitor.advise('compute', *health.read_compute(self.paths.bag.parent))
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
        exited = [name for name in self.processes.exited() if name != 'bag']
        if exited:
            self.monitor.advise('processes', 'WARN', 'exited: ' + ', '.join(exited))
        if self.profile.outputs == 'odroid':
            config = self.args.hardware_config
            level, self.pwm_readback = health.pwm_readback(config['pwm_chips'],
                                                           config['pwm_channels'])
            self.monitor.advise('PWM readback', level, ' | '.join(self.pwm_readback))

    def context(self):
        config = self.args.hardware_config
        blockers = self.motion_blockers() if self.armed else self.blockers()
        notice = self.notice if not blockers else f'{self.notice} | motion blocked: {blockers[0]}'
        return dict(
            view=self.view, armed=self.armed, notice=notice, keys=KEYS, manual=self.manual,
            command=self.command, profile=self.profile.name, units=self.profile.command_units,
            outputs=self.profile.outputs, specs=self.profile.sensors,
            host=socket.gethostname(), domain=os.environ.get('ROS_DOMAIN_ID', '0'),
            bag='RECORDING' if self.processes.running('bag') else 'OFF',
            process_summary=self.processes.summary(), bench_override=self.args.bench_override,
            channel_map=config['channel_map'], matrix=validate_mixer(config['thruster_matrix']),
            chips=config['pwm_chips'] if self.profile.outputs == 'odroid' else None,
            channels=config['pwm_channels'], pwm_readback=self.pwm_readback)

    def render(self):
        self.refresh_advisories()
        return render_dashboard(self.snapshot(), self.context(),
                                self.console.size.width, self.console.size.height)

    def export(self):
        report = build_report(self.profile, self.snapshot(), self.paths.bag,
                              self.args.hardware_config)
        path = write_report(report, self.paths.report)
        self.notice = f'Report saved: {path}'
        return path

    # --- the dashboard loop -------------------------------------------------------

    def handle(self, key):
        """Apply a dashboard key. Returns 'quit', an interactive action, or None."""
        if key in ('\t', 'right', 'left'):
            step = -1 if key == 'left' else 1
            self.view = VIEWS[(VIEWS.index(self.view) + step) % len(VIEWS)]
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.settings.amplitude = round(min(1.0, max(0.05, self.settings.amplitude + step)), 2)
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
        elif key == 'e':
            self.export()
        elif key in STOP_KEYS:
            self.stop('stopped by operator')
        elif key in ('q', 'escape'):
            return 'quit'
        elif key in INTERACTIVE:
            return INTERACTIVE[key]
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault())
        return None

    def watch(self):
        """Show the live dashboard until a key needs the terminal; returns that action."""
        with self.open_keys() as keys, self.live(self.render(), console=self.console,
                                                 screen=True, refresh_per_second=8) as view:
            while True:
                action = self.handle(keys.read(timeout=0.1))
                if action:
                    return action
                view.update(self.render())

    def run(self):
        while (action := self.watch()) != 'quit':
            try:
                self.act(action)
            except STOPPING_ERRORS as error:
                self.stop(str(error) or type(error).__name__)

    def act(self, action):
        if action == 'arm':
            if self.armed:
                self.disarm()
            else:
                self.arm()
            return
        if action == 'procedures':
            try:
                procedure = self.choose_procedure()
            except Cancelled:  # leaving the menu is not a stop
                self.notice = 'No procedure chosen'
                return
        else:  # a procedure's own shortcut
            procedure = next(p for p in PROCEDURES if p.key == action)
        self.run_procedure(procedure)

    # --- arming -------------------------------------------------------------------

    def arm(self):
        if self.arming is None:
            raise RuntimeError('this session cannot arm (no ROS)')
        # Another /cmd_vel source (Nav2) doesn't stop arming: Nav2 procedures need it, and
        # suite-commanded motion refuses to run beside it on every tick (motion_blockers).
        override = ('BENCH OVERRIDE: sensor and TF interlocks are off.\n'
                    if self.args.bench_override else '')
        what = {'odroid': 'REAL thruster outputs. Propellers submerged, area clear, '
                          'physical power cutoff within reach.',
                'topic': 'simulator thruster outputs over ROS.',
                'none': 'commands straight to the plant.'}[self.profile.outputs]
        self.console.print(Panel(f'Arm {what}\n{override}Space stops any motion.\n'
                                 'Type ARM to confirm; anything else cancels.',
                                 title=f'Arm {self.profile.name}', style='yellow'))
        if self.console.input('Confirm: ').strip() != 'ARM':
            self.notice = 'Arming cancelled'
            return
        self.arming.arm()
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.notice = 'Armed at neutral; P chooses a procedure'

    def disarm(self, reason='disarmed'):
        if self.plant is not None:
            self.plant.neutral()
        self.command = (0.0, 0.0, 0.0)
        if self.armed:
            self.arming.disarm()
            self.ros.emit('disarmed', reason=reason)
        self.notice = f'{reason}; outputs neutral'

    def stop(self, reason):
        """Neutral and disarmed, whatever happened."""
        try:
            self.disarm(reason)
        finally:
            self.monitor.event('STOP: ' + reason)

    # --- procedures ---------------------------------------------------------------

    def availability(self, procedure):
        return unavailable(procedure, self.profile, self.armed, self.args.bench_override)

    def choose_procedure(self):
        labels = []
        for procedure in PROCEDURES:
            reason = self.availability(procedure)
            labels.append(f'[{procedure.key}] {procedure.title}: '
                          + (f'[dim]{reason}[/]' if reason else procedure.description))
        label = select_option(self.console, 'Procedures (Q cancels)', labels,
                              {p.key: index for index, p in enumerate(PROCEDURES)})
        return PROCEDURES[labels.index(label)]

    def run_procedure(self, procedure):
        reason = self.availability(procedure)
        if reason:
            self.notice = f'{procedure.title}: {reason}'
            return
        self.notice = f'Running {procedure.title}'
        procedure.run(self.procedure_context())
        self.view = 'results'
        self.notice = f'{procedure.title} complete'

    def procedure_context(self):
        return Context(
            profile=self.profile, monitor=self.monitor, settings=self.settings,
            motion=self.motion, prompt=self.console.input,
            choose=lambda title, options, shortcuts=None: select_option(
                self.console, title, options, shortcuts),
            record=self.record, supervise=self.supervise, observe=self.observe,
            stale_after=self.args.stale_after,
            services=dict(
                nav2=self.nav2, task_watcher=self.task_watcher, start=self.processes.start,
                running=self.processes.running, stop=self.processes.stop,
                hardware_config=self.args.hardware_config, user_config=self.args.config,
                nav2_params=nav2_params(), nav2_overlay=self.paths.nav2,
                disarm=self.disarm, foreground=self.foreground, reload_config=self.reload_config,
                manual=dict(plant=self.plant, blockers=self.motion_blockers,
                            console=self.console, status=self.manual_status)))

    def motion(self, name, **details):
        """A Runner that commands the plant, inside the live dashboard."""
        return self.session(name, self.motion_blockers, details)

    def supervise(self, name, **details):
        """A Runner for watching Nav2 drive; the suite does not command meanwhile."""
        return self.session(name, self.supervision_blockers, details)

    def observe(self, name, **details):
        """A passive measurement window; no arming or readiness needed."""
        return self.session(name, lambda: [], details)

    def task_watcher(self):
        if self._watcher is None:
            if self.watcher_factory is None:
                raise RuntimeError('no task watcher in this session')
            self._watcher = self.watcher_factory()
        return self._watcher

    def nav2(self):
        if self._nav2 is None:
            if self.nav2_factory is None:
                raise RuntimeError('no Nav2 client in this session')
            self._nav2 = self.nav2_factory()
        return self._nav2

    @contextmanager
    def session(self, name, blockers, details):
        """Live dashboard plus a Runner for one supervised stretch."""
        def show(command, remaining, label):
            self.command = command
            self.notice = f'{label or name} | {remaining:.1f}s left | Space aborts'
            view.update(self.render())

        with self.open_keys() as keys, self.live(self.render(), console=self.console,
                                                 screen=True, refresh_per_second=10) as view:
            runner = Runner(self.plant, blockers, keys, show=show,
                            emit=self.ros.emit, clock=self.monitor.clock)
            try:
                with runner.session(name, **details):
                    yield runner
            finally:
                self.command = (0.0, 0.0, 0.0)

    def record(self, result):
        self.monitor.record_result(result)
        self.ros.emit('result', **result)
        self.export()

    def manual_status(self, label, amplitude, event_mode):
        self.manual = True
        self.notice = (f'MANUAL {amplitude:.0%} {label} | '
                       + ('multi-key' if event_mode else 'single-key fallback'))
        try:
            return self.render()
        finally:
            self.manual = False

    def reload_config(self):
        self.args.hardware_config = load_config(self.args.config)


def nav2_params():
    """mhseals_nav's checked-in Nav2 parameters, or None outside a ROS install."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config' / 'nav2_params.yaml'
    except (ImportError, KeyError):
        return None
    return path if path.exists() else None


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    parser.add_argument('--profile', default='boat',
                        help=f'plant profile name ({", ".join(available_profiles())}) or YAML path')
    parser.add_argument('--velocity-control', action='store_true',
                        help='command m/s and rad/s through calibrated_velocity_controller')
    parser.add_argument('--start-stack', action='store_true',
                        help="launch the profile's measurement stack; default attaches to running topics")
    parser.add_argument('--fcu-url', default='',
                        help='MAVROS FCU URL for --start-stack (default: detect the flight controller)')
    parser.add_argument('--start-sensors', action='store_true',
                        help='with --start-stack, also start local sensor drivers')
    parser.add_argument('--bench-override', action='store_true',
                        help='allow motion with no sensors or TF (bench test, scoring disabled)')
    parser.add_argument('--stale-after', type=float, default=2.0)
    parser.add_argument('--amplitude', type=float, default=0.25,
                        help='trial size in the profile units (effort fraction or m/s, rad/s)')
    parser.add_argument('--baseline', type=float, default=5.0, help='neutral seconds before a trial')
    parser.add_argument('--command-duration', type=float, default=3.0)
    parser.add_argument('--settle', type=float, default=5.0, help='neutral seconds after a trial')
    parser.add_argument('--repeats', type=int, default=1, help='trials per direction (1-5)')
    parser.add_argument('--observe', type=float, default=10.0,
                        help='seconds of passive measurement for perception checks')
    parser.add_argument('--nav-distance', type=float, default=4.0,
                        help='side of the waypoint square, m')
    parser.add_argument('--station-hold', type=float, default=60.0, help='station keeping, s')
    parser.add_argument('--station-tolerance', type=float, default=1.5,
                        help='station keeping radius, m')
    parser.add_argument('--output', type=Path, default=None,
                        help='session bag path (default bags/<profile>_<time>)')
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = configure_args(parser, argv)  # adds --config and the merged hardware_config
    profile = load_profile(args.profile)
    if args.velocity_control:
        profile = profile.with_velocity_control()
    if args.start_stack and profile.bringup is None:
        parser.error(f'profile {profile.name} has no bring-up; start its stack separately')
    for name in ('stale_after', 'baseline', 'command_duration', 'settle', 'observe'):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= 60:
            parser.error(f"--{name.replace('_', '-')} must be within (0, 60] seconds")
    if not 0 < args.amplitude <= 1:
        parser.error('--amplitude must be within (0, 1]')
    if not 1 <= args.nav_distance <= 50 or not 10 <= args.station_hold <= 600 \
            or not 0.2 <= args.station_tolerance <= 10:
        parser.error('--nav-distance (1-50 m), --station-hold (10-600 s) and '
                     '--station-tolerance (0.2-10 m) are out of range')
    if not 1 <= args.repeats <= 5:
        parser.error('--repeats must be within 1..5')
    args.output = args.output or Path('bags') / (
        f"{profile.name.replace('+', '_')}_{datetime.now():%Y%m%d_%H%M%S}")
    return args, profile


def bringup_command(profile, args):
    arguments = dict(profile.bringup)
    if arguments.get('fcu') != 'false':
        # The flight controller's stable /dev/serial/by-id name when one is plugged in.
        arguments['fcu_url'] = args.fcu_url or default_fcu_url()
    if args.start_sensors:
        arguments['start_sensors'] = 'true'
    return ['ros2', 'launch', 'mhseals_testing', 'measurement.launch.py',
            *(f'{key}:={value}' for key, value in arguments.items())]


def main(argv=None):
    args, profile = parse_args(argv)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.ros.arming import Arming
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.mission import TaskWatcher
    from mhseals_testing.ros.nav2 import Nav2
    from mhseals_testing.ros.node import RosIO
    from mhseals_testing.ros.plant import Plant
    from mhseals_testing.ros.processes import ProcessManager, bag_command, unique_path

    paths = SessionPaths.from_output(unique_path(args.output))
    console = Console()
    rclpy.init()
    monitor = BoatMonitor(args.stale_after, history_sensors=tuple(
        sensor.name for sensor in profile.sensors if sensor.kind == 'odometry'))
    ros = RosIO(profile, monitor, args.stale_after)
    processes = ProcessManager(paths.logs)
    plant = Plant(ros, profile)
    arming = Arming(profile, ros, processes, args.hardware_config, paths.hardware)
    app = None
    try:
        if profile.bringup and profile.bringup.get('fcu') != 'false':
            subscribe_fcu(ros, monitor)
        paths.bag.parent.mkdir(parents=True, exist_ok=True)
        processes.start('bag', bag_command(paths.bag))
        if args.start_stack:
            time.sleep(2.0)  # let discovery find an already-running stack
            if ros.publishers(profile.odometry):
                raise RuntimeError(f'{profile.odometry} already has publishers; '
                                   'attach without --start-stack')
            processes.start('measurement', bringup_command(profile, args))
        app = App(args, profile, monitor, ros, processes, paths, console, arming, plant,
                  nav2=lambda: Nav2(ros, profile.map_frame or profile.odom_frame,
                                    profile.base_frame),
                  task_watcher=lambda: TaskWatcher(ros))
        ros.emit('session_start', profile=profile.name, bag=str(paths.bag))
        app.run()
    except KeyboardInterrupt:
        console.print('\n[yellow]Interrupted; stopping at neutral.[/]')
    finally:
        try:
            if app is not None:
                app.stop('session ended')
                ros.emit('session_stop')
                app.export()  # before the bag stops, so the report names a complete bag
            else:
                plant.neutral()
                arming.disarm()
        finally:
            plant.close()
            processes.stop_all()
            ros.close()
            rclpy.try_shutdown()
    console.print(f'Session bag: {paths.bag}\nReport: {paths.report}')


if __name__ == '__main__':
    main()
