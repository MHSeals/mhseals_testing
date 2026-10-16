"""boat_test: the interactive test suite.

Starts passive: it watches sensors, TF and advisories and records the whole
session to one bag. Procedures are chosen from the dashboard; motion needs an
explicit arm and passes the interlocks in readiness.py on every tick.
"""

import argparse
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import socket
import time

from rich.console import Console
from rich.live import Live

from mhseals_hardware.configuration import configure_args
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.thruster_mixer import validate_mixer
from mhseals_testing import health
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.ui.dashboard import VIEWS, render_dashboard

KEYS = 'Tab views  P procedures\nE export report  Q quit'
ADVISORY_PERIOD = 1.0


@dataclass(frozen=True)
class SessionPaths:
    bag: Path
    report: Path
    logs: Path

    @classmethod
    def from_output(cls, output):
        output = Path(output)
        return cls(output, output.with_suffix('.report.json'), output.parent / 'logs' / output.name)


class App:
    """Dashboard loop. ROS, processes, keys and the live view are injected for tests."""

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 open_keys=KeyReader, live=Live, clock=time.monotonic):
        self.args = args
        self.profile = profile
        self.monitor = monitor
        self.ros = ros
        self.processes = processes
        self.paths = paths
        self.console = console or Console()
        self.open_keys = open_keys
        self.live = live
        self.clock = clock
        self.view = 'overview'
        self.notice = 'Passive monitor'
        self.last_advisory = float('-inf')
        self.armed = False
        self.pwm_readback = ['not sampled'] * 4

    def snapshot(self):
        return self.monitor.snapshot(self.profile.required_tf)

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        return reasons

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
        blockers = self.blockers()
        notice = self.notice if not blockers else f'{self.notice} | motion blocked: {blockers[0]}'
        return dict(
            view=self.view, armed=self.armed, notice=notice, keys=KEYS,
            profile=self.profile.name, units=self.profile.command_units,
            outputs=self.profile.outputs, specs=self.profile.sensors,
            host=socket.gethostname(), domain=os.environ.get('ROS_DOMAIN_ID', '0'),
            bag='RECORDING' if self.processes.running('bag') else 'OFF',
            process_summary=self.processes.summary(), bench_override=self.args.bench_override,
            channel_map=config['channel_map'], matrix=validate_mixer(config['thruster_matrix']),
            chips=config['pwm_chips'] if self.profile.outputs == 'odroid' else None,
            channels=config['pwm_channels'],
            pwm_readback=self.pwm_readback)

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

    def handle(self, key):
        """Apply one key; returns False to quit."""
        if key in ('\t', 'right', 'left'):
            step = -1 if key == 'left' else 1
            self.view = VIEWS[(VIEWS.index(self.view) + step) % len(VIEWS)]
        elif key == 'e':
            self.export()
        elif key == 'p':
            self.notice = 'No procedures are installed yet'
        elif key in ('q', 'escape'):
            return False
        return True

    def run(self):
        with self.open_keys() as keys, self.live(self.render(), console=self.console,
                                                 screen=True, refresh_per_second=8) as view:
            while self.handle(keys.read(timeout=0.1)):
                view.update(self.render())


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    parser.add_argument('--profile', default='boat',
                        help=f'plant profile name ({", ".join(available_profiles())}) or YAML path')
    parser.add_argument('--velocity-control', action='store_true',
                        help='command m/s and rad/s through calibrated_velocity_controller')
    parser.add_argument('--start-stack', action='store_true',
                        help="launch the profile's measurement stack; default attaches to running topics")
    parser.add_argument('--fcu-url', default='', help='MAVROS FCU URL for --start-stack')
    parser.add_argument('--start-sensors', action='store_true',
                        help='with --start-stack, also start local sensor drivers')
    parser.add_argument('--bench-override', action='store_true',
                        help='allow motion with no sensors or TF (bench test, scoring disabled)')
    parser.add_argument('--stale-after', type=float, default=2.0)
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
    if not 0 < args.stale_after <= 60:
        parser.error('--stale-after must be within (0, 60] seconds')
    args.output = args.output or Path('bags') / (
        f"{profile.name.replace('+', '_')}_{datetime.now():%Y%m%d_%H%M%S}")
    return args, profile


def bringup_command(profile, args):
    arguments = dict(profile.bringup)
    if args.fcu_url:
        arguments['fcu_url'] = args.fcu_url
    if args.start_sensors:
        arguments['start_sensors'] = 'true'
    return ['ros2', 'launch', 'mhseals_testing', 'measurement.launch.py',
            *(f'{key}:={value}' for key, value in arguments.items())]


def main(argv=None):
    args, profile = parse_args(argv)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.node import RosIO
    from mhseals_testing.ros.processes import ProcessManager, bag_command, unique_path

    paths = SessionPaths.from_output(unique_path(args.output))
    console = Console()
    rclpy.init()
    monitor = BoatMonitor(args.stale_after, history_sensors=tuple(
        sensor.name for sensor in profile.sensors if sensor.kind == 'odometry'))
    ros = RosIO(profile, monitor, args.stale_after)
    processes = ProcessManager(paths.logs)
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
        app = App(args, profile, monitor, ros, processes, paths, console)
        ros.emit('session_start', profile=profile.name, bag=str(paths.bag))
        app.run()
    except KeyboardInterrupt:
        console.print('\n[yellow]Interrupted; stopping at neutral.[/]')
    finally:
        if app is not None:
            ros.emit('session_stop')
            app.export()  # before the bag stops, so the report names a complete bag
        processes.stop_all()
        ros.close()
        rclpy.try_shutdown()
    console.print(f'Session bag: {paths.bag}\nReport: {paths.report}')


if __name__ == '__main__':
    main()
