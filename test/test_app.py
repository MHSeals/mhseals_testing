import io
import json
from types import SimpleNamespace

from rich.console import Console

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.ui.app import App, SessionPaths, bringup_command, parse_args


class Processes:
    def __init__(self, running=()):
        self.names = set(running)

    def running(self, name):
        return name in self.names

    def exited(self):
        return ['measurement']

    def summary(self):
        return 'bag: RUNNING'


class Keys:
    def __init__(self, script):
        self.script = list(script)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def read(self, timeout):
        return self.script.pop(0) if self.script else 'q'


class Live:
    frames = []

    def __init__(self, renderable, **kwargs):
        self.frames.append(renderable)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def update(self, renderable):
        self.frames.append(renderable)


def app(tmp_path, argv=(), keys=(), running=('bag',)):
    args, profile = parse_args(['--output', str(tmp_path / 'session'), *argv])
    console = Console(file=io.StringIO(), width=120, height=40, color_system=None)
    ros = SimpleNamespace(fault='', now=lambda: 0.0)
    return App(args, profile, BoatMonitor(clock=lambda: 0), ros, Processes(running),
               SessionPaths.from_output(args.output), console,
               open_keys=lambda: Keys(keys), live=Live, clock=lambda: 0)


def test_session_paths_sit_next_to_the_bag(tmp_path):
    paths = SessionPaths.from_output(tmp_path / 'bags' / 's1')
    assert paths.report == tmp_path / 'bags' / 's1.report.json'
    assert paths.logs == tmp_path / 'bags' / 'logs' / 's1'


def test_parse_args_profile_and_bringup(tmp_path):
    args, profile = parse_args(['--profile', 'crane_full_stack', '--start-stack', '--start-sensors'])
    assert profile.use_sim_time and args.output.name.startswith('crane_full_stack_')
    command = bringup_command(profile, args)
    assert 'fcu:=false' in command and 'start_sensors:=true' in command
    _, boat = parse_args(['--velocity-control'])
    assert boat.command_units == 'velocity'


def test_keys_cycle_views_export_and_quit(tmp_path):
    test = app(tmp_path, keys=['\t', '\t', 'left', 'e'])
    test.run()
    assert test.view == 'sensors'
    report = json.loads((tmp_path / 'session.report.json').read_text())
    assert report['profile']['name'] == 'boat' and report['bag'].endswith('session')


def test_advisories_flag_missing_bag_and_exited_children(tmp_path):
    test = app(tmp_path, running=())
    test.render()
    advisories = test.snapshot()['advisories']
    assert advisories['recording']['level'] == 'FAIL'
    assert advisories['processes']['detail'] == 'exited: measurement'
    assert advisories['PWM readback']['level'] == 'N/A'  # no pins on this computer


def test_blockers_include_ros_fault_and_bench_override_skips_sensors(tmp_path):
    test = app(tmp_path)
    test.ros.fault = 'executor died'
    assert test.blockers()[0] == 'ROS fault: executor died'
    assert any(reason.startswith('odometry') for reason in test.blockers())
    bench = app(tmp_path, ['--bench-override'])
    assert bench.blockers() == []
    bench.console.print(bench.render())
    assert 'BENCH OVERRIDE' in bench.console.file.getvalue()


def test_sim_profile_without_clock_explains_stale_stamps(tmp_path):
    test = app(tmp_path, ['--profile', 'crane_direct'])
    test.render()
    assert test.snapshot()['advisories']['clock']['level'] == 'FAIL'
