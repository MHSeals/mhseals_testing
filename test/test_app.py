import io
import json

import pytest
import yaml
from rich.console import Console

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui.app import App, SessionPaths, bringup_command, parse_args


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Boat:
    """A plant that answers commands with odometry, and keeps every sensor and TF edge live."""

    def __init__(self, monitor, profile, gain=1.2, sensors=True):
        self.monitor, self.profile, self.gain, self.sensors = monitor, profile, gain, sensors
        self.sent = []
        self.feed((0.0, 0.0, 0.0))

    def feed(self, command):
        if not self.sensors:
            return
        for sensor in self.profile.sensors:
            if sensor.required and sensor.name != 'odometry':
                self.monitor.observe(sensor.name, stamp_age=0.0)
        vx, vy, wz = (self.gain * value for value in command)
        self.monitor.observe('odometry', dict(vx=vx, vy=vy, wz=wz, x=0.0, y=0.0, frame='odom',
                                              child='base_link'), stamp_age=0.0)
        for parent, child in self.profile.required_tf:
            self.monitor.transform(parent, child, authority=parent)

    # The boat answers as time passes (Keys.read), never at the instant of a command.
    def publish(self, command):
        self.sent.append(command)

    def neutral(self):
        self.sent.append('neutral')

    def conflicts(self):
        return []


class Arming:
    def __init__(self):
        self.armed, self.calls = False, []

    def arm(self):
        self.armed = True
        self.calls.append('arm')

    def disarm(self):
        self.armed = False
        self.calls.append('disarm')

    def fault(self):
        return ''


class Keys:
    """Advances the shared clock by each read's timeout; keeps the boat's sensors fresh."""

    def __init__(self, clock, boat, script=()):
        self.clock, self.boat, self.script = clock, boat, list(script)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def read(self, timeout):
        self.clock.now += timeout
        last = self.boat.sent[-1] if self.boat.sent else 'neutral'
        self.boat.feed((0.0, 0.0, 0.0) if last == 'neutral' else last)
        return self.script.pop(0) if self.script else None


class Live:
    def __init__(self, renderable, **kwargs):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def update(self, renderable):
        pass


class Operator(Console):
    def __init__(self, answers=()):
        super().__init__(file=io.StringIO(), width=120, height=40, color_system=None)
        self.answers = list(answers)

    def input(self, prompt=''):
        return self.answers.pop(0) if self.answers else ''


class Ros:
    fault = ''

    def __init__(self):
        self.events = []

    def now(self):
        return 100.0

    def emit(self, phase, **details):
        self.events.append(phase)


class Processes:
    def running(self, name):
        return True

    def exited(self):
        return []

    def summary(self):
        return 'bag: RUNNING'


def make_app(tmp_path, argv=(), answers=(), keys=(), sensors=True):
    args, profile = parse_args(['--output', str(tmp_path / 'session'), '--baseline', '1',
                                '--command-duration', '1', '--settle', '1', *argv])
    clock = Clock()
    monitor = BoatMonitor(clock=clock)
    boat = Boat(monitor, profile, sensors=sensors)
    app = App(args, profile, monitor, Ros(), Processes(), SessionPaths.from_output(args.output),
              Operator(answers), Arming(), boat, open_keys=lambda: Keys(clock, boat, keys),
              live=Live, clock=clock)
    return app, boat


def procedure(key):
    return next(p for p in PROCEDURES if p.key == key)


def test_session_paths_sit_next_to_the_bag(tmp_path):
    paths = SessionPaths.from_output(tmp_path / 'bags' / 's1')
    assert paths.report == tmp_path / 'bags' / 's1.report.json'
    assert paths.logs == tmp_path / 'bags' / 'logs' / 's1'
    assert paths.hardware == tmp_path / 'bags' / 's1.hardware.yaml'
    assert paths.nav2 == tmp_path / 'bags' / 's1.nav2.yaml'


def test_parse_args_profile_bringup_and_limits(tmp_path):
    args, profile = parse_args(['--profile', 'crane_full_stack', '--start-stack', '--start-sensors'])
    assert profile.use_sim_time and args.output.name.startswith('crane_full_stack_')
    command = bringup_command(profile, args)
    assert 'fcu:=false' in command and 'start_sensors:=true' in command
    assert not any(part.startswith('fcu_url') for part in command)  # simulation: no FCU
    boat_args, boat = parse_args(['--start-stack'])
    assert any(part.startswith('fcu_url:=serial://') for part in bringup_command(boat, boat_args))
    assert parse_args(['--velocity-control'])[1].command_units == 'velocity'
    with pytest.raises(SystemExit):
        parse_args(['--repeats', '9'])


def test_dashboard_keys_views_amplitude_export_and_quit(tmp_path):
    app, _ = make_app(tmp_path, keys=['\t', '+', 'e', 'q'])
    app.run()
    assert app.view == 'sensors' and app.settings.amplitude == 0.3
    assert json.loads((tmp_path / 'session.report.json').read_text())['profile']['name'] == 'boat'


def test_arming_needs_the_typed_word(tmp_path):
    app, _ = make_app(tmp_path, answers=['yes', 'ARM'])
    app.act('arm')
    assert not app.armed and app.notice == 'Arming cancelled'
    app.act('arm')
    assert app.armed and 'armed' in app.ros.events
    app.act('arm')
    assert not app.armed and app.arming.calls == ['arm', 'disarm']


def test_axis_trial_scores_both_directions_and_ends_neutral(tmp_path):
    app, boat = make_app(tmp_path, answers=['ARM'])
    app.act('arm')
    app.run_procedure(procedure('1'))
    results = app.snapshot()['results']
    assert [(r['procedure'], r['direction'], r['verdict']) for r in results] == [
        ('surge', 1, 'SIGN MATCH'), ('surge', -1, 'SIGN MATCH')]
    assert results[0]['response'] == pytest.approx(1.2 * 0.25, rel=0.05)
    assert boat.sent[-1] == 'neutral' and app.armed
    assert app.ros.events.count('axis_trial_start') == 2
    report = json.loads((tmp_path / 'session.report.json').read_text())
    assert len(report['results']) == 2


def test_space_aborts_motion_and_disarms(tmp_path):
    app, boat = make_app(tmp_path, answers=['ARM'], keys=[None] * 15 + ['space'])
    app.act('arm')
    app.watch = iter(['1', 'quit']).__next__  # choose the surge trial, then quit
    app.run()
    assert not app.armed and boat.sent[-1] == 'neutral'
    assert 'cancelled' in app.notice


def test_lost_sensors_mid_trial_stop_the_boat(tmp_path):
    app, boat = make_app(tmp_path, answers=['ARM'])
    app.act('arm')
    boat.sensors = False  # every sensor goes stale two seconds later
    app.watch = iter(['1', 'quit']).__next__
    app.run()
    assert not app.armed and 'interlock' in app.notice
    assert boat.sent[-1] == 'neutral'


def test_procedures_explain_why_they_cannot_run(tmp_path):
    app, boat = make_app(tmp_path)
    app.run_procedure(procedure('1'))
    assert 'arm first' in app.notice and boat.sent == []
    velocity_app, _ = make_app(tmp_path, ['--bench-override'], answers=['ARM'])
    velocity_app.act('arm')
    velocity_app.run_procedure(procedure('1'))
    assert 'bench override' in velocity_app.notice


def test_controller_steps_need_velocity_control_and_report_metrics(tmp_path):
    app, _ = make_app(tmp_path, ['--velocity-control', '--amplitude', '0.2'], answers=['ARM'])
    app.act('arm')
    app.run_procedure(procedure('v'))
    results = app.snapshot()['results']
    assert len(results) == 6 and all(r['procedure'] == 'controller' for r in results)
    # The fake boat answers instantly with gain 1.2, so it overshoots by 20% and never settles.
    assert results[0]['overshoot'] == pytest.approx(0.2, abs=0.01)
    assert results[0]['steady_state_error'] == pytest.approx(-0.04, abs=0.005)


def test_characterization_fits_and_saves_only_on_request(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text('frequency: 50.0\n')
    app, _ = make_app(tmp_path, ['--config', str(config), '--amplitude', '0.4',
                                 '--command-duration', '3'],
                      answers=['ARM', ''])
    app.act('arm')
    nav2 = tmp_path / 'nav2_params.yaml'
    nav2.write_text(yaml.safe_dump({
        'controller_server': {'ros__parameters': {'FollowPath': {}}},
        'velocity_smoother': {'ros__parameters': {}}}))

    def context(original):
        ctx = _choose(original(), 'save')
        ctx.services['nav2_params'] = nav2
        return ctx

    app.procedure_context = (lambda original: lambda: context(original))(app.procedure_context)
    app.run_procedure(procedure('k'))
    fitted = app.snapshot()['results'][0]['characterization']
    assert fitted['velocity_gain']['surge']['surge'] == pytest.approx(1.2, rel=0.02)
    assert 'characterization' in config.read_text()
    smoother = yaml.safe_load(app.paths.nav2.read_text())['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == pytest.approx(1.2 * 0.4, rel=0.02)


def _choose(context, answer):
    context.choose = lambda title, options, shortcuts=None: answer
    return context


def test_thruster_mapping_disarms_and_runs_the_hardware_tool(tmp_path):
    calls = []
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'], answers=['ARM'])
    app.foreground = lambda argv: calls.append(argv) or 0
    app.act('arm')
    app.procedure_context = (lambda original: lambda: _choose(original(), 'pulse each output'))(
        app.procedure_context)
    app.run_procedure(procedure('i'))
    assert not app.armed
    assert calls[0][:4] == ['ros2', 'run', 'mhseals_hardware', 'thruster_identify']
    assert '--output-backend' in calls[0] and 'topic' in calls[0] and '--manual' not in calls[0]
