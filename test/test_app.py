import io
import math
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
    def start(self, name, argv):
        pass

    def stop(self, name):
        pass

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


class Grid:
    def __init__(self, data):
        self.data = data


class NavGoal:
    def __init__(self, clock, finish_at):
        self.clock, self.finish_at, self.cancelled = clock, finish_at, False

    @property
    def done(self):
        return self.clock.now >= self.finish_at

    @property
    def status(self):
        return 'SUCCEEDED' if self.done else 'RUNNING'

    def cancel(self):
        self.cancelled = True


class FakeNav:
    """Nav2 whose boat follows the route at 1 m/s once a goal is sent."""
    map_frame, base_frame = 'map', 'base_link'

    def __init__(self, clock, drift=0.0):
        self.clock, self.drift = clock, drift
        self.costmap = Grid([0, 0, 100, 99, 50])
        self.goals, self.route, self.sent_at = [], None, None

    def lifecycle(self):
        return {'controller_server': 'active', 'bt_navigator': 'active'}

    def plan(self, goal):
        return [(0, 0), (2.5, 0), goal[:2]]

    def pose(self):
        if self.route is None:
            return (self.drift * self.clock.now, 0.0, 0.0)
        travelled = self.clock.now - self.sent_at
        previous = (0.0, 0.0)
        for corner in self.route:
            length = math.dist(previous, corner[:2])
            if travelled <= length:
                f = travelled / length if length else 1
                return (previous[0] + f * (corner[0] - previous[0]),
                        previous[1] + f * (corner[1] - previous[1]), 0.0)
            travelled -= length
            previous = corner[:2]
        return (*previous, 0.0)

    def navigate(self, goals):
        if len(goals) > 1:
            self.route, self.sent_at = goals, self.clock.now
            length = sum(math.dist(a[:2], b[:2]) for a, b in zip([(0, 0)] + goals, goals))
            goal = NavGoal(self.clock, self.clock.now + length)
        else:
            goal = NavGoal(self.clock, self.clock.now + 1.0)
        self.goals.append(goal)
        return goal


def nav_app(tmp_path, argv=(), nav=None, answers=('ARM',)):
    app, boat = make_app(tmp_path, ['--velocity-control', *argv], answers=answers)
    clock = app.clock
    app.nav2_factory = lambda: nav or FakeNav(clock)
    return app, boat


def test_nav2_bringup_checks_without_arming(tmp_path):
    app, boat = nav_app(tmp_path)
    app.run_procedure(procedure('n'))
    checks = {r['check']: r['verdict'] for r in app.snapshot()['results']}
    assert checks == {'lifecycle': 'PASS', 'costmap': 'PASS', 'plan': 'PASS'}
    assert not app.armed and all(sent == 'neutral' for sent in boat.sent)


def test_waypoint_square_scores_the_route_and_cancels_on_exit(tmp_path):
    nav = None
    app, _ = nav_app(tmp_path, ['--nav-distance', '3'])
    nav = FakeNav(app.clock)
    app.nav2_factory = lambda: nav
    app.act('arm')
    app.run_procedure(procedure('w'))
    [result] = app.snapshot()['results']
    assert result['verdict'] == 'REACHED'
    assert result['cross_track_max'] == pytest.approx(0, abs=1e-6)
    assert result['route_length'] == pytest.approx(12)
    assert result['final_error'] == pytest.approx(0, abs=0.15)
    assert nav.goals[0].cancelled


def test_station_keeping_resends_goals_and_reports_drift(tmp_path):
    app, _ = nav_app(tmp_path, ['--station-hold', '20', '--station-tolerance', '1'])
    nav = FakeNav(app.clock, drift=0.03)
    app.nav2_factory = lambda: nav
    app.act('arm')
    app.run_procedure(procedure('s'))
    [result] = app.snapshot()['results']
    assert result['drift_speed'] == pytest.approx(0.03, rel=0.05)
    assert result['verdict'] == 'HELD' and result['resends'] >= 1


def test_waypoints_need_velocity_commands(tmp_path):
    app, _ = make_app(tmp_path, answers=['ARM'])
    app.act('arm')
    app.run_procedure(procedure('w'))
    assert 'velocity' in app.notice


def test_perception_rates_and_costmap(tmp_path):
    app, _ = nav_app(tmp_path, ['--observe', '2'])
    clock = app.clock
    original = app.open_keys

    def keys():
        reader = original()
        read = reader.read

        def read_and_publish(timeout):
            key = read(timeout)
            app.monitor.observe('LiDAR', dict(summary='9000 points'), stamp_age=0.0)
            if int(clock.now * 10) % 10 == 0:  # detections at 1 Hz only, lower than LiDAR
                app.monitor.observe('detections', dict(count=2, summary='2 detections'),
                                    stamp_age=0.0)
            return key

        reader.read = read_and_publish
        return reader

    app.open_keys = keys
    app.run_procedure(procedure('o'))
    checks = {r['check']: r['verdict'] for r in app.snapshot()['results']}
    assert checks['LiDAR'] == 'PASS' and checks['camera'] == 'MISSING'
    assert checks['tracks'] == 'MISSING' and checks['costmap'] == 'PASS'


class Watcher:
    def __init__(self):
        self.events, self.status = [], {}

    def reset(self):
        self.events.clear()


def test_mission_rehearsal_runs_the_task_and_scores_it(tmp_path):
    app, _ = nav_app(tmp_path)
    watcher, started, stopped = Watcher(), [], []
    app.watcher_factory = lambda: watcher
    clock = app.clock

    class Processes(type(app.processes)):
        def start(self, name, argv):
            started.append(argv)
            watcher.status = {'tip': 'through gates'}

        def running(self, name):
            if name == 'bag':
                return True
            if clock.now > 5 and not watcher.events:  # the task finishes after 5 s
                watcher.events += [dict(event='gate_cleared', elapsed=3.0),
                                   dict(event='gate_cleared', elapsed=4.0),
                                   dict(event='task_finished', outcome='SUCCESS', elapsed=5.0)]
            return not watcher.events

        def stop(self, name):
            stopped.append(name)

    app.processes = Processes()
    app.act('arm')
    app.procedure_context = (lambda original: lambda: _choose(original(), 'gates'))(
        app.procedure_context)
    app.run_procedure(procedure('t'))
    [result] = app.snapshot()['results']
    assert result['verdict'] == 'PASS' and result['points'] == 20
    assert started[0][:4] == ['ros2', 'launch', 'mhseals_nav', 'task.launch.py']
    assert 'task:=gates' in started[0] and stopped == ['task_gates']
