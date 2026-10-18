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
    """Advances the shared clock by each read's timeout; keeps the boat's sensors fresh.

    ``script`` feeds the page and running motion; ``answers`` feed popups only, so
    a runner can't swallow the reply meant for the next prompt. A popup waiting
    with no answer left is a test bug, so it fails fast.
    """

    def __init__(self, clock, boat, script=()):
        self.clock, self.boat, self.script = clock, boat, list(script)
        self.answers = []
        self.app = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def read(self, timeout):
        self.clock.now += timeout
        last = self.boat.sent[-1] if self.boat.sent else 'neutral'
        self.boat.feed((0.0, 0.0, 0.0) if last == 'neutral' else last)
        if self.app is not None and self.app.modal is not None:
            if not self.answers:
                raise AssertionError(f"popup {self.app.modal['title']!r} waiting for a key")
            return self.answers.pop(0)
        return self.script.pop(0) if self.script else None


class Live:
    def __init__(self, renderable, **kwargs):
        self.frames = 0

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def update(self, renderable, refresh=False):
        self.frames += 1

    def stop(self):
        pass

    def start(self, refresh=False):
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

    def stop(self, name, timeout=10):
        self.stopped = getattr(self, 'stopped', []) + [name]

    def running(self, name):
        return True

    def exited(self):
        return []

    def summary(self):
        return 'bag: RUNNING'


def make_app(tmp_path, argv=(), keys=(), sensors=True):
    config = tmp_path / 'hardware.yaml'
    if not config.exists():
        config.write_text('frequency: 50.0\n')
    args, profile = parse_args(['--output', str(tmp_path / 'session'), '--baseline', '1',
                                '--command-duration', '1', '--settle', '1',
                                '--config', str(config), *argv])
    clock = Clock()
    monitor = BoatMonitor(clock=clock)
    boat = Boat(monitor, profile, sensors=sensors)
    reader = Keys(clock, boat, keys)
    pulses = []
    app = App(args, profile, monitor, Ros(), Processes(), SessionPaths.from_output(args.output),
              Operator(), Arming(), boat, open_keys=lambda: reader, live=Live, clock=clock,
              pulses=pulses.append)
    reader.app, app.reader, app.pulse_log = app, reader, pulses
    return app, boat


def press(app, *keys):
    """Queue replies for the next popups."""
    app.reader.answers.extend(keys)


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


def arm(app):
    """Arm through the popup: typed ARM for real pins, the Arm button otherwise."""
    if app.profile.outputs == 'odroid':
        press(app, 'a', 'r', 'm', 'enter')
    else:
        press(app, 'y')
    app.dispatch('a')
    assert app.armed


def test_focus_moves_and_keys_act_on_the_focused_panel(tmp_path):
    app, _ = make_app(tmp_path, keys=['\t', '\t', 'backtab', 'j', '+', 'x', '?', 'q'])
    press(app, 'z', 'y')  # close the help popup; confirm quitting
    app.console.size = (120, 40)
    app.run()
    assert app.focus == 'motion' and app.settings.amplitude == 0.3
    assert json.loads((tmp_path / 'session.report.json').read_text())['profile']['name'] == 'boat'
    assert app.quit and app.live is None


def test_status_cursor_walks_sensors_frames_and_system(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        rows = app.rows('status')
        for _ in range(rows + 3):
            app.dispatch('down')
        assert app.cursors['status'] == rows - 1
        press(app, 'enter')
        app.dispatch('enter')  # details popup, closed by any key


def test_arming_needs_the_typed_word_on_real_outputs(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        press(app, 'n', 'o', 'enter')
        app.dispatch('a')
        assert not app.armed and app.notice == 'Arming cancelled'
        press(app, 'a', 'r', 'm', 'backspace', 'm', 'enter')
        app.dispatch('a')
        assert app.armed and 'armed' in app.ros.events
        press(app, 'enter')  # the arm panel offers Disarm first
        app.dispatch('a')
        assert not app.armed and app.arming.calls == ['arm', 'disarm']


def test_simulator_arms_with_a_button_and_escape_cancels(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    with app.attached():
        press(app, 'escape')
        app.dispatch('a')
        assert not app.armed
        press(app, 'down', 'enter')
        app.dispatch('a')
        assert app.armed


def test_axis_trial_scores_both_directions_and_ends_neutral(tmp_path):
    app, boat = make_app(tmp_path)
    with app.attached():
        arm(app)
        press(app, 'enter', 'enter')  # one prompt per direction
        app.run_procedure(procedure('1'))
    results = app.snapshot()['results']
    assert [(r['procedure'], r['direction'], r['verdict']) for r in results] == [
        ('surge', 1, 'SIGN MATCH'), ('surge', -1, 'SIGN MATCH')]
    assert results[0]['response'] == pytest.approx(1.2 * 0.25, rel=0.05)
    assert boat.sent[-1] == 'neutral' and app.armed
    assert app.focus == 'activity' and app.ros.events.count('axis_trial_start') == 2


def test_escape_at_a_prompt_cancels_and_disarms(tmp_path):
    app, boat = make_app(tmp_path)
    with app.attached():
        arm(app)
        press(app, 'escape')
        app.run_procedure(procedure('1'))
    assert not app.armed and 'cancelled' in app.notice and app.running is None


def test_space_aborts_motion_and_disarms(tmp_path):
    app, boat = make_app(tmp_path, keys=['1'] + [None] * 15 + ['space', 'q'])
    press(app, 'enter', 'y')
    app.focus = 'procedures'
    app.arming.armed = True
    app.run()
    assert not app.armed and boat.sent[-1] == 'neutral'
    assert any('cancelled' in message for _, message in app.monitor.events)


def test_lost_sensors_mid_trial_stop_the_boat(tmp_path):
    app, boat = make_app(tmp_path, keys=['1'])
    press(app, 'enter', 'y')
    app.focus = 'procedures'
    app.arming.armed = True
    boat.sensors = False  # every sensor goes stale two seconds later
    app.reader.script += [None] * 60 + ['q']  # quit only after the interlock tripped
    app.run()
    assert not app.armed and boat.sent[-1] == 'neutral'
    assert any('interlock' in message for _, message in app.monitor.events)


def test_procedures_that_need_arming_offer_the_arm_panel(tmp_path):
    app, boat = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    app.open_events = lambda: Events(app.clock, ['press:x'])
    with app.attached():
        press(app, 'n')                      # decline: nothing runs
        app.run_procedure(procedure('m'))
        assert not app.armed and 'arm first' in app.notice and boat.sent == []
        press(app, 'y')                      # arm: manual control starts
        app.run_procedure(procedure('m'))
    assert app.armed and app.notice == 'Manual control ended'


def test_procedures_explain_why_they_cannot_run(tmp_path):
    app, boat = make_app(tmp_path)
    with app.attached():
        press(app, 'escape')
        app.run_procedure(procedure('1'))
    assert 'arm first' in app.notice and boat.sent == []
    bench, _ = make_app(tmp_path, ['--bench-override'])
    with bench.attached():
        arm(bench)
        bench.run_procedure(procedure('1'))
    assert 'bench override' in bench.notice


def _choose(context, answer):
    context.choose = lambda title, options, shortcuts=None, **kwargs: answer
    context.prompt = lambda message, title=None: ''
    return context


def scripted(app, answer):
    original = app.procedure_context
    app.procedure_context = lambda: _choose(original(), answer)
    app._scripted = True


def test_controller_steps_need_velocity_control_and_report_metrics(tmp_path):
    app, _ = make_app(tmp_path, ['--velocity-control', '--amplitude', '0.2'])
    with app.attached():
        arm(app)
        scripted(app, None)
        app.run_procedure(procedure('v'))
    results = app.snapshot()['results']
    assert len(results) == 6 and all(r['procedure'] == 'controller' for r in results)
    # The fake boat answers instantly with gain 1.2, so it overshoots by 20% and never settles.
    assert results[0]['overshoot'] == pytest.approx(0.2, abs=0.01)
    assert results[0]['steady_state_error'] == pytest.approx(-0.04, abs=0.005)


def test_characterization_fits_and_saves_only_on_request(tmp_path):
    app, _ = make_app(tmp_path, ['--amplitude', '0.4', '--command-duration', '3'])
    nav2 = tmp_path / 'nav2_params.yaml'
    nav2.write_text(yaml.safe_dump({
        'controller_server': {'ros__parameters': {'FollowPath': {}}},
        'velocity_smoother': {'ros__parameters': {}}}))
    original = app.procedure_context

    def context():
        ctx = _choose(original(), 'save')
        ctx.services['nav2_params'] = nav2
        return ctx

    with app.attached():
        arm(app)
        app.procedure_context = context
        app.dispatch('k')
    fitted = app.snapshot()['results'][0]['characterization']
    assert fitted['velocity_gain']['surge']['surge'] == pytest.approx(1.2, rel=0.02)
    assert 'characterization' in (tmp_path / 'hardware.yaml').read_text()
    smoother = yaml.safe_load(app.paths.nav2.read_text())['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == pytest.approx(.48, rel=0.02)  # measured speed at 0.4 effort


def test_thruster_panel_reverses_and_rewires_with_hot_reload(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    with app.attached():
        arm(app)
        app.focus = 'thrusters'
        app.dispatch('down')  # FR
        app.dispatch('r')
        app.dispatch('3')     # FR to output 3, RR takes FR's output 2
        assert app.draft and 'reversed' not in (tmp_path / 'hardware.yaml').read_text()
        app.dispatch('s')
    saved = yaml.safe_load((tmp_path / 'hardware.yaml').read_text())
    assert saved['reversed'] == [False, True, False, False]
    assert saved['channel_map'] == [1, 3, 2, 4]
    assert app.args.hardware_config['channel_map'] == (1, 3, 2, 4) or \
        list(app.args.hardware_config['channel_map']) == [1, 3, 2, 4]
    snapshot = yaml.safe_load(app.paths.hardware.read_text())  # what the driver reloads
    assert snapshot['channel_map'] == [1, 3, 2, 4] and snapshot['output_backend'] == 'topic'
    assert 'config_reloaded' in app.ros.events


def test_external_config_edits_reload_and_bad_ones_are_refused(tmp_path):
    app, _ = make_app(tmp_path)
    config = tmp_path / 'hardware.yaml'
    config.write_text('frequency: 50.0\nreversed: [true, false, false, false]\n')
    app.config_mtime = None
    app.watch_config()
    assert app.args.hardware_config['reversed'] == [True, False, False, False]
    config.write_text('reversed: [true]\n')
    app.config_mtime = None
    app.watch_config()
    assert 'rejected' in app.notice and app.args.hardware_config['reversed'][0] is True


def test_test_pulses_need_arming_follow_the_wiring_and_clear(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    with app.attached():
        app.focus = 'thrusters'
        app.dispatch('right')
        assert app.test_pulses is None and 'Arm first' in app.notice
        arm(app)
        app.save_hardware(channel_map=[2, 1, 3, 4])
        for _ in range(3):
            app.dispatch('right')  # FL is wired to output 2
        app.tick()
        assert app.pulse_log[-1] == [1500, 1530, 1500, 1500]
        app.dispatch('\t')         # leaving the panel ends the test pulses
        assert app.test_pulses is None and app.pulse_log[-1] == [1500] * 4


def test_thruster_set_maps_outputs_and_reversals(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    # Outputs 1-4 drive FR, FL, RL, RR; RL thrusts backwards. Each output: Enter to pulse,
    # pick the thruster by number, then polarity; finally save.
    script = []
    for pick, polarity in (('2', 'y'), ('1', 'y'), ('2', 'n'), ('1', 'y')):
        script += ['enter', pick, polarity]
    with app.attached():
        arm(app)
        press(app, *script, 's')
        app.run_procedure(procedure('i'))
    saved = yaml.safe_load((tmp_path / 'hardware.yaml').read_text())
    assert saved['channel_map'] == [2, 1, 4, 3]
    assert saved['reversed'] == [False, False, False, True]
    assert app.snapshot()['results'][-1]['verdict'] == 'SAVED'
    pulsed = [p for p in app.pulse_log if p != [1500] * 4]
    assert [p.index(max(p)) + 1 for p in pulsed[::max(1, len(pulsed) // 4)]][:1] == [1]
    assert app.pulse_log[-1] == [1500] * 4 and app.test_pulses is None


def run(app, key, answer=None):
    """Run a procedure on the attached page; popups answer ``answer`` and Enter."""
    if not getattr(app, '_scripted', False):
        scripted(app, answer)
        app._scripted = True
    with app.attached():
        app.run_procedure(procedure(key))


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


def nav_app(tmp_path, argv=(), nav=None):
    app, boat = make_app(tmp_path, ['--velocity-control', *argv])
    clock = app.clock
    app.nav2_factory = lambda: nav or FakeNav(clock)
    return app, boat


def test_nav2_bringup_checks_without_arming(tmp_path):
    app, boat = nav_app(tmp_path)
    run(app, 'n')
    checks = {r['check']: r['verdict'] for r in app.snapshot()['results']}
    assert checks == {'lifecycle': 'PASS', 'costmap': 'PASS', 'plan': 'PASS'}
    assert not app.armed and all(sent == 'neutral' for sent in boat.sent)


def test_waypoint_square_scores_the_route_and_cancels_on_exit(tmp_path):
    nav = None
    app, _ = nav_app(tmp_path, ['--nav-distance', '3'])
    nav = FakeNav(app.clock)
    app.nav2_factory = lambda: nav
    app.arming.armed = True
    run(app, 'w')
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
    app.arming.armed = True
    run(app, 's')
    [result] = app.snapshot()['results']
    assert result['drift_speed'] == pytest.approx(0.03, rel=0.05)
    assert result['verdict'] == 'HELD' and result['resends'] >= 1


def test_waypoints_need_velocity_commands(tmp_path):
    app, _ = make_app(tmp_path)
    app.arming.armed = True
    run(app, 'w')
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
    run(app, 'o')
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
    app.arming.armed = True
    scripted(app, 'nav_channel')
    run(app, 't')
    [result] = app.snapshot()['results']
    assert result['verdict'] == 'PASS' and result['points'] == 20
    assert started[0][:4] == ['ros2', 'launch', 'mhseals_nav', 'task.launch.py']
    assert 'task:=nav_channel' in started[0] and stopped == ['task_nav_channel']


@pytest.mark.parametrize('width,path,expected', [
    (120, ['ctrl-l', 'ctrl-j', 'ctrl-l', 'ctrl-k', 'ctrl-h'], 'motion'),
    (120, ['ctrl-right', 'ctrl-right', 'ctrl-down', 'ctrl-left'], 'procedures'),
    (120, ['ctrl-k', 'ctrl-h'], 'status'),               # no panel there: focus stays
    (80, ['ctrl-l', 'ctrl-j', 'ctrl-j', 'ctrl-k'], 'procedures'),
    (80, ['ctrl-l', 'ctrl-j', 'ctrl-h'], 'status'),
])
def test_ctrl_directions_focus_the_neighbouring_panel(tmp_path, width, path, expected):
    app, _ = make_app(tmp_path)
    app.console.size = (width, 40)
    for key in path:
        app.dispatch(key)
    assert app.focus == expected


def test_leaving_thrusters_by_direction_ends_test_pulses(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    app.console.size = (120, 40)
    with app.attached():
        arm(app)
        app.focus = 'thrusters'
        app.dispatch('right')
        assert app.test_pulses is not None
        app.dispatch('ctrl-h')
    assert app.focus == 'motion' and app.test_pulses is None


def test_grab_moves_a_thruster_and_undo_restores(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    app.focus = 'thrusters'
    app.dispatch('r')        # FL reversed
    app.dispatch('enter')    # pick FL up
    app.dispatch('down')
    app.dispatch('down')     # carried to RR's row: FL<->FR, then FR<->RR swaps
    app.dispatch('enter')
    assert app.draft == dict(channel_map=[2, 3, 1, 4], reversed=[False, False, True, False])
    assert app.cursors['thrusters'] == 2 and app.grabbed is None
    app.dispatch('u')
    assert app.draft is None and app.mapping() == ([1, 2, 3, 4], [False] * 4)


def test_escape_never_quits_and_quit_asks_first(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        app.dispatch('escape')
        assert not app.quit
        press(app, 'escape')
        app.dispatch('q')
        assert not app.quit
        press(app, 'enter')
        app.dispatch('q')
    assert app.quit


def test_quitting_with_an_unsaved_mapping_can_save_it(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'], keys=['r', 'q'])
    app.focus = 'thrusters'
    press(app, 'enter')  # Save mapping and quit
    app.run()
    assert yaml.safe_load((tmp_path / 'hardware.yaml').read_text())['reversed'][0] is True


def test_close_runs_every_step_once_and_summarizes(tmp_path):
    app, boat = make_app(tmp_path)
    app.arming.armed = True
    with app.attached():
        app.close()
        app.close()
    assert not app.armed and boat.sent[-1] == 'neutral'
    assert app.processes.stopped == ['bag'] and app.ros.events.count('session_stop') == 1
    assert (tmp_path / 'session.report.json').exists()
    assert dict(app.summary())['Results'] == '0/0 good'


def test_arm_panel_lists_interlocks_before_arming(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'], sensors=False)
    rows = dict(app.arm_rows())
    assert rows['Outputs'].startswith('simulator') and rows['Driver'] == 'starts on arm'
    assert rows['Interlocks'].startswith('×')


class Events:
    """A key reader that reports presses and releases, as Kitty terminals do."""

    def __init__(self, clock, keys):
        self.clock, self.keys, self.entered = clock, list(keys), 0

    def __enter__(self):
        self.entered += 1
        return self

    def __exit__(self, *exc):
        return False

    def read(self, timeout=None):
        self.clock.now += 0.05  # each event arrives a frame later, like a held key
        return self.keys.pop(0) if self.keys else 'x'


def test_manual_control_drives_held_keys_on_the_page_and_ends_neutral(tmp_path):
    app, boat = make_app(tmp_path)
    app.console.size = (120, 40)
    events = Events(app.clock, ['press:w', None, 'repeat:w', 'release:w', None,
                                'press:a', None, 'release:a', 'press:x'])
    app.open_events = lambda: events
    shown = []
    original = app.manual_show
    app.manual_show = lambda *a, **k: (shown.append((app.manual, a[0])), original(*a, **k))
    with app.attached():
        arm(app)
        app.dispatch('m')
    sent = [command for command in boat.sent if command != 'neutral']
    assert any(command[0] > 0 for command in sent)          # W drove forward
    assert any(command[1] > 0 for command in sent)          # A drove to port
    assert boat.sent[-1] == 'neutral' and not app.manual and app.armed
    assert events.entered == 1 and app.live is None
    assert app.focus == 'motion' and app.notice == 'Manual control ended'


def test_manual_control_stops_on_an_interlock(tmp_path):
    app, boat = make_app(tmp_path)
    app.open_events = lambda: Events(app.clock, ['press:w'] + ['repeat:w'] * 80)
    with app.attached():
        arm(app)
        boat.sensors = False
        try:
            app.dispatch('m')
        except Exception as error:  # run() turns this into a stop
            app.stop(str(error))
    assert not app.armed and boat.sent[-1] == 'neutral'


def test_ctrl_c_closes_gracefully_like_quit(tmp_path):
    app, boat = make_app(tmp_path)
    app.arming.armed = True

    class Interrupting(Keys):
        def read(self, timeout):
            raise KeyboardInterrupt

    app.open_keys = lambda: Interrupting(app.clock, boat)
    app.run()
    assert app.quit and app.closed and not app.armed and boat.sent[-1] == 'neutral'
    assert app.ros.events.count('session_stop') == 1
    assert (tmp_path / 'session.report.json').exists()


def test_children_die_with_the_suite(tmp_path):
    import os, signal, subprocess, sys, time
    # A middle process starts a child through ProcessManager, then is killed hard.
    script = (f"import sys, time; sys.path[:0] = {sys.path!r}\n"
              "from mhseals_testing.ros.processes import ProcessManager\n"
              f"p = ProcessManager({str(tmp_path)!r})\n"
              "c = p.start('sleeper', [sys.executable, '-c', 'import time; time.sleep(60)'])\n"
              "print(c.pid, flush=True); time.sleep(60)\n")
    parent = subprocess.Popen([sys.executable, '-c', script], stdout=subprocess.PIPE, text=True)
    child = int(parent.stdout.readline())
    parent.send_signal(signal.SIGKILL)
    parent.wait()
    for _ in range(50):
        try:
            os.kill(child, 0)
            # A container PID 1 may leave a dead orphan unreaped. It cannot drive outputs.
            from pathlib import Path
            if Path(f'/proc/{child}/stat').read_text().split(') ', 1)[1].startswith('Z'):
                break
        except (ProcessLookupError, FileNotFoundError):
            break
        time.sleep(0.05)
    else:
        os.kill(child, signal.SIGKILL)
        raise AssertionError('child outlived the suite')


@pytest.mark.parametrize('focus', ['status', 'motion', 'procedures', 'activity', 'thrusters'])
def test_characterize_shortcut_reaches_procedure_from_every_panel(tmp_path, focus):
    app, _ = make_app(tmp_path)
    app.focus = focus
    called = []
    app.run_procedure = lambda p: called.append(p.key)
    app.dispatch('k')
    assert called == ['k']


def test_panel_edit_keys_take_precedence_over_procedure_shortcuts(tmp_path):
    app, _ = make_app(tmp_path)
    app.focus = 'thrusters'
    called = []
    app.run_procedure = lambda p: called.append(p.key)
    app.dispatch('3')
    app.dispatch('s')
    assert called == []
    assert app.args.hardware_config['channel_map'][0] == 3
    app.focus = 'activity'
    app.dispatch('v')
    assert app.activity == 'log' and called == []
    app.dispatch('n')
    assert called == ['n']


@pytest.mark.parametrize('letter,expected', [
    ('w', (.35, 0, 0)), ('s', (-.35, 0, 0)),
    ('a', (0, .35, 0)), ('d', (0, -.35, 0)),
])
def test_manual_wasd_reserved_after_yaw_and_speed_change(tmp_path, letter, expected):
    app, boat = make_app(tmp_path)
    events = Events(app.clock, ['press:left', None, '+', None, '+', None,
                                letter, None, 'repeat:left', None, 'release:left',
                                None, 'x'])
    app.open_events = lambda: events
    with app.attached():
        arm(app)
        # Manual A/S must never reach Arm, station keeping or Save shortcuts.
        app.arm_panel = lambda: pytest.fail('manual key opened Arm')
        app.on_thrusters = lambda key: pytest.fail('manual key edited wiring')
        app.focus = 'thrusters'
        app.dispatch('m')
    sent = [command for command in boat.sent if command != 'neutral']
    assert any(command[2] > 0 for command in sent)
    assert sent[-1] == pytest.approx(expected)
    assert boat.sent[-1] == 'neutral' and app.armed and not app.manual
    assert not app.snapshot()['results']  # S did not run station keeping.
