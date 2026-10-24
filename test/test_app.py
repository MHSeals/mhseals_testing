import contextlib
import errno
import io
import math
import os
import json

import pytest
from rich.segment import Segments
import yaml
from rich.console import Console

from mhseals_hardware.menu import Cancelled
from mhseals_testing import events
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import theme
from mhseals_testing.harness.registry import Registry as RegistryBase
from mhseals_testing.ui.app import (
    App, Detach, SessionPaths, decoupled, parse_args, stack_overlay)


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
    def publish(self, command, manual=False):
        self.sent.append(command)

    def neutral(self):
        self.sent.append('neutral')

    def hold(self, cause='keys'):
        self.holds = getattr(self, 'holds', []) + [cause]

    def conflicts(self):
        return []

    def engage(self):
        pass

    def release(self):
        pass


class Arming:
    def __init__(self):
        self.armed, self.calls = False, []

    def arm(self):
        self.armed = True
        self.calls.append('arm')

    def begin(self):  # the driver subscribes at once in the fake
        self.arm()

    def poll(self):
        return 'armed' if self.armed else 'starting'

    def cancel(self):
        self.calls.append('cancel')

    def disarm(self):
        self.armed = False
        self.calls.append('disarm')

    def fault(self):
        return ''

    def rival(self):
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
        self.last = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def update(self, renderable, refresh=False):
        self.frames += 1
        self.last = renderable

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

    def subscribers(self, topic):
        return []

    def nodes(self):
        return []

    def emit(self, phase, **details):
        self.events.append(phase)
        self.lines = getattr(self, 'lines', []) + [(phase, details)]


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

    def records(self):
        return {'bag': {'pid': 9, 'pgid': 9}}

    def release(self, names=()):
        self.released = list(names)


def make_app(tmp_path, argv=(), keys=(), sensors=True):
    config = tmp_path / 'hardware.yaml'
    if not config.exists():
        config.write_text('frequency: 50.0\nchannel_map: [1, 2, 3, 4]\n')
    args, profile = parse_args(['--output', str(tmp_path / 'session'), '--set', 'baseline=1',
                                '--set', 'command=1', '--set', 'settle=1',
                                '--config', str(config), *argv])
    clock = Clock()
    monitor = BoatMonitor(clock=clock)
    boat = Boat(monitor, profile, sensors=sensors)
    reader = Keys(clock, boat, keys)
    pulses = []
    paths = SessionPaths.from_output(args.output)
    paths.hardware.parent.mkdir(parents=True, exist_ok=True)   # main() gets these from session.create
    app = App(args, profile, monitor, Ros(), Processes(), paths,
              Operator(), Arming(), boat, open_keys=lambda: reader, live=Live, clock=clock,
              pulses=pulses.append)
    reader.app, app.reader, app.pulse_log = app, reader, pulses
    return app, boat


def press(app, *keys):
    """Queue replies for the next popups."""
    app.reader.answers.extend(keys)


def procedure(key):
    return next(p for p in PROCEDURES if p.key == key)


def test_session_paths_all_sit_inside_the_session_directory(tmp_path):
    d = tmp_path / 'sessions' / 's1'
    paths = SessionPaths.from_output(d)
    assert paths.bag == d / 'bag' and paths.logs == d / 'logs'
    assert paths.journal == d / 'events.jsonl' and paths.runs == d / 'runs'
    assert paths.session == d / 'session.json' and paths.truth == d / 'course_truth.json'
    assert paths.hardware.parent == d / 'snapshots' and paths.report.parent == d / 'analysis'
    assert all(d in p.parents for p in (paths.report, paths.hardware, paths.nav2))


def test_parse_args_profile_components_and_limits(tmp_path):
    args, profile = parse_args(['--profile', 'crane_full_stack', '--start-stack'])
    assert profile.use_sim_time and args.output is None
    assert not profile.uses_fcu and stack_overlay(profile, args) == {}  # simulation: no FCU
    boat_args, boat = parse_args(['--start-stack'])
    assert boat.uses_fcu
    assert stack_overlay(boat, boat_args)['odometry']['fcu_url'].startswith('serial://')
    assert parse_args(['--set', 'command_mode=velocity'])[1].command_units == 'velocity'
    with pytest.raises(SystemExit):
        parse_args(['--set', 'repeats=9'])


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
    assert json.loads((tmp_path / 'session' / 'analysis' / 'report.json').read_text())['profile']['name'] == 'boat'
    assert app.quit and app.live is None


@pytest.mark.parametrize('up', ['up', 'k'])
def test_status_cursor_walks_sensors_frames_and_system(tmp_path, up):
    app, _ = make_app(tmp_path)
    with app.attached():
        rows = app.rows('status')
        for _ in range(rows + 3):
            app.dispatch('down')
        assert app.cursors['status'] == rows - 1
        app.dispatch(up)
        assert app.cursors['status'] == rows - 2
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


def test_escape_at_a_prompt_cancels_and_disarms(tmp_path):
    app, boat = make_app(tmp_path)
    with app.attached():
        arm(app)
        press(app, 'escape')
        app.run_procedure(procedure('f'))
    assert not app.armed and 'cancelled' in app.notice and app.running is None


def test_space_aborts_motion_and_disarms(tmp_path):
    app, boat = make_app(tmp_path, keys=['f'] + [None] * 15 + ['space', 'q'])
    press(app, 'enter', 'y')
    app.focus = 'procedures'
    app.arming.armed = True
    app.run()
    assert not app.armed and boat.sent[-1] == 'neutral'
    assert boat.holds == ['keys']
    assert any('cancelled' in message for _, message in app.monitor.events)


def test_space_still_disarms_when_the_hold_request_fails(tmp_path):
    app, boat = make_app(tmp_path, keys=['space', 'q'])

    def broken(cause='keys'):
        raise RuntimeError('publisher gone')

    boat.hold = broken
    app.arming.armed = True
    try:
        app.dispatch('space')
    except RuntimeError:
        pass
    assert not app.armed and boat.sent[-1] == 'neutral'


def test_lost_sensors_mid_trial_stop_the_boat(tmp_path):
    app, boat = make_app(tmp_path, keys=['f'])
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
        app.run_procedure(procedure('f'))
    assert 'arm first' in app.notice and boat.sent == []
    bench, _ = make_app(tmp_path, ['--set', 'bench_override=on'])
    with bench.attached():
        arm(bench)
        bench.run_procedure(procedure('f'))
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
    app, _ = make_app(tmp_path, ['--set', 'command_mode=velocity', '--set', 'amplitude=0.2'])
    with app.attached():
        arm(app)
        scripted(app, None)
        app.run_procedure(procedure('v'))
    results = app.snapshot()['results']
    assert len(results) == 6 and all(r['procedure'] == 'controller' for r in results)
    # The fake boat answers instantly with gain 1.2, so it overshoots by 20% and never settles.
    assert results[0]['overshoot'] == pytest.approx(0.2, abs=0.01)
    assert results[0]['steady_state_error'] == pytest.approx(-0.04, abs=0.005)


def test_characterization_fits_and_saves_only_on_request(tmp_path, nav2_user_file):
    app, _ = make_app(tmp_path, ['--set', 'amplitude=0.4', '--set', 'command=3'])
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
        app.dispatch('f')
    fitted = app.snapshot()['results'][0]['characterization']
    assert set(fitted['speed_curves']) == {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'}
    assert fitted['motion_limits']['velocity']['surge'] == pytest.approx(.48, rel=0.02)
    assert 'characterization' in (tmp_path / 'hardware.yaml').read_text()
    smoother = yaml.safe_load(app.paths.nav2.read_text())['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == pytest.approx(.48, rel=0.02)  # measured speed at 0.4 effort
    # Saving also keeps the limits in the lasting file the Navigation component launches with.
    assert yaml.safe_load(nav2_user_file.read_text()) == yaml.safe_load(app.paths.nav2.read_text())


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
    app.announce(app.watcher.reload('hardware'))
    assert app.args.hardware_config['reversed'] == [True, False, False, False]
    config.write_text('reversed: [true]\n')
    app.announce(app.watcher.reload('hardware'))
    assert 'rejected' in app.notice and app.args.hardware_config['reversed'][0] is True


@pytest.mark.parametrize('text', ['reversed: [true, false\n', 'reversed:\n\t- true\n', 'channel_map: 5\n',
                                  'manual_amplitude: "x"\n'])
def test_unparsable_or_mistyped_config_is_refused_not_raised(tmp_path, text):
    app, _ = make_app(tmp_path)
    (tmp_path / 'hardware.yaml').write_text(text)
    app.announce(app.watcher.reload('hardware'))  # a typo must not end the session
    assert 'rejected' in app.notice and app.config_note == 'invalid edit'


def test_reload_refreshes_the_arming_copy_of_the_config(tmp_path):
    app, _ = make_app(tmp_path)
    (tmp_path / 'hardware.yaml').write_text('reversed: [true, false, false, false]\n')
    app.announce(app.watcher.reload('hardware'))
    assert app.arming.hardware_config is app.args.hardware_config
    assert app.arming.hardware_config['reversed'][0] is True


def test_manual_yaw_uses_the_configured_scale(tmp_path):
    app, boat = make_app(tmp_path)
    app.console.size = (120, 40)
    app.args.hardware_config['manual_yaw_scale'] = 0.2
    app.open_events = lambda: Events(app.clock, ['press:left', None, 'x'])
    with app.attached():
        arm(app)
        app.dispatch('m')
    yaws = [c[2] for c in boat.sent if c != 'neutral' and c[2] > 0]
    amplitude = app.args.hardware_config['manual_amplitude']
    assert yaws and yaws[0] == pytest.approx(0.2 * amplitude)


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

    @contextlib.contextmanager
    def costmap_lease(self, who):
        yield self

    def fetch_costmap(self, timeout=2.0):
        return self.costmap

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
    app, boat = make_app(tmp_path, ['--set', 'command_mode=velocity', *argv])
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
    app, _ = nav_app(tmp_path, ['--set', 'nav_distance=3'])
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
    app, _ = nav_app(tmp_path, ['--set', 'station_hold=20', '--set', 'station_tolerance=1'])
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
    app, _ = nav_app(tmp_path, ['--set', 'observe=2'])
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
    run(app, 'p')
    checks = {r['check']: r['verdict'] for r in app.snapshot()['results']}
    assert checks['LiDAR'] == 'PASS' and 'camera' not in checks
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
    (120, ['ctrl-right', 'ctrl-right', 'ctrl-down', 'ctrl-down', 'ctrl-left'], 'procedures'),
    (120, ['ctrl-k', 'ctrl-h'], 'status'),               # no panel there: focus stays
    (120, ['ctrl-l', 'ctrl-l', 'ctrl-j', 'ctrl-k'], 'conditions'),
    (100, ['ctrl-l', 'ctrl-l', 'ctrl-j'], 'thrusters'),
    (80, ['ctrl-l', 'ctrl-j', 'ctrl-j', 'ctrl-j', 'ctrl-k'], 'thrusters'),   # stack: Motion, Conditions, Thrusters
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


@pytest.mark.parametrize('down,up', [('down', 'up'), ('j', 'k')])
def test_grab_moves_a_thruster_and_undo_restores(tmp_path, down, up):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    app.focus = 'thrusters'
    app.dispatch('r')        # FL reversed
    app.dispatch('enter')    # pick FL up
    app.dispatch(down)
    app.dispatch(up)         # navigation also carries the grabbed thruster back
    app.dispatch(down)
    app.dispatch(down)       # carried to RR's row: FL<->FR, then FR<->RR swaps
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


def test_shifted_keys_still_quit_and_cancel_prompts(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        press(app, 'enter')
        app.dispatch('Q')
        assert app.quit
        press(app, 'Q')
        with pytest.raises(Cancelled):
            app.prompt('Ready?')


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
    assert (tmp_path / 'session' / 'analysis' / 'report.json').exists()
    assert dict(app.summary())['Results'] == '0/0 good'


def test_arm_panel_lists_interlocks_before_arming(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'], sensors=False)
    rows = dict(app.arm_rows())
    assert rows['Outputs'].startswith('simulator') and rows['Driver'] == 'starts on arm'
    assert rows['Interlocks'].startswith('✗')


def test_arm_panel_shows_the_driver_check_and_wraps_long_reasons(tmp_path):
    app, boat = make_app(tmp_path)
    assert dict(app.arm_rows())['Interlocks'] == theme.icon('ok').plain + ' clear'
    app.arming.rival = lambda: 'another thruster_pwm_node already drives /thrusters/pwm'
    boat.conflicts = lambda: ['/cmd_vel also published by /mhseals_teleop_node_12345 (pid 12345)']
    rows = app.arm_rows()
    start = [label for label, _ in rows].index('Interlocks')
    shown = rows[start:]
    assert 'already drives' in ' '.join(value for _, value in shown)
    assert any('12345)' in value for _, value in shown)
    assert all(len(value) <= 54 for _, value in shown)


@pytest.mark.parametrize('velocity', [False, True])
def test_status_tracks_driver_and_kernel_pwm_independently(tmp_path, velocity):
    argv = ['--set', 'command_mode=velocity'] if velocity else []
    app, _ = make_app(tmp_path, argv)
    chips = [tmp_path / f'pwmchip{index}' for index in range(4)]
    for chip in chips:
        pin = chip / 'pwm0'
        pin.mkdir(parents=True)
        for field, value in [('enable', '0'), ('period', '20000000'),
                             ('duty_cycle', '1500000')]:
            (pin / field).write_text(value)
    app.args.hardware_config.update(pwm_chips=chips, pwm_channels=[0] * 4)
    drivers, managed, topics = [], False, []
    app.processes.running = lambda name: managed if name == 'hardware' else True
    app.ros.nodes = lambda: list(drivers)
    app.ros.subscribers = lambda topic: topics.append(topic) or drivers

    def sample():
        app.clock.now += 1
        app.refresh_advisories()
        return app.snapshot()['advisories']

    status = sample()
    assert status['PWM driver']['detail'].startswith('STOPPED;')
    assert status['PWM readback']['level'] == 'OK'
    assert status['PWM readback']['detail'].startswith('0/4 ON;')
    drivers.append('/thruster_pwm_node')
    managed = app.arming.armed = True
    for chip in chips:
        (chip / 'pwm0' / 'enable').write_text('1')
    status = sample()
    assert status['PWM driver']['level'] == status['PWM readback']['level'] == 'OK'
    assert status['PWM driver']['detail'].startswith('RUNNING;')
    assert status['PWM readback']['detail'].startswith('4/4 ON;')
    assert set(topics) == {'/cmd_vel_effort' if velocity else '/cmd_vel/applied'}
    assert app.pwm_readback == ['ON 1500us / 50Hz'] * 4

    # A live driver endpoint must not hide a disabled hardware output.
    (chips[2] / 'pwm0' / 'enable').write_text('0')
    status = sample()
    assert status['PWM driver']['level'] == 'OK'
    assert status['PWM readback']['level'] == 'FAIL'
    assert app.pwm_readback[2].startswith('OFF')
    drivers.clear()
    assert sample()['PWM driver']['level'] == 'FAIL'
    app.arming.armed = managed = False
    drivers.append('/thruster_pwm_node')
    status = sample()
    assert status['PWM driver']['level'] == status['PWM readback']['level'] == 'WARN'


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


def test_manual_notice_labels_the_chord_modes_like_keyboard_control(tmp_path):
    app, _ = make_app(tmp_path)
    app.console.size = (120, 40)
    app.manual_show((0.0, 0.0, 0.0), 0.5, True)
    assert 'Chords' in app.notice and 'held keys' not in app.notice
    app.manual_show((0.0, 0.0, 0.0), 0.5, False)
    assert 'Single key' in app.notice and 'tap-repeat' not in app.notice


def test_manual_control_stops_on_an_interlock(tmp_path):
    app, boat = make_app(tmp_path)
    app.open_events = lambda: Events(app.clock, ['press:w'] + ['repeat:w'] * 80)
    with app.attached():
        arm(app)
        app.ros.fault = 'executor stopped'
        try:
            app.dispatch('m')
        except Exception as error:  # run() turns this into a stop
            app.stop(str(error))
    assert not app.armed and boat.sent[-1] == 'neutral'


def test_manual_interlock_trip_is_journaled_as_a_guard_stop(tmp_path):
    app, boat = make_app(tmp_path)

    class Tripping(Events):
        def read(self, timeout=None):
            if len(self.keys) < 4:
                app.ros.fault = 'executor stopped'  # trips between two drive ticks
            return super().read(timeout)

    app.open_events = lambda: Tripping(app.clock, ['press:w', None, 'repeat:w', None, 'repeat:w', None])
    with app.attached():
        arm(app)
        try:
            app.dispatch('m')
        except Exception as error:  # run() turns this into a stop
            app.stop(str(error))
    phases = [(p, d) for p, d in app.ros.lines if p.startswith('manual_')]
    start, stop = phases[0][1], phases[-1][1]
    assert [p for p, _ in phases] == ['manual_start', 'manual_stop']
    assert stop['by'] == 'guard' and stop['cause'] == 'guard:ROS fault'
    assert start['run_id'] == stop['run_id'] and boat.sent[-1] == 'neutral'


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
    assert (tmp_path / 'session' / 'analysis' / 'report.json').exists()


@pytest.mark.parametrize('error', [EOFError('terminal input disconnected'),
                                   OSError(errno.EIO, 'Input/output error')])
def test_a_vanished_terminal_closes_like_quit(tmp_path, error):
    app, boat = make_app(tmp_path)
    app.arming.armed = True

    class Gone(Keys):
        def read(self, timeout):
            raise error

    app.open_keys = lambda: Gone(app.clock, boat)
    app.run()
    assert app.quit and app.closed and not app.armed and boat.sent[-1] == 'neutral'
    assert app.ros.events.count('session_stop') == 1


def test_children_die_with_the_suite(tmp_path):
    import os, signal, subprocess, sys, time
    # A middle process starts a child through ProcessManager, then is killed hard.
    script = (f"import sys, time; sys.path[:0] = {sys.path!r}\n"
              "from mhseals_testing.harness.processes import ProcessManager\n"
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
@pytest.mark.parametrize('key', ['f'])
def test_characterize_shortcut_reaches_procedure_from_every_panel(tmp_path, focus, key):
    app, _ = make_app(tmp_path)
    app.focus = focus
    called = []
    app.run_procedure = lambda p: called.append(p.key)
    app.dispatch(key)
    assert called == [key]


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


def test_arming_shows_progress_and_space_cancels_a_slow_driver(tmp_path):
    app, boat = make_app(tmp_path)

    class Slow(type(app.arming)):
        def begin(self):
            self.calls.append('begin')

        def poll(self):
            return 'starting'

    app.arming = Slow()
    app.open_events = lambda: Events(app.clock, [])
    with app.attached():
        app.keys.script[:0] = [None, None, 'space']
        with pytest.raises(Exception, match='cancelled'):
            app.wait_for_driver()
    assert app.arming.calls == ['begin', 'cancel'] and not app.armed
    assert 'starting the thruster driver' in app.notice


def test_manual_control_arms_without_odometry_but_trials_are_unavailable(tmp_path):
    app, boat = make_app(tmp_path, sensors=False)  # no odometry or TF ever arrives
    with app.attached():
        arm(app)
        assert app.manual_blockers() == []
        trial = next(p for p in PROCEDURES if 'sensors' in p.needs)
        assert app.availability(trial)  # can't be picked without odometry


def test_characterization_never_offers_to_install_the_secant_decoupling(tmp_path):
    app, _ = make_app(tmp_path, ['--set', 'amplitude=0.4', '--set', 'command=3'])


def test_characterization_offers_only_save_and_writes_unit_diagonal_columns(tmp_path):
    app, _ = make_app(tmp_path, ['--set', 'amplitude=0.4', '--set', 'command=3'])
    offered = []
    original = app.procedure_context

    def context():
        ctx = original()
        ctx.choose = lambda title, options, shortcuts=None, **kw: offered.append(options) or 'save'
        ctx.prompt = lambda message, title=None: ''
        return ctx

    with app.attached():
        arm(app)
        app.procedure_context = context
        app.dispatch('f')
    assert offered and all('save + decouple' not in options for options in offered)
    saved = yaml.safe_load((tmp_path / 'hardware.yaml').read_text())
    for key in ('decoupling', 'decoupling_negative'):  # absent C- equals C+
        assert [saved.get(key, saved['decoupling'])[i] for i in (0, 4, 8)] == [1.0, 1.0, 1.0]


def test_mid_write_operator_config_never_reaches_the_armed_snapshot(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text('channel_map: [3, 1, 2, 4]\nreversed: [true, false, false, true]\n')
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    with app.attached():
        arm(app)
        config.write_text(config.read_text() + 'manual_amplitude: 0.3\n')
        app.watch_config()
        app.watch_config()  # settled, so the armed snapshot now exists
        before = app.paths.hardware.read_text()
        with open(config, 'w') as handle:  # an in-place save caught after its first chunk
            handle.write('manual_amplitude: 0.35\n')
            handle.flush()
            app.watch_config()
            assert app.paths.hardware.read_text() == before
            handle.write('channel_map: [3, 1, 2, 4]\nreversed: [true, false, false, true]\n')
        app.watch_config()  # settling: the stat key moved again since the last poll
        app.watch_config()  # unchanged on two polls, so now it is applied
    snapshot = yaml.safe_load(app.paths.hardware.read_text())
    assert snapshot['channel_map'] == [3, 1, 2, 4] and snapshot['reversed'] == [True, False, False, True]


@pytest.mark.parametrize('text, message', [
    ('reversed: [true, false', 'hardware.yaml'),
    ('- 1\n- 2\n', 'mapping'),
    ('channel_map: [0, 0, 1, 2, 3, 4, 5, 6]\n', 'permutation'),
    ('no_such_setting: 1\n', 'unknown hardware settings'),
])
def test_parse_args_broken_hardware_yaml_names_the_file(tmp_path, capsys, text, message):
    path = tmp_path / 'hardware.yaml'
    path.write_text(text)
    for argv in (['--config', str(path)], [f'--config={path}']):
        with pytest.raises(SystemExit) as exit_info:
            parse_args(argv)
        assert exit_info.value.code == 2
        err = capsys.readouterr().err
        assert str(path) in err and message in err and 'Traceback' not in err


def test_pins_popup_applies_through_the_driver_readback(tmp_path):
    app, _ = make_app(tmp_path)
    app.ros.parameter_service = lambda name: True
    app.ros.driver_parameters = lambda name, keys: {k: app.args.hardware_config[k] for k in keys}
    app.ask_text = lambda *a: '1 0 0 0 29'
    app.focus = 'thrusters'
    app.dispatch('M')
    assert app.notice == 'Pins applied'
    assert app.args.hardware_config['pwm_channels'] == [1, 0, 0, 0]


def test_pins_popup_saves_for_the_next_arm_without_a_driver(tmp_path):
    app, _ = make_app(tmp_path)
    app.ros.parameter_service = lambda name: False
    app.ros.driver_parameters = lambda name, keys: pytest.fail('no driver to read back')
    app.ask_text = lambda *a: '1 0 0 0 29'
    app.focus = 'thrusters'
    app.dispatch('M')
    assert app.notice == 'Pins saved; applied at next arm'
    assert app.args.hardware_config['pwm_channels'] == [1, 0, 0, 0]


def test_pins_popup_refuses_while_armed(tmp_path):
    app, _ = make_app(tmp_path)
    app.arming.armed = True
    app.ask_text = lambda *a: '1 0 0 0 29'
    app.focus = 'thrusters'
    app.dispatch('M')
    assert app.notice.startswith('Pins refused')
    assert app.args.hardware_config['pwm_channels'] != [1, 0, 0, 0]


def test_bad_theme_overlay_is_the_first_notice(tmp_path, monkeypatch):
    monkeypatch.setattr(theme, 'OVERLAY_WARNING', 'theme overlay x ignored: bad')
    assert make_app(tmp_path)[0].notice == 'theme overlay x ignored: bad'
    monkeypatch.setattr(theme, 'OVERLAY_WARNING', None)
    assert make_app(tmp_path)[0].notice.startswith('Passive monitor')


class Registry(RegistryBase):
    """A registry whose pids are all alive, in the test's own directory."""

    def __init__(self, directory):
        super().__init__(directory / 'session.json', alive=lambda pid: True)


def test_sigusr1_detaches_at_neutral_and_leaves_the_children_recorded(tmp_path):
    app, boat = make_app(tmp_path)
    app.registry = Registry(tmp_path / 'run')
    app.registry.claim({'pid': os.getpid(), 'node': 'n'})
    app.registry.update(lambda state: state.update(handover={'to': 77, 'deadline': 1e12}))
    app.arming.armed = True

    class Signalled(Keys):
        def read(self, timeout):
            raise Detach

    app.open_keys = lambda: Signalled(app.clock, boat)
    app.run()
    assert app.detaching and not app.armed and boat.sent[-1] == 'neutral'
    assert 'handover' in app.ros.events and 'session_stop' not in app.ros.events
    assert app.processes.released == ['bag'] and not hasattr(app.processes, 'stopped')
    assert app.registry.owner() is None and app.registry.children() == {'bag': {'pid': 9, 'pgid': 9}}


def test_a_watcher_cannot_arm_or_run_but_space_still_sends_neutral(tmp_path):
    app, boat = make_app(tmp_path)
    app.watching = 4012
    app.dispatch('a')
    assert not app.armed and 'no arming' in app.notice
    assert 'watching pid 4012' in app.availability(procedure('f'))
    app.dispatch('space')
    assert boat.sent[-1] == 'neutral'


def test_a_watcher_closing_leaves_the_owners_motion_events_and_report_alone(tmp_path):
    app, boat = make_app(tmp_path)
    app.watching = 4012
    sent = len(boat.sent)
    with app.attached():
        app.close()
    assert len(boat.sent) == sent and 'session_stop' not in app.ros.events
    assert not (tmp_path / 'session' / 'analysis' / 'report.json').exists()


def test_a_stale_dds_node_is_an_advisory_for_30_s_then_blocks_arming(tmp_path):
    app, _ = make_app(tmp_path)
    app.ros.instances = lambda: ['mhseals_testing_h_5']
    assert app.foreign_instances() == (['mhseals_testing_h_5'], []) and not app.foreign_blockers()
    app.clock.now += 29
    assert not app.foreign_blockers()
    app.clock.now += 2
    assert 'mhseals_testing_h_5' in app.foreign_blockers()[0]
    app.ros.instances = lambda: []                  # the lease expired: nothing left to block
    assert app.foreign_blockers() == [] and app.foreign_seen == {}


def test_a_registered_instance_is_not_foreign_and_guests_are_announced(tmp_path):
    app, _ = make_app(tmp_path)
    app.registry = Registry(tmp_path / 'run')
    app.registry.claim({'pid': 5, 'node': 'mhseals_testing_h_5'})
    app.ros.instances = lambda: ['mhseals_testing_h_5']
    assert app.foreign_instances() == ([], [])
    app.registry.add_guest(8, 'boat_trial_8', 'characterize')
    app.sync_guests()
    app.registry.remove_guest(8)
    app.sync_guests()
    assert app.ros.events == ['guest_attach', 'guest_detach']


def journaling(app):
    """Make the fake ROS append to the session journal like RosIO.emit."""
    from mhseals_testing.harness import journal

    def emit(phase, **details):
        journal.append(app.paths.journal, events.payload(phase, 1.0, details))
    app.ros.emit = emit
    return journal


def test_b_marks_and_asks_for_a_note_when_idle(tmp_path):
    app, _ = make_app(tmp_path)
    journal = journaling(app)
    with app.attached():
        press(app, 'w', 'space', 'o', 'enter')
        app.dispatch('b')
        assert [(m['id'], m['note']) for m in journal.markers(journal.read(app.paths.journal))] == [(1, 'w o')]
        press(app, 'x', 'escape')  # Esc keeps the marker bare
        app.dispatch('b')
        press(app, 'n', 'enter')
        app.dispatch('B')  # raw capital: note on the last marker
    assert [m['note'] for m in journal.markers(journal.read(app.paths.journal))] == ['w o', 'n']


def test_no_popup_while_moving(tmp_path):
    app, _ = make_app(tmp_path)
    journal = journaling(app)
    app.running = 'Manual'
    app.dispatch('b')  # a popup would fail the test: no answers queued
    assert app.notice == f'{theme.pick("▾", "v")}1 marked{theme.SEPARATOR}B adds a note'
    app.dispatch('B')
    [mark] = journal.markers(journal.read(app.paths.journal))
    assert mark['note'] == '' and app.notice == 'Not while moving'


def test_one_run_file_per_procedure_run_with_settings(tmp_path):
    from mhseals_testing.procedures import Procedure

    def several(ctx):
        for name in 'abc':
            ctx.record(dict(procedure=name, verdict='OK'))

    app, _ = make_app(tmp_path)
    app.run_procedure(Procedure('z', 'nav2 bring-up', several))
    files = sorted((tmp_path / 'session' / 'runs').glob('*.json'))
    assert [f.name for f in files] == ['001-nav2-bring-up.json']
    saved = json.loads(files[0].read_text())
    assert len(saved['result']['results']) == 3 and saved['status'] == 'MEASURED'
    assert saved['settings']['amplitude'] == app.settings.amplitude
    assert {r['run_id'] for r in saved['result']['results']} == {'001-nav2-bring-up'}


def test_error_out_of_a_procedure_leaves_one_aborted_run(tmp_path):
    from mhseals_testing.procedures import Procedure
    from mhseals_testing.runner import Aborted

    def stopped(ctx):
        ctx.record(dict(procedure='a', verdict='OK'))
        raise Aborted('interlock')

    app, _ = make_app(tmp_path)
    with pytest.raises(Aborted):
        app.run_procedure(Procedure('z', 'step', stopped))
    files = list((tmp_path / 'session' / 'runs').glob('*.json'))
    assert len(files) == 1
    saved = json.loads(files[0].read_text())
    assert saved['status'] == 'ABORTED' and saved['result']['reason'] == 'interlock'


def test_o_on_status_opens_the_odometry_popup_without_running_perception(tmp_path):
    app, _ = make_app(tmp_path)
    shown = []
    app.choose = lambda title, options, shortcuts=None, rows=None, **kw: shown.append((title, rows)) or 'Close'
    app.ran = []
    app.run_procedure = app.ran.append
    app.focus = 'status'
    app.dispatch('o')
    assert shown[0][0] == 'Odometry' and shown[0][1][0] == ('Estimates', '')
    assert ('ekf_local', 'parameters unavailable') in shown[0][1] and not app.ran
    app.focus = 'procedures'
    app.dispatch('p')                       # Perception moved to p (procedure groups)
    assert app.ran and len(shown) == 1


class ReplaySource:
    """A source that is not live: a fixed snapshot and no host access."""

    live = False

    def __init__(self, snapshot):
        self._snapshot = snapshot
        self.ticks = []

    def snapshot(self):
        return self._snapshot

    def tick(self, now):
        self.ticks.append(now)


def test_review_source_feeds_snapshots_and_skips_live_only_work(tmp_path):
    app, boat = make_app(tmp_path)
    marker = object()
    app.source = ReplaySource(marker)
    called = []
    app.refresh_advisories = lambda: called.append('advisories')
    app.watch_config = lambda: called.append('watch')
    app.tick()
    assert app.snapshot() is marker
    assert app.source.ticks == [app.clock()] and called == []


def test_review_source_refuses_arming_and_procedures(tmp_path):
    app, boat = make_app(tmp_path)
    app.source = ReplaySource(app.snapshot())
    with app.attached():
        app.dispatch('a')
        assert not app.armed and app.notice == 'Arming is disabled in review'
        app.run_procedure(procedure('m'))
    assert app.notice == 'Procedures are disabled in review' and boat.sent == []


def test_clear_decoupling_also_clears_the_negative_matrix(tmp_path):
    split = [1, 0, 0, 0, 1, 0.3, 0, 0, 1]
    (tmp_path / 'hardware.yaml').write_text(
        f'frequency: 50.0\nchannel_map: [1, 2, 3, 4]\ndecoupling_negative: {split}\n')
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    assert decoupled(app.args.hardware_config) is not None  # C- alone counts
    with app.attached():
        app.focus = 'thrusters'
        press(app, 'y')
        app.dispatch('c')
    saved = yaml.safe_load((tmp_path / 'hardware.yaml').read_text())
    assert saved.get('decoupling_negative') is None
    assert decoupled(app.args.hardware_config) is None




def test_save_hardware_records_after_logging_a_hand_edit(tmp_path):
    app, _ = make_app(tmp_path)
    (tmp_path / 'hardware.yaml').write_text('frequency: 50.0\nchannel_map: [4, 3, 2, 1]\n')   # hand edit
    assert app.save_hardware(channel_map=[2, 1, 3, 4])
    changes = app.history.changes('hardware')
    assert [c['source'] for c in changes] == ['external', 'tui:hardware']
    assert app.history.version(changes[-1])['value']['overrides']['channel_map'] == [2, 1, 3, 4]


def test_arming_copy_is_never_recorded(tmp_path):
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack'])
    with app.attached():
        arm(app)
        app.save_hardware(channel_map=[2, 1, 3, 4])
        app.tick()                                  # reload copies the config for the driver
        assert app.paths.hardware.exists()
        assert [c['source'] for c in app.history.changes()] == ['external', 'tui:hardware']   # the first file seen, then our write




def test_bad_yaml_keeps_the_old_config_with_one_notice_per_distinct_error(tmp_path):
    app, _ = make_app(tmp_path)
    config = tmp_path / 'hardware.yaml'
    config.write_text('reversed: [true, false, false, false]\n')
    app.announce(app.watcher.reload('hardware'))
    config.write_text('reversed: [true, false\n')
    app.watch_config()
    app.watch_config()  # settled: parsed once
    assert 'rejected' in app.notice and app.args.hardware_config['reversed'][0] is True
    app.notice = ''
    config.write_text('reversed:  [true, false\n')  # the same error again
    app.watch_config()
    app.watch_config()
    assert app.notice == ''
    config.write_text('reversed: [false, false, false, false]\n')
    app.watch_config()
    app.watch_config()
    assert app.args.hardware_config['reversed'][0] is False and 'reloaded' in app.notice


def test_config_row_and_popup_list_the_watched_files(tmp_path):
    app, _ = make_app(tmp_path)
    app.watch_config()
    assert app.monitor.advisories['Config'][1] == 'OK' and '1 file' in app.monitor.advisories['Config'][2]
    modals = []
    app.popup = lambda modal, handle, refresh=None: modals.append(modal)
    app.open('config')
    assert modals[0]['title'] == 'Config' and 'hardware' in modals[0]['rows'][0][0]
    (tmp_path / 'hardware.yaml').write_text('channel_map: 5\n')
    app.announce(app.watcher.reload('hardware'))
    app.watch_config()
    assert app.monitor.advisories['Config'][1] == 'WARN'


# --- Nav2 parameters from the Navigation row (ts089) --------------------------------------


def nav2_app(tmp_path, applier=None, **kwargs):
    from mhseals_testing.harness.nav2_edit import Nav2File
    path = tmp_path / 'nav2.yaml'
    path.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.5\n')
    app, boat = make_app(tmp_path, **kwargs)
    app.nav2_file = Nav2File(path, applier)
    return app, path


def edit_nav2(app, path, text):
    path.write_text(text)
    app.tick()  # sees the new stat key
    app.tick()  # unchanged for one tick: settled


def test_nav2_file_change_toasts_and_marks_navigation_pending(tmp_path):
    app, path = nav2_app(tmp_path)
    assert next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')['value'] == 'pending'
    edit_nav2(app, path, 'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.8\n')
    assert app.notice.startswith('nav2.yaml changed')
    assert next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')['value'] == 'pending'


def dashboard_rows(app):
    from mhseals_testing.ui.panels.status import status_rows
    return status_rows(app.snapshot(), app.profile.sensors)


def test_e_on_navigation_previews_and_y_applies_once(tmp_path):
    seen = []
    app, path = nav2_app(tmp_path, applier=lambda changes: seen.append(changes) or dict(status='verified', verified=changes))
    edit_nav2(app, path, 'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.8\n')
    app.focus = 'status'
    app.cursors['status'] = next(i for i, r in enumerate(dashboard_rows(app)) if r['name'] == 'Navigation')
    press(app, 'y')
    with app.attached():
        app.dispatch('e')
    assert seen == [{'controller_server.ros__parameters.FollowPath.vx_max': 0.8}]
    assert app.notice == 'Nav2 applied 1 key(s)'
    assert next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')['value'] == 'applied'


def test_e_opens_the_nav2_popup_only_on_the_navigation_row(tmp_path):
    app, path = nav2_app(tmp_path)
    opened = []
    app.nav2_popup = lambda: opened.append(1)
    app.focus = 'status'
    rows = dashboard_rows(app)
    other = next(i for i, r in enumerate(rows) if r['group'] == 'Components' and r['name'] != 'Navigation')
    app.cursors['status'] = other
    app.dispatch('e')
    assert not opened
    app.cursors['status'] = next(i for i, r in enumerate(rows) if r['name'] == 'Navigation')
    app.dispatch('e')
    assert opened == [1]


def test_escape_leaves_navigation_pending_and_a_bad_edit_keeps_the_last_good(tmp_path):
    app, path = nav2_app(tmp_path)
    edit_nav2(app, path, 'controller_server: [unclosed\n')
    assert 'rejected' in app.notice
    edit_nav2(app, path, 'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.9\n')
    app.focus = 'status'
    app.cursors['status'] = next(i for i, r in enumerate(dashboard_rows(app)) if r['name'] == 'Navigation')
    press(app, 'escape')
    with app.attached():
        app.dispatch('e')
    assert next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')['value'] == 'pending'


@pytest.mark.parametrize('size', [(120, 40), (80, 30)])
def test_navigation_row_and_popup_render_at_120_and_80(tmp_path, size):
    app, path = nav2_app(tmp_path)
    edit_nav2(app, path, 'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.8\n')
    app.console = Console(file=io.StringIO(), width=size[0], height=size[1], color_system=None, record=True)
    app.console.print(app.render())
    text = app.console.export_text()
    assert 'Navigation' in text and 'pending' in text
    app.modal = dict(kind='choice', title='Nav2 parameters', message=None, options=['Apply', 'Close'],
                     selected=0, danger=False, rows=[('vx_max', '0.5 -> 0.8')], width=76,
                     hint='', keys=[None, None], scroll_hint='[ ]')
    app.console.print(app.render())
    assert 'Nav2 parameters' in app.console.export_text()


def test_nav2_file_diff_and_apply_refusal(tmp_path):
    from mhseals_testing.harness.nav2_edit import diff
    assert diff({'a': {'b': 1, 'c': 2}}, {'a': {'b': 1, 'c': 3}, 'd': 4}) == [('a.c', 2, 3), ('d', None, 4)]
    app, path = nav2_app(tmp_path, applier=lambda changes: {'refused': 'goal active'})
    edit_nav2(app, path, 'a: 1\n')
    assert app.nav2_file.apply() == 'Nav2 apply refused: goal active'
    assert app.nav2_file.pending


def test_deleted_nav2_file_keeps_the_last_good_and_sends_no_none(tmp_path):
    seen = []
    app, path = nav2_app(tmp_path, applier=lambda changes: seen.append(changes) or dict(status='verified', verified=changes))
    app.nav2_file.apply()
    seen.clear()
    path.unlink()
    app.tick()
    app.tick()
    assert 'changed' not in app.notice
    assert next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')['value'] == 'error'
    edit_nav2(app, path, 'a: 1\n')  # the old FollowPath key is now removed from the file
    assert app.nav2_file.removed() == ['controller_server.ros__parameters.FollowPath.vx_max']
    app.nav2_file.apply()
    assert seen == []


def test_edit_between_preview_and_apply_stays_pending(tmp_path):
    app, path = nav2_app(tmp_path, applier=lambda changes: dict(status='verified', verified=changes))
    edit_nav2(app, path, 'a: 1\n')
    previewed = app.nav2_file.current
    edit_nav2(app, path, 'a: 2\n')
    app.nav2_file.apply(previewed)
    assert app.nav2_file.changes() == [('a', 1, 2)]


def test_nav2_text_is_ascii_safe_in_ascii_mode(tmp_path, monkeypatch):
    from mhseals_testing.ui import theme
    monkeypatch.setattr(theme, 'SEPARATOR', ' | ')
    app, path = nav2_app(tmp_path)
    edit_nav2(app, path, 'a: 1\n')
    row = next(r for r in dashboard_rows(app) if r['name'] == 'Navigation')
    assert '·' not in app.notice and '·' not in row['detail'] and '|' in app.notice




def test_events_carry_run_id_by_and_cause_for_the_timeline(tmp_path):
    app, _ = make_app(tmp_path, ['--set', 'amplitude=0.4', '--set', 'command=3'])
    with app.attached():
        arm(app)
        press(app, 'enter', 'n')
        app.dispatch('f')
        app.disarm('done', by='fault')
    lines = dict(reversed(app.ros.lines))
    start = next(d for p, d in app.ros.lines if p == 'characterization_start')
    stop = lines['characterization_stop']
    assert start['run_id'] == stop['run_id'] and start['key'] == 'f'
    assert start['procedure'] == 'Characterize' and stop['by'] == 'procedure'
    result = next(d for p, d in app.ros.lines if p == 'result' and d.get('verdict') in ('FITTED', 'PARTIAL'))
    assert result['run_id'] == start['run_id']
    t0, t1 = result['window']
    edges = [result['slices'][0]['window'][0]] + [s['window'][1] for s in result['slices']]
    assert (t0, t1) == (edges[0], edges[-1])
    assert all(a['window'][1] == b['window'][0] for a, b in zip(result['slices'], result['slices'][1:]))
    assert dict(app.ros.lines)['disarmed']['by'] == 'fault'


def test_manual_drive_is_bracketed_by_manual_events(tmp_path):
    app, _ = make_app(tmp_path)
    app.console.size = (120, 40)
    app.open_events = lambda: Events(app.clock, ['press:w', None, 'press:x'])
    with app.attached():
        arm(app)
        app.dispatch('m')
    phases = [(p, d) for p, d in app.ros.lines if p.startswith('manual_')]
    assert [p for p, _ in phases] == ['manual_start', 'manual_stop']
    assert phases[1][1]['by'] == 'operator' and phases[0][1]['run_id'] == phases[1][1]['run_id']




def test_disk_trip_marks_then_stops_the_bag_and_says_so(tmp_path):
    from mhseals_testing.harness.recwatch import DISK_MARKER, RecWatch
    app, _ = make_app(tmp_path)
    order = []
    app.ros.emit = lambda phase, **details: order.append((phase, details.get('note')))
    app.processes.stop = lambda name, timeout=10: order.append(('stop', name))
    app.recwatch = RecWatch(app.clock, free=lambda _: 0.5, size=lambda _: 0)
    app.watch_recording()
    assert order == [('marker', DISK_MARKER), ('stop', 'bag')]
    assert DISK_MARKER in app.notice




def test_enter_on_a_result_opens_the_run_popup_and_capital_x_excludes_it(tmp_path):
    app, _ = make_app(tmp_path)
    app.monitor.record_result(dict(procedure='axis', verdict='SIGN MATCH'))
    app.monitor.record_result(dict(procedure='step', verdict='OK'))
    app.cursors['activity'] = 1  # the older result (the list is newest first)
    with app.attached():
        press(app, 'X', 'enter', 'escape')  # exclude, first reason, close
        app.on_activity('enter')
    first, second = app.monitor.snapshot()['results']
    assert first['excluded']['reason'] == 'operator error' and 'excluded' not in second


def test_run_popup_stays_on_its_run_when_a_result_lands(tmp_path):
    app, _ = make_app(tmp_path)
    app.monitor.record_result(dict(procedure='axis', verdict='SIGN MATCH'))
    app.cursors['activity'] = 0
    reader = app.reader
    read = reader.read

    def landing(timeout):
        key = read(timeout)
        if key == 'down':  # a harmless popup key; a new result lands while the popup is open
            app.monitor.record_result(dict(procedure='step', verdict='OK'))
        return key

    reader.read = landing
    with app.attached():
        press(app, 'down', 'X', 'enter', 'escape')
        app.on_activity('enter')
    first, second = app.monitor.snapshot()['results']
    assert first['excluded']['reason'] == 'operator error'
    assert 'excluded' not in second


class FakeFeed:
    def __init__(self):
        self.ticks = []

    def tick(self, app, snapshot, now):
        self.ticks.append(now)

    def view(self):
        return {'fake_key': 42}


def test_feed_view_reaches_layout_render_and_feeds_tick(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path)
    feed = FakeFeed()
    app.feeds.append(feed)
    seen = {}
    monkeypatch.setattr('mhseals_testing.ui.layout.render', lambda snapshot, view, *a, **k: seen.update(view))
    app.tick()
    app.render()
    assert seen['fake_key'] == 42 and feed.ticks


def test_app_view_keys_win_a_feed_collision(tmp_path):
    app, _ = make_app(tmp_path)
    feed = FakeFeed()
    feed.view = lambda: {'tab': 'bogus', 'fake_key': 1}
    app.feeds.append(feed)
    view = app.view()
    assert view['tab'] == app.tab and view['fake_key'] == 1


def test_unbound_action_is_a_noop_and_odometry_opens_through_the_table(tmp_path):
    app, _ = make_app(tmp_path)
    assert app.open('no_such_action') is False
    shown = []
    app.choose = lambda title, *args, **kw: shown.append(title) or 'Close'
    assert app.open('odometry') is True
    assert shown == ['Odometry']


def test_a_hand_edit_reaches_the_history_when_the_watcher_reloads_it(tmp_path):
    app, _ = make_app(tmp_path)
    config = tmp_path / 'hardware.yaml'
    config.write_text('frequency: 50.0\n')
    app.watch_config()
    app.watch_config()
    assert app.history.head('hardware')['source'] == 'external'
    count = len(app.history.changes('hardware'))
    app.watch_config()
    assert len(app.history.changes('hardware')) == count   # nothing new, nothing recorded twice


@pytest.mark.parametrize('key', ['E', 'M', 'G', 'N', 'W', 'S', 'T', 'I', 'F', 'X', 'U'])
def test_shifted_keys_never_start_a_procedure(tmp_path, key):
    app, _ = make_app(tmp_path)
    started = []
    app.on_procedures = started.append
    app.run_procedure = lambda proc, *a, **k: started.append(proc)
    app.ask_text = app.choose = lambda *a, **k: None   # M (pins) and X (exclude) are page keys that ask
    for focus in ('procedures', 'motion', 'activity', 'thrusters'):
        if focus in app.panels():
            app.focus = focus
            app.dispatch(key)
    assert started == []
    assert not app.moving()


def test_shift_e_revokes_the_agent_claim_and_shift_u_is_consumed(tmp_path):
    from mhseals_testing.ros.authority import Arbiter
    app, _ = make_app(tmp_path)
    holds = []
    app.arbiter = Arbiter(lambda record: None, lambda phase, **fields: None, tui_owner=True,
                          neutral=lambda: None, hold=lambda cause, by: holds.append((cause, by)))
    app.dispatch('E')
    assert app.notice == 'No agent claim to revoke' and holds == []
    claim = app.arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'tune'})['claim']
    app.dispatch('E')
    assert app.notice == 'Agent revoked' and holds == [('revoke', 'tui')]
    assert app.arbiter.table.get(claim['id'])['state'] == 'revoked'
    app.dispatch('U')
    assert app.notice == 'Release hold is not available here'
    released = []
    app.plant = __import__("types").SimpleNamespace(release_hold=lambda: released.append(1) or True)
    app.arbiter.on_mux({'input': 'hold', 'rc': {'mode': 'GUIDED'}})
    with app.attached():
        press(app, 'up', 'enter')
        app.dispatch('U')
    assert app.notice == 'Release requested' and released == [1]


@pytest.mark.parametrize('open_popup, answers', [
    (lambda app: app.show('Odometry', message='x'), ('space', 'escape')),
    (lambda app: app.choose('Pick', ['a', 'b']), ('space', 'escape')),
    (lambda app: app.prompt('Ready?'), ('space', 'escape')),
])
def test_space_stops_inside_popups(tmp_path, open_popup, answers):
    """Keymap rule 5: Space disarms inside message, choice and prompt popups."""
    app, _ = make_app(tmp_path)
    with app.attached():
        arm(app)
        press(app, *answers)
        try:
            open_popup(app)
        except Cancelled:
            pass
        assert not app.armed


def test_space_types_in_text_popups_without_stopping(tmp_path):
    app, _ = make_app(tmp_path)
    with app.attached():
        arm(app)
        press(app, 'space', 'x', 'enter')
        assert app.text_input('Name', 'say') == 'x'
        assert app.armed



def test_map_tab_keys_toggle_layers_and_the_bags_tab_lists_sessions(tmp_path):
    app, _ = make_app(tmp_path)
    app.show_tab('Map')
    assert app.focus == 'map'
    app.dispatch('c'), app.dispatch('R')
    assert app.layerbox.costmap and app.layerbox.referee
    app.show_tab('Bags')
    assert app.focus == 'list'
    assert isinstance(app.view()['bags'], list)


def test_bags_tab_with_a_session_renders_and_moves_the_cursor(tmp_path):
    from mhseals_testing.harness.bags import Bag
    app, _ = make_app(tmp_path)
    app.show_tab('Bags')
    app.bag_scan = (app.clock(), [Bag(id='boat_1', path=None)], (0, 0, 0))
    app.bag_recording_key = app.recording_paths()
    assert app.view()['position'] == '1/1'
    app.view()
    app.dispatch('j')
    assert app.cursors['list'] == 0


def test_bags_tab_marks_the_live_session_recording(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'data'))
    live = tmp_path / 'data' / 'sessions' / 'live_1'
    (live / 'bag').mkdir(parents=True)
    app, _ = make_app(tmp_path)
    app.paths = SessionPaths.from_output(live)
    app.processes.running = lambda name: name == 'bag'
    app.bag_scan = (-100.0, [], (0, 0, 0))
    sessions, _ = app.bags_view()
    assert [(bag.id, bag.recording) for bag in sessions] == [('live_1', True)]


def test_map_tab_floor_and_trail_restart_reach_the_map_state(tmp_path):
    class State:
        calls = []
        set_floor = lambda self, floor: self.calls.append(('floor', floor))
        restart_trail = lambda self: self.calls.append('restart')
    app, _ = make_app(tmp_path)
    app.ros.map = type('Map', (), {'state': State()})()
    app.show_tab('Map')
    app.dispatch('D'), app.dispatch('Y')
    assert State.calls[0][0] == 'floor' and State.calls[1] == 'restart'


def test_global_keys_reach_pins_odometry_and_panel_exclude(tmp_path):
    app, _ = make_app(tmp_path)
    app.ask_text = lambda *a: None
    app.ran = []
    app.run_procedure = app.ran.append
    app.focus = 'thrusters'
    app.dispatch('p')                       # Perception stays on p
    assert [x.key for x in app.ran] == ['p'] and app.notice != 'Pins unchanged'
    app.ran.clear()
    app.dispatch('M')                       # keymap pins: M
    assert app.notice == 'Pins unchanged' and not app.ran
    opened = []
    app.open = opened.append
    app.focus = 'motion'
    app.dispatch('o')
    assert opened == ['odometry']
    app.focus = 'status'
    app.dispatch('o')
    assert opened == ['odometry'] * 2
    app.review = True                       # review: Status no longer blocks o
    app.dispatch('o')
    assert opened == ['odometry'] * 3
    app.review = False
    app.monitor.record_result(dict(procedure='axis', verdict='OK'))
    app.focus = 'activity'
    with app.attached():
        press(app, 'enter')  # first reason
        app.dispatch('X')
    assert app.monitor.snapshot()['results'][0]['excluded']['reason'] == 'operator error'


def test_idle_refresh_skips_identical_frames(tmp_path):
    """rv1-48: an unchanged idle frame is not rewritten; a key-driven refresh always is."""
    app, _ = make_app(tmp_path)
    app.console.size = (120, 40)
    app.live = Live(None)
    for _ in range(3):
        app.last_frame = 0.0
        app.refresh(force=False)
    assert app.live.frames == 1
    app.refresh()
    assert app.live.frames == 2
    app.notice = 'changed'
    app.last_frame = 0.0
    app.refresh(force=False)
    assert app.live.frames == 3
    assert isinstance(app.live.last, Segments)  # the rendered segments, not laid out again


def test_rec_hours_are_coarse():
    from mhseals_testing.harness import recwatch as r
    mk = lambda m: r.RecSample('lean', 2e5, 50.0, m, False)
    assert r.strip_text(mk(1196 * 60))[1] == r.strip_text(mk(1204 * 60))[1]
    assert r.strip_text(mk(5 * 60))[1].endswith('5 h')


def test_session_runner_sees_the_mux_override_and_a_moving_boat(tmp_path):
    """rv2-30: the TUI Runner has an override source (mux) and a still gate (conditions)."""
    from mhseals_testing.ros.authority import Arbiter
    app, _ = make_app(tmp_path)
    clock = [100.0]
    app.arbiter = Arbiter(lambda record: None, lambda phase, **fields: None, tui_owner=True,
                          clock=lambda: clock[0])
    app.arbiter.on_mux({'input': 'auto'})
    app.arming = Arming()
    app.arming.armed = True
    with app.observe('x') as runner:
        assert runner.override() == '' and runner.gate() == []
        app.arbiter.on_mux({'input': 'rc'})
        assert runner.override() == 'rc'
        clock[0] += 5.0                      # a stale mux is unknown, not auto
        assert runner.override() == 'unknown'
        app.monitor.observe('odometry', {'vx': .5, 'vy': 0, 'wz': 0})
        assert runner.gate() and 'moving' in runner.gate()[0]


def test_override_is_empty_without_a_mux_state_when_unmuxed_or_disarmed(tmp_path, monkeypatch):
    """rv2-30 round 2: an arbiter with no mux state is not an override unless armed and muxed."""
    from mhseals_testing.ros.authority import Arbiter
    app, _ = make_app(tmp_path)
    app.arbiter = Arbiter(lambda record: None, lambda phase, **fields: None, tui_owner=True)
    app.arming = Arming()
    with app.observe('x') as runner:
        assert runner.override() == ''           # disarmed: the mux is not running yet
        app.arming.armed = True
        monkeypatch.setattr('mhseals_testing.ui.app.muxed', lambda profile: False)
        assert runner.override() == ''           # unmuxed profile: no mux at all
        monkeypatch.undo()
        assert runner.override() == 'unknown'    # armed and muxed: a missing state is unknown


def test_session_pause_restores_c_and_resume_restages_identity(tmp_path):
    app, _ = make_app(tmp_path)
    staging = app.staging()
    staging.raw_correction()
    with app.observe('x') as runner:
        runner.on_pause()
        assert staging.pending() is None
        runner.on_resume()
        assert staging.pending() is not None
        runner.on_pause(); runner.on_pause(); runner.on_resume()
        assert staging.pending() is not None


def test_segment_and_session_close_build_the_index(tmp_path):
    from mhseals_testing.harness import session
    root = tmp_path / 'root'
    made = session.create('boat', host='h', directory=root / 'sessions' / 's1', kind='boat')
    app, _ = make_app(tmp_path, ['--output', str(made.directory)])
    assert app.paths.session == made.file and not (root / 'INDEX.json').exists()
    app.set_recording('off')
    rows = json.loads((root / 'INDEX.json').read_text())['sessions']
    assert [r['id'] for r in rows] == ['s1'] and 'open' in rows[0]['flags']
    app.close_index()
    assert json.loads(made.file.read_text())['ended']
    assert 'open' not in json.loads((root / 'INDEX.json').read_text())['sessions'][0]['flags']
    assert (root / 'README.md').exists()


@pytest.mark.parametrize('popup', [False, True])
@pytest.mark.parametrize('fails', [False, True])
def test_operator_space_never_engages_watchers_auto_topic(tmp_path, popup, fails):
    app, boat = make_app(tmp_path)
    app.watching = 123
    app.arming.armed = True
    app.command = (0.3, 0, 0)
    calls = []

    def hold(cause):
        calls.append(cause)
        if fails:
            raise RuntimeError('hold failed')

    boat.hold = hold
    boat.engage = lambda: pytest.fail('watcher engaged auto publisher')
    with app.attached():
        press(app, 'space')
        try:
            if popup:
                app.show('Watch', message='watching')
            else:
                app.dispatch('space')
        except RuntimeError:
            assert fails
    assert calls == ['keys']
    assert app.command == (0, 0, 0) and not app.armed
    assert boat.sent and all(command == 'neutral' for command in boat.sent)


def release_app(tmp_path, result=True):
    from mhseals_testing.ros.authority import Arbiter
    app, boat = make_app(tmp_path)
    calls = []
    boat.release_hold = lambda: calls.append('release') or result
    app.arbiter = Arbiter(lambda record: None, lambda *a, **kw: None,
                          clock=app.clock, tui_owner=True)
    app.arbiter.on_mux({'input': 'hold', 'rc': {'mode': 'GUIDED'}})
    return app, boat, calls


@pytest.mark.parametrize('keys', [('enter',), ('escape',), ('q',)])
def test_release_defaults_to_cancel(tmp_path, keys):
    app, boat, calls = release_app(tmp_path)
    with app.attached():
        press(app, *keys)
        app.dispatch('U')
    assert calls == [] and boat.sent == []


@pytest.mark.parametrize('mode', ['HOLD', 'MANUAL', 'stale', 'unknown', 'missing'])
def test_release_refuses_rc_and_untrusted_state(tmp_path, mode):
    app, boat, calls = release_app(tmp_path)
    if mode == 'stale':
        app.arbiter.mux_at = -10
    elif mode == 'unknown':
        app.arbiter.mux['input'] = 'unknown'
    elif mode == 'missing':
        app.arbiter.mux = None
    else:
        app.arbiter.mux['rc']['mode'] = mode
    app.dispatch('U')
    assert app.notice.startswith('Release refused:')
    assert calls == [] and boat.sent == []


@pytest.mark.parametrize('result', [False, True])
def test_release_request_is_single_and_never_claims_acknowledgement(tmp_path, result):
    app, boat, calls = release_app(tmp_path, result)
    with app.attached():
        press(app, 'U', 'up', 'enter')
        app.dispatch('U')
    assert calls == ['release'] and boat.sent == []
    assert app.notice == ('Release requested' if result else 'Release hold is not available here')
    assert app.arbiter.mux['input'] == 'hold'  # no ack has arrived


def test_nested_release_restores_authority_and_e_revokes_inside_confirm(tmp_path):
    app, boat, calls = release_app(tmp_path)
    claim = app.arbiter.handle({'op': 'claim', 'request': 'r', 'who': 'tune'})['claim']
    seen, revoked = [], []
    app.arbiter.neutral = lambda: None
    app.arbiter.hold = lambda cause, by: revoked.append(cause)
    outer = dict(kind='message', title='Authority')

    def handle(key):
        seen.append((key, app.modal is outer))
        return True

    with app.attached():
        press(app, 'U', 'E', 'enter', 'escape')
        app.popup(outer, handle)
    assert seen == [('escape', True)] and calls == []
    assert revoked == ['revoke']
    assert app.arbiter.table.get(claim['id'])['state'] == 'revoked'
    assert app.modal is None and boat.sent == []


def test_release_confirmation_names_procedure_and_rechecks_rc(tmp_path):
    app, boat, calls = release_app(tmp_path)
    app.running = 'characterize'

    def choose(title, options, **kwargs):
        assert kwargs['message'] == 'characterize resumes' and kwargs['selected'] == 1
        app.arbiter.on_mux({'input': 'rc', 'rc': {'mode': 'MANUAL'}})
        return 'Release'

    app.choose = choose
    app.dispatch('U')
    assert calls == [] and 'MANUAL' in app.notice and boat.sent == []


def test_authority_popup_keeps_its_selected_claim_e_binding(tmp_path):
    app, _, _ = release_app(tmp_path)
    app.revoke_agent = lambda: pytest.fail('global revoke stole selected claim')
    seen = []
    with app.attached():
        press(app, 'E')
        app.popup(dict(kind='message', title='Authority · 12'),
                  lambda key: seen.append(key) or True)
    assert seen == ['E']


def test_release_names_the_claimant(tmp_path):
    app, _, calls = release_app(tmp_path)
    app.arbiter.handle({'op': 'claim', 'request': 'r', 'who': 'tune_mppi'})

    def choose(title, options, **kwargs):
        assert kwargs['message'] == 'tune_mppi resumes'
        return 'Cancel'

    app.choose = choose
    app.dispatch('U')
    assert calls == []


@pytest.mark.parametrize('popup', [False, True])
def test_watch_space_stays_held_after_rc_returns_guided(tmp_path, popup):
    from mhseals_hardware.command_arbitration import Arbitration
    app, boat = make_app(tmp_path)
    app.watching = 42
    mux = Arbitration()
    manual = []

    def hold(cause):
        now = app.clock()
        manual.append((0, 0, 0))
        mux.update_manual(manual[-1], now)
        mux.request('hold', cause, 'tui', now)

    boat.hold = hold
    boat.engage = lambda: pytest.fail('watcher acquired automatic motion')
    with app.attached():
        if popup:
            press(app, 'space')
            app.show('Watch', message='live')
        else:
            app.dispatch('space')
    now = app.clock() + 1
    for mode in (1550, 1900):
        mux.update_rc([1500] * 5 + [1900, 1500, mode], now)
    mux.update_auto((0.8, 0, 0), now)
    source, command = mux.select(now)[:2]
    assert source == 'hold' and command == (0, 0, 0)
    assert 'keys' in mux.holds and manual == [(0, 0, 0)]


@pytest.mark.parametrize('fails', [False, True])
def test_actual_app_manual_space_ends_at_zero_even_on_hold_error(tmp_path, fails):
    app, boat = make_app(tmp_path, keys=['m', 'q'])
    app.arming.armed = True
    events = Events(app.clock, ['press:w', None, 'space', 'x'])
    app.open_events = lambda: events
    stopped = []

    def hold(cause):
        stopped.append(len(boat.sent))
        assert cause == 'keys'
        if fails:
            raise RuntimeError('hold failed')

    boat.hold = hold
    press(app, 'y')
    app.run()
    assert len(stopped) == 1 and not app.manual
    assert all(command == 'neutral' or command == (0, 0, 0)
               for command in boat.sent[stopped[0]:])
    assert app.command == (0, 0, 0)
    if fails:
        assert not app.armed


def test_popup_space_clears_raw_pulses_before_background_tick(tmp_path):
    app, _ = make_app(tmp_path)
    app.test_pulses = [1600] * 4
    app.arming.armed = True
    with app.attached():
        press(app, 'space')
        app.show('Pulses', message='bench')
    assert app.pulse_log and all(pulses == [1500] * 4 for pulses in app.pulse_log)
    assert app.test_pulses is None and not app.armed



def test_held_unclaimed_auto_is_named_truthfully(tmp_path):
    app, boat, calls = release_app(tmp_path)
    app.arbiter.mux['auto'] = {'active': True, 'node': 'external_publisher'}

    def choose(title, options, **kwargs):
        assert kwargs['message'] == 'unclaimed automation resumes'
        assert 'idle' not in kwargs['message']
        return 'Cancel'

    app.choose = choose
    app.dispatch('U')
    assert calls == [] and boat.sent == []


@pytest.mark.parametrize('change', ['replace', 'revoke', 'new', 'procedure'])
def test_release_refuses_changed_resumer_after_confirmation(tmp_path, change):
    app, boat, calls = release_app(tmp_path)
    if change in ('replace', 'revoke'):
        claim = app.arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'agent_one'})['claim']
    elif change == 'procedure':
        app.running, app.run_id = 'characterize', 'run1'
    app.arbiter.hold = lambda *a: None

    def choose(title, options, **kwargs):
        if change in ('replace', 'revoke'):
            assert kwargs['message'] == 'agent_one resumes'
            app.arbiter.revoke(claim['id'])
        if change in ('replace', 'new'):
            app.arbiter.handle({'op': 'claim', 'request': 'r2', 'who': 'agent_two'})
        if change == 'procedure':
            app.run_id = 'run2'
        return 'Release'

    app.choose = choose
    app.dispatch('U')
    assert app.notice == 'Release refused: resumer changed; confirm again'
    assert calls == [] and boat.sent == []



@pytest.mark.parametrize('context', ['config', 'choose'])
def test_ordinary_popups_ignore_release_key(tmp_path, context):
    app, boat, calls = release_app(tmp_path)
    with app.attached():
        press(app, 'U', 'up', 'enter', 'escape')
        if context == 'config':
            app.open('config')
        else:
            assert app.choose('Ordinary choice', ['Continue', 'Cancel'], selected=1) == 'Continue'
    assert calls == [] and boat.sent == []


def test_authority_popup_release_confirms_and_restores_parent(tmp_path):
    app, boat, calls = release_app(tmp_path)
    outer = dict(kind='message', title='Authority · 7')
    restored = []

    def handle(key):
        restored.append((key, app.modal is outer))
        return True

    with app.attached():
        press(app, 'U', 'up', 'enter', 'escape')
        app.popup(outer, handle)
    assert calls == ['release'] and boat.sent == []
    assert restored == [('escape', True)] and app.modal is None


def authority_popup_app(tmp_path):
    app, boat, releases = release_app(tmp_path)
    app.arbiter.neutral = boat.neutral
    app.arbiter.hold = lambda cause, by: boat.hold(cause)
    app.arbiter.on_mux({'input': 'hold', 'rc': {'mode': 'GUIDED', 'valid': True,
                                               'rate_hz': 25, 'interval_p95_s': 0.04},
                        'holds': [{'cause': 'keys', 'by': 'tui', 'since_ns': 0}]})
    app.arbiter.tick(force=True)
    app.focus = 'status'
    from mhseals_testing.ui.panels.status import status_rows
    rows = status_rows(app.snapshot(), app.profile.sensors)
    app.cursors['status'] = next(i for i, row in enumerate(rows) if row['name'] == 'Driving')
    return app, boat, releases


def test_actual_driving_enter_opens_live_authority_popup(tmp_path):
    app, boat, releases = authority_popup_app(tmp_path)
    frames = []
    refresh = app.refresh

    def capture(*args, **kwargs):
        if app.modal:
            frames.append(dict(app.modal))
        return refresh(*args, **kwargs)

    app.refresh = capture
    with app.attached():
        press(app, 'escape')
        app.dispatch('enter')
    box = next(frame for frame in frames if frame['title'].startswith('Authority'))
    rows = dict(box['rows'])
    assert rows['Physical'] == 'hold · hold keys'
    assert rows['RC'] == 'GUIDED · 25 Hz · p95 40 ms'
    assert releases == [] and boat.sent == [] and app.modal is None


def test_actual_authority_e_uses_selected_history_then_active_claim(tmp_path):
    app, boat, releases = authority_popup_app(tmp_path)
    old = app.arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'old'})['claim']
    app.arbiter.table.revoke(old['id'], app.clock())
    active = app.arbiter.handle({'op': 'claim', 'request': 'r2', 'who': 'current'})['claim']
    revoked = []
    revoke = app.arbiter.revoke
    app.arbiter.revoke = lambda id: revoked.append(id) or revoke(id)
    with app.attached():
        press(app, 'E', 'down', 'E', 'escape')
        app.dispatch('enter')
    assert revoked == [old['id'], active['id']]
    assert app.arbiter.table.get(active['id'])['state'] == 'revoked'
    assert releases == [] and all(command == 'neutral' for command in boat.sent)


@pytest.mark.parametrize('confirm', [False, True])
def test_actual_authority_u_uses_confirmed_release(tmp_path, confirm):
    app, boat, releases = authority_popup_app(tmp_path)
    with app.attached():
        press(app, 'U', *(['up', 'enter'] if confirm else ['enter']), 'escape')
        app.dispatch('enter')
    assert releases == (['release'] if confirm else []) and boat.sent == []
    assert app.modal is None
    if confirm:
        assert app.notice == 'Release requested'


def test_actual_authority_space_is_sticky_and_disarms(tmp_path):
    app, boat, releases = authority_popup_app(tmp_path)
    app.arming.armed = True
    app.command = (0.2, 0, 0)
    with app.attached():
        press(app, 'space', 'escape')
        app.dispatch('enter')
    assert boat.holds == ['keys'] and not app.armed and app.command == (0, 0, 0)
    assert releases == [] and boat.sent and all(command == 'neutral' for command in boat.sent)


def test_actual_authority_lowercase_e_u_do_nothing(tmp_path):
    app, boat, releases = authority_popup_app(tmp_path)
    claim = app.arbiter.handle({'op': 'claim', 'request': 'r', 'who': 'agent'})['claim']
    with app.attached():
        press(app, 'e', 'u', 'escape')
        app.dispatch('enter')
    assert app.arbiter.table.get(claim['id'])['state'] == 'active'
    assert releases == [] and boat.sent == []


@pytest.mark.parametrize('state', ['stale', 'unknown', 'missing'])
def test_actual_authority_untrusted_records_render_safely(tmp_path, state):
    app, boat, releases = authority_popup_app(tmp_path)
    if state == 'stale':
        app.arbiter.record['t_ns'] = -10_000_000_000
    elif state == 'unknown':
        app.arbiter.record = dict(t_ns=0, owner='unknown', input=None)
    else:
        app.arbiter.record = None
    seen = []
    refresh = app.refresh

    def capture(*args, **kwargs):
        if app.modal:
            seen.extend(app.modal['rows'])
        return refresh(*args, **kwargs)

    app.refresh = capture
    with app.attached():
        press(app, 'escape')
        app.dispatch('enter')
    text = ' '.join(str(value) for _, value in seen)
    assert ('UNKNOWN' in text if state != 'missing' else 'no authority record yet' in text)
    if state == 'unknown':
        assert 'hold state unknown' in text
    assert releases == [] and boat.sent == []


def test_failed_hardware_snapshot_retains_memory_and_draft(tmp_path, monkeypatch):
    import mhseals_testing.ui.app as module
    app, _ = make_app(tmp_path)
    app.arming = Arming()
    app.arming.armed = True
    app.profile = type('Profile', (), {'outputs': 'memory'})()
    old = app.args.hardware_config
    app.arming.hardware_config = old
    app.draft = draft = {'channel_map': [2, 1, 3, 4]}
    def fail(*args):
        raise OSError('snapshot write failed')
    monkeypatch.setattr(module, 'save_config', fail)
    assert not app.save_hardware(channel_map=[2, 1, 3, 4])
    assert app.args.hardware_config is old and app.arming.hardware_config is old
    assert app.draft is draft
    assert 'snapshot write failed' in app.notice
    assert 'reloaded' not in app.notice
    assert not app.watcher.reload('hardware').report
