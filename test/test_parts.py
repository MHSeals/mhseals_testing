"""The device parts' Status rows, Enter's confirmed start or stop, and the Quit question."""

from types import SimpleNamespace

from mhseals_testing.model import Cancelled

from mhseals_testing.harness.components import Stack
from mhseals_testing.ui import parts, popups
from test_components import FakeProcesses, FakeUnits, Profile, items


class App:
    def say(self, text):   # App.say, the notice's one writer
        self.notice = text

    def __init__(self, answer='Start', armed=False):
        units = FakeUnits()
        self.stack = Stack(items(Profile), FakeProcesses(), lambda: 0.0, units)
        self.ros = SimpleNamespace(nodes=lambda: [], emit=lambda *a, **k: self.events.append((a, k)))
        self.answer, self.armed, self.asked, self.events, self.notice = answer, armed, [], [], ''

    def choose(self, title, options, shortcuts=None, rows=None, danger=False, default=0, **_):
        # The real modal refuses a danger default, as the popup would.
        popups.choice_modal(title, options, shortcuts, rows=rows, danger=danger, default=default)
        self.asked.append((title, options, dict(rows), danger))
        self.default = options[default]
        if self.answer is None:
            raise Cancelled()
        return self.answer


def test_rows_show_state_and_host_and_an_issue_only_as_a_glyph():
    from mhseals_testing.ui import theme
    from mhseals_testing.ui.feeds.issues import Issues
    app = App()
    app.stack.units.records['camera'] = {'state': 'running', 'how': 'unit', 'running': True}
    app.stack.units.errors['camera'] = 'ssh: timeout'
    rows = {row['key']: row for row in parts.feed(app)}
    assert rows['odometry']['value'] == '' and rows['odometry']['state'] == 'OFF'   # the glyph says off
    assert rows['camera']['value'] == '' and rows['camera']['state'] == 'OK'     # never 'unreachable'
    assert rows['camera']['detail'].startswith('running on blastoise-jetson')    # no host in the column
    assert rows['camera']['state'] == 'OK'
    app.issues = Issues()
    app.issues.shown['device:blastoise-jetson'] = 'blastoise-jetson does not answer (ssh: timeout)'
    rows = {row['key']: row for row in parts.feed(app)}
    assert rows['camera']['value'] == theme.PALETTE.glyph('issue')
    assert rows['camera']['state'] == 'OK' and 'does not answer' in rows['camera']['detail']
    assert rows['odometry']['value'] == ''


def test_a_stopped_heartbeat_says_the_link_watchdog_holds():
    from test_components import ME, PARTS, part
    from mhseals_testing.harness.components import components

    class Operator(Profile):
        components = dict(Profile.components, operator_heartbeat={})
    processes = FakeProcesses()
    stack = Stack(components(Operator, None, (ME, dict(
        PARTS, operator_heartbeat=part('operator_heartbeat', None, nodes=['operator_heartbeat'])))),
        processes, lambda: 0.0, FakeUnits())
    row = {row['key']: row for row in parts.rows(stack, [])}['operator_heartbeat']
    assert 'link watchdog holds autonomy until started' in row['detail']
    processes.live.add('operator_heartbeat')
    row = {row['key']: row for row in parts.rows(stack, ['/operator_heartbeat'])}['operator_heartbeat']
    assert 'link watchdog' not in row['detail']


def test_enter_confirms_with_the_host_then_starts_a_remote_part():
    app = App()
    app.stack.units.records['camera'] = {'state': 'off'}
    parts.toggle(app, 'camera')
    title, options, rows, danger = app.asked[0]
    assert title == 'Camera on blastoise-jetson: Start?' and options == ['Start', 'Cancel']
    assert rows['Computer'] == 'blastoise-jetson' and not danger and app.default == 'Cancel'
    assert app.stack.units.calls == [('start', 'camera', False)]


def test_a_commander_start_is_a_danger_confirm_and_cancel_does_nothing():
    app = App(answer=None)
    app.stack.processes.live.add('odometry')
    parts.toggle(app, 'navigation')
    assert app.asked[0][3] and 'Motion' in app.asked[0][2] and app.default == 'Cancel'
    assert app.stack.processes.started == []


def test_stop_while_armed_warns_but_stops():
    app = App(answer='Stop', armed=True)
    app.stack.units.records['camera'] = {'state': 'running'}
    parts.toggle(app, 'camera')
    assert app.asked[0][0] == 'Camera on blastoise-jetson: Stop?'
    assert app.stack.units.calls == [('stop', 'camera')]


def test_ownership_is_only_a_popup_detail():
    app = App(answer='Stop')
    app.stack.units.records['camera'] = {'state': 'off', 'running': True, 'pid': 9, 'how': 'launch'}
    row = {row['key']: row for row in parts.feed(app)}['camera']
    assert row['state'] == 'OK' and row['detail'].startswith('running on blastoise-jetson')
    assert 'external' not in row['detail']
    parts.toggle(app, 'camera')
    assert app.asked[0][2]['Owner'] == 'pid 9 on blastoise-jetson, not started by a TUI'
    assert 'stop it where it was started' in app.notice and app.stack.units.calls == []


def test_a_part_not_read_yet_offers_stop():
    app = App(answer='Stop')
    parts.toggle(app, 'camera')
    assert app.asked[0][1] == ['Stop', 'Cancel'] and app.stack.units.calls == [('stop', 'camera')]


def test_quit_names_the_units_held_on_each_host():
    app = App()
    held = {False: {'blastoise-jetson': ['Camera', 'Objects']}, True: {'vlp16': ['LiDAR']}}
    app.stack.units.held_by_host = lambda adopted=None: held[adopted]
    assert parts.quit_rows(app) == [('On blastoise-jetson', 'Camera, Objects: Quit stops them'),
                                    ('On vlp16', 'LiDAR: adopted, Quit keeps them running')]
    assert parts.quit_rows(SimpleNamespace(stack=None)) == []


def test_quit_keeps_adopted_units_unless_the_operator_chooses_to_stop_them():
    from mhseals_testing.ui.app import App as Tui
    held = {False: {}, True: {'blastoise-jetson': ['Camera']}}
    units = SimpleNamespace(held_by_host=lambda adopted=None: held[adopted],
                            close=lambda stop, adopted: closed.append((stop, adopted)))
    closed, asked = [], []

    def tui(answer):
        def choose(title, options, shortcuts=None, rows=None, default=0, **_):
            asked.append((options, dict(rows), options[default], options[shortcuts['y']]))
            return answer
        return SimpleNamespace(armed=False, paths=SimpleNamespace(bag='b', report='r'),
                               services=SimpleNamespace(manual=SimpleNamespace(draft=None)),
                               watching=None, snapshot=lambda: {'results': []}, quit=False,
                               choose=choose, stack=SimpleNamespace(units=units),
                               keep_units=False, stop_adopted=False)
    plain = tui('Quit')
    Tui.confirm_quit(plain)
    options, rows, default, yes = asked[0]
    assert options == ['Quit', 'Quit, also stop adopted parts', 'Cancel'] and yes == 'Quit'
    assert default != 'Quit, also stop adopted parts'
    assert rows['On blastoise-jetson'] == 'Camera: adopted, Quit keeps them running'
    Tui.close_units(plain)
    explicit = tui('Quit, also stop adopted parts')
    Tui.confirm_quit(explicit)
    Tui.close_units(explicit)
    assert closed == [(True, False), (True, True)] and plain.quit and explicit.quit


def test_u_releases_only_from_a_release_modal_whatever_the_title():
    released = []
    keys = iter(['U', 'x'])
    app = SimpleNamespace(modal=None, refresh=lambda: None, tick=lambda: None,
                          keys=SimpleNamespace(read=lambda timeout: next(keys)),
                          release_hold=lambda: released.append(1))
    popups.popup(app, dict(kind='choice', title='Stop part?'), lambda key: key)
    assert released == []
    keys = iter(['U', 'x'])
    popups.popup(app, dict(kind='message', title='Hold', release=True), lambda key: key)
    assert released == [1]


def watched_app(answer='Start'):
    from test_components import ME, WATCHED_PARTS, Watched
    from mhseals_testing.harness.components import components
    from test_components import FakeProcesses as Processes
    app = App(answer=answer)
    app.stack = Stack(components(Watched, None, (ME, WATCHED_PARTS)), Processes(), lambda: 0.0)
    return app


def test_enter_on_the_watchdogs_starts_their_odom_stack_after_a_confirm():
    app = watched_app()
    parts.toggle(app, 'watchdogs')
    title, options, rows, danger = app.asked[0]
    assert title == 'Safety watchdogs start with Odometry: start Odometry?' and options == ['Start', 'Cancel']
    assert app.default == 'Cancel' and not danger                    # Enter alone never starts it
    assert [name for name, _ in app.stack.processes.started] == ['odometry']   # the carrier, never a second copy
    assert app.notice == 'Starting Odometry on this computer'


def test_cancel_on_the_watchdogs_starts_nothing():
    app = watched_app(answer='Cancel')
    parts.toggle(app, 'watchdogs')
    assert app.stack.processes.started == []


def test_odom_up_without_the_watchdogs_says_so_with_the_fix():
    app = watched_app(answer='Close')
    app.stack.start('odometry', set())                               # running, but no watchdog nodes
    started = list(app.stack.processes.started)
    parts.toggle(app, 'watchdogs')
    title, options, rows, _ = app.asked[0]
    assert title == 'Safety watchdogs: not running' and options == ['Close']
    assert 'started outside the TUI or without them' in rows['Problem']
    assert rows['Fix'].startswith('restart Odometry from the TUI')
    assert app.stack.processes.started == started                    # nothing more started


def stacked_app(answer='Start', clock=None):
    from test_components import JETSON, ME, WATCHED_PARTS, Profile, part
    from mhseals_testing.harness.components import components

    class Stacked(Profile):
        components = {'odometry': {'title': 'Odom stack', 'stack': True},
                      'ekf': {'title': 'EKFs', 'needs': ['odometry'], 'stack': True},
                      'navigation': {'needs': ['odometry', 'ekf'], 'limits': True},
                      'camera': {'stack': True},
                      'objects': {'needs': ['camera']},
                      'watchdogs': {'title': 'Safety watchdogs'}}
    placed = dict(WATCHED_PARTS, ekf=part('ekf', 'ekf.launch.py', nodes=['ekf_global']))
    app = App(answer=answer)
    clock = clock if clock is not None else [0.0]
    app.stack = Stack(components(Stacked, None, (ME, placed)), FakeProcesses(), lambda: clock[0],
                      app.stack.units)
    app.stack.units.records['camera'] = {'state': 'off'}
    assert app.stack.items['camera'].host == JETSON
    return app


def test_the_stack_row_starts_exactly_its_parts_in_needs_order_and_never_a_watchdog_copy():
    app = stacked_app()
    row = parts.feed(app)[0]
    assert (row['source'], row['name'], row['value'], row['state']) == ('stack', 'Stack', '0/3 up', 'OFF')
    assert 'Odom stack, EKFs, Camera' in row['detail'] and 'Safety watchdogs with them' in row['detail']
    parts.toggle_stack(app)
    title, options, rows, danger = app.asked[0]
    assert options == ['Start', 'Cancel'] and app.default == 'Cancel' and not danger
    assert rows['Odom stack'] == 'off on this computer' and rows['Safety watchdogs'] == 'with Odom stack'
    assert [name for name, _ in app.stack.processes.started] == ['odometry']   # EKFs wait for it
    assert app.stack.state('ekf', set()) == ('starting', 'waits for Odom stack')
    assert app.stack.units.calls == [('start', 'camera', False)]
    assert app.notice == 'Stack: starting Odom stack, EKFs, Camera'
    assert [k['component'] for _, k in app.events] == ['odometry', 'ekf', 'camera']
    app.ros.nodes = lambda: ['/ekf_local']                          # the Odom stack reports running
    parts.feed(app)
    assert [name for name, _ in app.stack.processes.started] == ['odometry', 'ekf']   # no watchdogs, no Nav2
    parts.start_stack(app)                                          # all up: nothing twice
    assert len(app.stack.processes.started) == 2 and len(app.stack.units.calls) == 1
    assert app.notice == 'Stack: nothing to start'


def test_cancel_on_the_stack_row_starts_nothing():
    app = stacked_app(answer=None)
    parts.toggle_stack(app)
    assert app.stack.processes.started == [] and app.stack.units.calls == []


def test_a_stack_part_that_does_not_start_skips_the_ones_that_need_it():
    app = stacked_app()
    app.stack.processes.codes['odometry'] = 1                      # exited: a start still goes out
    app.stack.units.records['camera'] = {'state': 'unknown', 'probe_error': 'x'}
    app.stack.items['odometry'] = replace_enabled(app.stack.items['odometry'])
    started, failed = parts.start_stack(app)
    assert started == [] and set(failed) == {'odometry', 'ekf', 'camera'}
    assert failed['ekf'] == 'Odom stack did not start' and 'try again shortly' in failed['camera']
    assert app.stack.processes.started == [] and app.stack.units.calls == []


def replace_enabled(item):
    from dataclasses import replace
    return replace(item, carried_by='Somewhere')                    # its start refuses


def test_stop_stack_stops_the_up_parts_and_names_what_else_stops():
    app = stacked_app(answer='Stop')
    parts.start_stack(app)
    app.ros.nodes = lambda: ['/ekf_local']
    parts.feed(app)
    app.stack.processes.live.add('navigation')                     # ours, needs the Odom stack
    parts.toggle_stack(app)
    title, options, rows, _ = app.asked[0]
    assert options == ['Stop', 'Cancel'] and rows['Stop also stops'] == 'Navigation'
    assert app.stack.units.calls[-1] == ('stop', 'camera')
    assert app.stack.processes.stopped == ['navigation', 'ekf', 'odometry']
    assert app.notice.startswith('Stack: stopping Camera, Navigation, EKFs, Odom stack')


def test_the_stack_row_never_holds_a_part_that_can_command_motion():
    import pytest
    from test_components import ME, PARTS, Profile
    from mhseals_testing.harness.components import components

    class Moving(Profile):
        components = {'odometry': {}, 'navigation': {'needs': ['odometry'], 'stack': True}}
    with pytest.raises(ValueError, match='navigation.stack: navigation can command motion'):
        components(Moving, None, (ME, PARTS))


def test_the_boat_profile_stack_is_the_sensing_start_set():
    from pathlib import Path
    import yaml
    data = yaml.safe_load((Path(__file__).parents[1] / 'config/profiles/boat.yaml').read_text())
    assert [key for key, spec in data['components'].items() if (spec or {}).get('stack')] == \
        ['odometry', 'ekf', 'sensors', 'camera', 'objects']


def test_a_start_brings_up_its_needs_first_and_the_confirm_names_them():
    clock = [0.0]
    app = stacked_app(clock=clock)
    parts.toggle(app, 'navigation')
    title, options, rows, danger = app.asked[0]
    assert title == 'Navigation needs Odom stack, EKFs: start all 3?' and danger
    assert rows['Also starts'] == 'Odom stack, EKFs'
    assert [name for name, _ in app.stack.processes.started] == ['odometry']
    app.ros.nodes = lambda: ['/ekf_local']
    parts.feed(app)
    assert [name for name, _ in app.stack.processes.started] == ['odometry', 'ekf']
    app.ros.nodes = lambda: ['/ekf_local', '/ekf_global']
    parts.feed(app)
    assert [name for name, _ in app.stack.processes.started] == ['odometry', 'ekf', 'navigation']


def test_a_need_that_never_reports_running_gives_up_with_its_dependants():
    clock = [0.0]
    app = stacked_app(clock=clock)
    parts.start(app, 'objects')                                     # needs the Camera on the Jetson
    assert app.stack.units.calls == [('start', 'camera', False)] and 'objects' in app.stack.waiting
    app.stack.units.records['camera'] = {'state': 'unknown', 'probe_error': 'no ps'}
    clock[0] = 61.0                                                  # past NEED_WAIT_S
    parts.feed(app)
    assert app.notice == 'Objects not started: Camera not running within 60 s'
    assert app.stack.processes.started == [] and app.stack.waiting == {}


def test_a_need_that_exits_gives_up_its_waiting_dependants():
    app = stacked_app()
    parts.start(app, 'navigation')
    app.stack.processes.live.discard('odometry')
    app.stack.processes.codes['odometry'] = 1
    parts.feed(app)
    assert app.notice == 'EKFs, Navigation not started: Odom stack is exited'
    assert app.stack.waiting == {}


def test_stopping_a_part_never_stops_what_it_needs_and_calls_off_a_waiting_start():
    app = stacked_app()
    parts.start(app, 'ekf')                                         # brings the Odom stack; EKFs wait
    assert parts.stop(app, 'ekf') == ['ekf']
    assert app.stack.processes.stopped == [] and app.stack.waiting == {}
    assert app.stack.processes.running('odometry')
