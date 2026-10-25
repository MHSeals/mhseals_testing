"""The device parts' Status rows, Enter's confirmed start or stop, and the Quit question."""

from types import SimpleNamespace

from mhseals_hardware.menu import Cancelled

from mhseals_testing.harness.components import Stack
from mhseals_testing.ui import parts
from test_components import FakeProcesses, FakeUnits, Profile, items


class App:
    def __init__(self, answer='Start', armed=False):
        units = FakeUnits()
        self.stack = Stack(items(Profile), FakeProcesses(), lambda: 0.0, units)
        self.ros = SimpleNamespace(nodes=lambda: [], emit=lambda *a, **k: self.events.append((a, k)))
        self.answer, self.armed, self.asked, self.events, self.notice = answer, armed, [], [], ''

    def choose(self, title, options, shortcuts=None, rows=None, danger=False, **_):
        self.asked.append((title, options, dict(rows), danger))
        if self.answer is None:
            raise Cancelled()
        return self.answer


def test_rows_show_state_and_host():
    app = App()
    app.stack.units.errors['camera'] = 'ssh: timeout'
    rows = {row['key']: row for row in parts.feed(app)}
    assert rows['odometry']['value'] == 'off' and rows['odometry']['state'] == 'OFF'
    assert rows['camera']['value'] == 'unreachable on blastoise-jetson'
    assert rows['camera']['state'] == 'WARN' and 'ssh: timeout' in rows['camera']['detail']


def test_enter_confirms_with_the_host_then_starts_a_remote_part():
    app = App()
    app.stack.units.records['camera'] = {'state': 'off'}
    parts.toggle(app, 'camera')
    title, options, rows, danger = app.asked[0]
    assert title == 'Camera on blastoise-jetson: Start?' and options == ['Start', 'Cancel']
    assert rows['Computer'] == 'blastoise-jetson' and not danger
    assert app.stack.units.calls == [('start', 'camera', False)]


def test_a_commander_start_is_a_danger_confirm_and_cancel_does_nothing():
    app = App(answer=None)
    app.stack.processes.live.add('odometry')
    parts.toggle(app, 'navigation')
    assert app.asked[0][3] and 'Motion' in app.asked[0][2]
    assert app.stack.processes.started == []


def test_stop_while_armed_warns_but_stops():
    app = App(answer='Stop', armed=True)
    app.stack.units.records['camera'] = {'state': 'running'}
    parts.toggle(app, 'camera')
    assert app.asked[0][0] == 'Camera on blastoise-jetson: Stop?'
    assert not app.asked[0][0].startswith(('Stop', 'Authority'))    # U never releases a hold here
    assert app.stack.units.calls == [('stop', 'camera')]


def test_a_part_not_read_yet_offers_stop():
    app = App(answer='Stop')
    parts.toggle(app, 'camera')
    assert app.asked[0][1] == ['Stop', 'Cancel'] and app.stack.units.calls == [('stop', 'camera')]


def test_quit_names_the_units_held_on_each_host():
    app = App()
    app.stack.units.held_by_host = lambda: {'blastoise-jetson': ['Camera', 'Objects']}
    assert parts.quit_rows(app) == [('On blastoise-jetson', 'Camera, Objects: Quit stops them')]
    assert parts.quit_rows(SimpleNamespace(stack=None)) == []
