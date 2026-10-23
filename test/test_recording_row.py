"""The System group's Recording row: the app records through harness.bags.Recorder (UIW13)."""

import io

from rich.console import Console

from mhseals_testing.harness.bags import Recorder
from mhseals_testing.harness.recwatch import RecSample, strip_text
from mhseals_testing.ui import layout
from mhseals_testing.ui.panels.status import status_panel, status_rows

from test_app import make_app
from test_chrome import check
from test_dashboard import monitor, view
from mhseals_testing.profiles import load_profile


class FakeProcesses:
    """Records start/stop calls for the 'bag' child; running follows them."""

    def __init__(self):
        self.calls, self.argv = [], {}

    def start(self, name, argv):
        self.calls.append(('start', name))
        self.argv[name] = argv

    def stop(self, name, timeout=10):
        self.calls.append(('stop', name))
        self.argv.pop(name, None)

    def running(self, name):
        return name in self.argv

    def exited(self):
        return []

    def records(self):
        return {}


def rig(tmp_path):
    app, _ = make_app(tmp_path)
    app.processes = FakeProcesses()
    app.recorder = Recorder(app.processes, tmp_path / 'bag', app.profile.name)
    app.ros.events = events = []
    app.ros.emit = lambda phase, **d: events.append((phase, d))
    rows = status_rows(app.snapshot(), app.profile.sensors)
    app.cursors['status'] = next(i for i, r in enumerate(rows) if r['name'] == 'Recording')
    return app, events


def test_lean_start_uses_topic_profile_and_arrows_step_to_full_then_off(tmp_path):
    app, events = rig(tmp_path)
    app.set_recording('lean')
    argv = app.processes.argv['bag']
    assert '--topics' in argv and '-e' in argv and '--all-topics' not in argv
    assert events == [('rec_start', {'segment': '00', 'mode': 'lean'})]
    app.dispatch('right')
    assert app.recorder.mode == 'full' and '--all-topics' in app.processes.argv['bag']
    app.dispatch('right')
    assert app.recorder.mode == 'off' and not app.processes.running('bag')
    app.dispatch('right')                                  # no wrap past off
    assert app.recorder.mode == 'off'
    assert [e[0] for e in events] == ['rec_start', 'rec_stop', 'rec_start', 'rec_stop']
    assert events[3][1] == {'segment': '01', 'reason': 'operator'}


def test_off_to_lean_starts_the_next_segment_and_the_strip_follows(tmp_path):
    app, events = rig(tmp_path)
    app.set_recording('lean')
    app.dispatch('left')                                   # already first: stays
    app.set_recording('off')
    app.dispatch('left')                                   # off -> full
    app.dispatch('left')                                   # full -> lean
    assert app.recorder.mode == 'lean' and events[-1] == ('rec_start', {'segment': '02', 'mode': 'lean'})
    app.set_recording('off')
    assert app.ros.recording is False                      # events while off stamp rec=false
    app.set_recording('lean')
    assert app.ros.recording is True
    assert events[-1][1]['segment'] == '03'
    assert strip_text(app.recwatch.sample(tmp_path, True, 'full'))[1].startswith('rec full')
    assert strip_text(app.recwatch.sample(tmp_path, False, 'off'))[1] == 'rec off'


def test_quit_stops_the_recorder_and_journals_it(tmp_path):
    app, events = rig(tmp_path)
    app.set_recording('lean')
    assert app.close_recording().startswith('flushed')
    assert not app.processes.running('bag') and events[-1][0] == 'rec_stop'
    assert events[-1][1]['reason'] == 'quit'


def test_system_group_with_the_recording_row_golden():
    profile = load_profile('boat')
    snapshot = {**monitor().snapshot(profile.required_tf), 'rec_mode': 'lean'}
    rows = status_rows(snapshot, profile.sensors)
    assert next(r for r in rows if r['name'] == 'Recording')['group'] == 'System'
    console = Console(file=io.StringIO(), width=layout.status_width(120), height=30, color_system=None)
    console.print(status_panel(snapshot, view(profile), 30))
    check('status/recording_120.txt', console.file.getvalue().rstrip('\n') + '\n')
