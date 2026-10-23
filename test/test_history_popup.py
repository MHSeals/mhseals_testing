"""popup.history: goldens at 120 and 80 columns, and the revert flow through a fake backend (UIW15)."""

import contextlib
import datetime
import io
from pathlib import Path

from rich.console import Console
import pytest

from mhseals_testing.history import History, revert
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.popups import history as popup

GOLDEN = Path(__file__).parent / 'golden' / 'history'
NOW = datetime.datetime.fromisoformat('2026-10-23T16:00:00+00:00')
PINS = {n: (pin, '', '', '') for n, pin in ((1, 7), (2, 12), (3, 15), (4, 33))}
BASE = {'channel_map': [1, 2, 3, 4], 'reversed': [False] * 4, 'command_timeout': 0.5,
        'decoupling': [1, 0, 0, 0, 1, 0, 0, 0, 1]}
needs_unicode = pytest.mark.skipif(theme.ASCII, reason='goldens are the Unicode glyph set')


def hw(**changes):
    return {'overrides': dict(changes), 'effective': {**BASE, **changes}}


class FakeHardware(revert.Backend):
    live_keys = frozenset({'command_timeout'})

    def __init__(self, value):
        self.value, self.applied = value, []

    def read(self):
        return self.value, None, {}

    def apply(self, version):
        self.value = version['value']
        self.applied.append(version['sha'])

    def verify(self, version, timeout):
        return True


class FakeApp:
    def __init__(self, text=None):
        self.text, self.notice, self.asked = text, '', []

    def ask_text(self, title, message, text=''):
        self.asked.append(title)
        return self.text


def make_env(**probes):
    return revert.Env(probes={k: (lambda v=v: v) for k, v in probes.items()}, armed=lambda: False,
                      hold=lambda token: contextlib.nullcontext())


@pytest.fixture
def state(tmp_path):
    h = History(tmp_path)
    h.record('hardware', hw(), source='tui:pins', time='2026-10-21T09:00:00+00:00')
    h.record('hardware', hw(channel_map=[3, 2, 3, 4], decoupling=[1, 0, 0, 0, 1, 0, 0, 0.08, 1]),
             source='tui:pins', applied='pending', time='2026-10-23T14:05:00+00:00')
    h.label('c0002', name='split C v2')
    h.record('hardware', hw(channel_map=[3, 2, 3, 4], command_timeout=0.3), source='external',
             time='2026-10-23T15:40:00+00:00')
    h.record('nav2', {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.4}}}},
             source='tui:nav2', time='2026-10-22T11:00:00+00:00')
    backend = FakeHardware(hw(channel_map=[3, 2, 3, 4], command_timeout=0.3))
    return popup.State(h, {'hardware': backend}, make_env(), now=NOW, pins=PINS,
                       fields=popup.render.load_fields(), rows=12)


def paint(state, cols, rows=30):
    state.height = rows
    modal = popup.modal(state, cols)
    panel, width = layout.popup(modal, cols, rows)
    out = io.StringIO()
    Console(width=width, file=out, color_system=None).print(panel)
    return out.getvalue()


def check(request, name, text):
    path = GOLDEN / name
    if request.config.getoption('--update-goldens'):
        path.parent.mkdir(exist_ok=True)
        path.write_text(text)
    assert text == path.read_text()


@needs_unicode
@pytest.mark.parametrize('cols', [120, 80])
def test_golden(request, state, cols):
    state.tab = 1                      # Hardware: three changes
    state.cursor = 1                   # the pending one, so the diff shows against the current version
    text = paint(state, cols)
    assert all(len(line) <= cols for line in text.splitlines())
    check(request, f'popup_{cols}.txt', text)


@needs_unicode
def test_glyph_grammar_in_the_list(state):
    state.tab = 1
    text = '\n'.join(line.plain for line in popup.list_lines(state, 60))
    assert text.count('now') == 1 and '± c0003' in text and '◌ c0002' in text
    assert 'split C v2' in text and 'hw' in text and 'Today' in text


def test_tabs_follow_subsystems_with_history(state):
    assert state.tabs() == [None, 'hardware', 'nav2']
    popup.handle(state, ']', FakeApp())
    assert state.tab == 1 and {c['subsystem'] for c in state.entries()} == {'hardware'}


def test_enter_plans_and_y_reverts_recording_revert_of(state):
    app = FakeApp()
    state.tab, state.cursor = 1, 2      # c0001, the oldest hardware change
    popup.handle(state, 'enter', app)
    assert state.plan.ok and 'channel_map' in state.plan.keys and popup.footer(state) == theme.SEPARATOR.join(('y apply', 'Esc cancel'))
    popup.handle(state, 'y', app)
    head = state.history.head('hardware')
    assert head['revert_of'] == 'c0001' and head['source'] == 'tui:revert'
    assert state.backends['hardware'].value == hw() and 'Reverted' in app.notice


def test_a_blocker_refuses_and_applies_nothing(state):
    state.env = make_env(procedure=True)
    state.tab, state.cursor = 1, 2
    popup.handle(state, 'enter', FakeApp())
    assert state.plan.blockers
    assert 'a procedure is running' in '\n'.join(line.plain for line in popup.detail_lines(state, 60))
    before = len(state.history.changes())
    popup.handle(state, 'y', FakeApp())
    assert state.backends['hardware'].applied == [] and len(state.history.changes()) == before
    assert state.message.plain == 'a procedure is running'


def test_escape_cancels_a_plan_before_it_closes(state):
    state.tab, state.cursor = 1, 2
    popup.handle(state, 'enter', FakeApp())
    assert popup.handle(state, 'escape', FakeApp()) is False and state.plan is None
    assert popup.handle(state, 'escape', FakeApp()) is True


def test_n_names_and_capital_n_notes(state):
    app = FakeApp('baseline')
    state.tab, state.cursor = 1, 0
    popup.handle(state, 'n', app)
    popup.handle(state, 'N', FakeApp('check the pins'))
    top = state.entries()[0]
    assert (top['name'], top['note']) == ('baseline', 'check the pins')


def test_capital_h_opens_the_popup(monkeypatch):
    from mhseals_testing.ui import app as app_module
    opened = []
    fake = type('A', (), {'open': lambda self, name: opened.append(name)})()
    app_module.App.dispatch(fake, 'H')
    assert opened == ['history']


def test_missing_backend_says_not_wired(state):
    state.backends = {}
    state.tab, state.cursor = 1, 2
    popup.handle(state, 'enter', FakeApp())
    assert state.plan is None and state.message.plain == 'cannot revert hardware: not wired'


def wide_revert(state):
    """A hardware revert whose diff is long: it touches outputs, reversed, timing and decoupling."""
    state.history.record('hardware', hw(channel_map=[4, 3, 2, 1], reversed=[True] * 4, command_timeout=0.9,
                                        decoupling=[1, 0.2, 0, 0, 1, 0, 0, 0.3, 1]),
                         source='external', time='2026-10-23T15:50:00+00:00')
    state.backends['hardware'].value = hw(channel_map=[4, 3, 2, 1], reversed=[True] * 4, command_timeout=0.9,
                                          decoupling=[1, 0.2, 0, 0, 1, 0, 0, 0.3, 1])
    state.tab, state.cursor = 1, 3
    popup.handle(state, 'enter', FakeApp())


@pytest.mark.parametrize('cols,rows', [(100, 30), (60, 24), (80, 20)])
def test_plan_and_blockers_survive_a_short_terminal(state, cols, rows):
    state.env = make_env(procedure=True)
    wide_revert(state)
    text = paint(state, cols, rows)
    assert 'Revert hardware to c0001' in text and 'a procedure is running' in text
    assert 'j/k scroll' not in text and len(text.splitlines()) <= rows
    assert 'y apply' in text


def test_the_diff_scrolls_with_capital_j_and_k(state):
    wide_revert(state)
    state.height = 16
    first = [line.plain for line in popup.detail_lines(state, 60, 10)]
    popup.handle(state, 'J', FakeApp())
    assert state.scroll == 1 and [line.plain for line in popup.detail_lines(state, 60, 10)] != first
    popup.handle(state, 'K', FakeApp())
    assert state.scroll == 0


def test_a_plan_shows_current_to_target_whatever_d_says(state):
    state.tab, state.cursor = 1, 1
    state.base = 'previous'
    before = [line.plain for line in popup.detail_lines(state, 60)]
    popup.handle(state, 'enter', FakeApp())
    detail = [line.plain for line in popup.detail_lines(state, 60)]
    assert any('vs current c0003' in line for line in detail) and not any('vs previous' in line for line in detail)
    assert any('vs previous' in line for line in before)


def test_plan_keys_wrap_instead_of_truncating(state):
    plan = revert.Plan('fcu', {'id': 'c0009'}, {'id': 'c0010'}, [f'PARAM_{n:02d}' for n in range(30)], [])
    lines = [line.plain for line in popup.plan_lines(plan, 40)]
    assert len(lines) > 3 and all(len(line) <= 40 for line in lines) and 'PARAM_29' in ' '.join(lines)


class TypeAheadApp(FakeApp):
    """The popup loop over scripted keys, recording what the shell showed before each key."""
    def __init__(self, keys):
        super().__init__()
        self.keys = keys
        self.console = type('C', (), {'size': type('S', (), {'width': 120, 'height': 30})()})()
        self.shown = []

    def revert_env(self):
        return make_env()

    def popup(self, shell, handle, refresh):
        for key in self.keys:
            refresh()
            self.shown.append(shell['body'].plain)
            if handle(key) is not None:
                break


def test_typed_ahead_enter_y_draws_the_preview_before_applying(state):
    app = TypeAheadApp(['down', 'enter', 'y'])
    app.history, app.config_backends = state.history, state.backends
    popup.open_history(app, 'hardware')
    assert 'Revert hardware to' in app.shown[2]          # drawn after Enter, before y arrived
    assert state.history.head('hardware')['revert_of']
