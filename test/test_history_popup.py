"""popup.history: goldens at 120 and 80 columns (UIW15)."""

import datetime
import io
from pathlib import Path

from rich.console import Console
import pytest

from mhseals_testing.history import History
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


class FakeApp:
    def __init__(self, text=None):
        self.text, self.notice, self.asked = text, '', []

    def ask_text(self, title, message, text=''):
        self.asked.append(title)
        return self.text


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
    return popup.State(h, now=NOW, pins=PINS,
                       fields=popup.render.load_fields(), rows=12)


def paint(state, cols, rows=30):
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
    state.cursor = 1
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
