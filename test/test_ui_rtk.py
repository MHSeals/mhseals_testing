"""The RTK pane in the Odometry popup and the base controls behind ``r`` (gps.md 7)."""
import time
from types import SimpleNamespace

import pytest
import yaml

from mhseals_hardware.menu import Cancelled
from mhseals_testing import rtk
from mhseals_testing.harness import reload as reload_
from mhseals_testing.ui import rtk as ui_rtk

EVENTS = [dict(phase='gps_fix', t=t, fix=f) for t, f in ((0, 'none'), (10, '3D'), (40, 'FLOAT'), (100, 'FIXED'))]
POSITION = dict(latitude=33.0, longitude=-117.0, height=20.0)


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / '.config'))
    monkeypatch.delenv('MHSEALS_RTK_CONFIG', raising=False)
    return tmp_path


class FakeApp:
    """choose/ask_text answer from queues; Cancelled when empty."""

    def __init__(self, link, choices=(), texts=()):
        self.rtk_link, self.choices, self.texts, self.notice, self.events = link, list(choices), list(texts), '', []

    def choose(self, title, options, *args, **kwargs):
        if not self.choices:
            raise Cancelled('cancelled')
        answer = self.choices.pop(0)
        assert answer in options
        return answer

    def ask_text(self, title, message, text=''):
        return self.texts.pop(0) if self.texts else None

    def emit(self, phase, **details):
        self.events.append(phase)


def link():
    base = SimpleNamespace(position=POSITION)
    return SimpleNamespace(base=base, reload=reload_.Reload(reload_.load_rules()))


def test_pane_golden_from_a_fake_fixlog():
    rows = ui_rtk.pane_rows(EVENTS, 130, width=24, ascii=True)
    assert [text for _, text in rows] == [
        '', '__------+++++++++++#####', 'now FIXED', 'none     0:10  x1', '3D       0:30  x1',
        'DGPS     0:00  x0', 'FLOAT    1:00  x1', 'FIXED    0:30  x1', 'first FIXED 1:40', 'r  base controls']
    assert rows[0] == ('RTK', '')


def test_r_in_the_odometry_popup_opens_the_controls(tmp_path, home, monkeypatch):
    from test_app import make_app, press
    app, _ = make_app(tmp_path)
    app.keys, app.rtk_link = app.reader, link()
    opened = []
    monkeypatch.setattr(ui_rtk, 'open_controls', opened.append)
    press(app, 'r')
    app.odometry()
    assert opened == [app]
    monkeypatch.undo()
    assert not (home / '.config/mhseals_testing/rtk.yaml').exists()
    app.rtk_link = None
    press(app, 'r')
    app.odometry()
    assert app.notice == 'no base'


def test_r_saves_atomically_and_restarts_once(home):
    app, calls = FakeApp(link(), ['Start survey', 'Start'], ['300 1.5']), []
    done = ui_rtk.open_controls(app, restart=lambda *args: calls.append(args) or True)
    saved = yaml.safe_load((home / '.config/mhseals_testing/rtk.yaml').read_text())
    assert done and saved['survey_seconds'] == 300 and saved['survey_accuracy'] == 1.5
    assert len(calls) == 1 and calls[0][1].position is None and calls[0][3] == {}
    assert not list((home / '.config/mhseals_testing').glob('*.tmp*')) and 'survey' in app.notice


def test_known_position_restarts_on_the_chosen_name(home):
    rtk.save_config(dict(rtk.DEFAULTS, positions={'pier': POSITION}))
    app, calls = FakeApp(link(), ['Known position', 'pier', 'Start']), []
    assert ui_rtk.open_controls(app, restart=lambda *args: calls.append(args) or True)
    assert calls[0][3] == {'pier': POSITION}


def test_no_base_says_so_and_writes_nothing(home):
    app = FakeApp(None, ['Start survey', 'Start'], ['300 1.5'])
    assert not ui_rtk.open_controls(app, restart=lambda *args: pytest.fail('restarted'))
    assert app.notice == 'no base' and not (home / '.config').exists()


def test_bad_input_and_cancel_write_nothing(home):
    app = FakeApp(link(), ['Start survey'], ['300 500'])
    assert not ui_rtk.open_controls(app, restart=lambda *args: pytest.fail('restarted'))
    assert 'accuracy' in app.notice and not (home / '.config/mhseals_testing/rtk.yaml').exists()
    assert not ui_rtk.open_controls(FakeApp(link(), ['Start survey'], ['300 1.5']))   # no confirm: cancelled
    assert not (home / '.config/mhseals_testing/rtk.yaml').exists()


@pytest.mark.parametrize('choice, text', [('Start survey', 'inf 1'), ('Settings', '- inf'), ('Start survey', 'nan 1')])
def test_non_finite_numbers_are_refused_not_a_crash(home, choice, text):
    app = FakeApp(link(), [choice, 'Start'], [text])
    assert not ui_rtk.open_controls(app, restart=lambda *args: pytest.fail('restarted'))
    assert 'numbers only' in app.notice and not (home / '.config/mhseals_testing/rtk.yaml').exists()


def test_odometry_pane_uses_wall_time_for_journal_events(tmp_path, monkeypatch):
    """gps_fix events carry ISO wall ``time`` (no ``t``); the pane's now must be wall seconds."""
    from datetime import datetime, timezone
    from test_app import make_app
    from mhseals_testing.harness import journal
    app, _ = make_app(tmp_path)
    t0 = 1_800_000_000.0
    for dt, fix in ((0, 'none'), (10, '3D'), (40, 'FIXED')):
        stamp = datetime.fromtimestamp(t0 + dt, timezone.utc).isoformat()
        journal.append(app.paths.journal, dict(phase='gps_fix', time=stamp, fix=fix))
    monkeypatch.setattr(time, 'time', lambda: t0 + 100)
    modal = {}
    monkeypatch.setattr(type(app), 'choose', lambda self, title, options, keys=None, rows=None, **kw: modal.update(rows=rows) or 'Close')
    app.odometry()
    text = [row[1] for row in modal['rows']]
    assert any(line.startswith('FIXED') and '1:00' in line for line in text), text
