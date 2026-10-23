"""The Correction tab in the app: body, goldens and keys (ui spec 4.1, 5.7, 6.7)."""

import io
import os
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing import data
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.layout import render
from mhseals_testing.ui.panels import correction

from mhseals_testing.ui.app import CORRECTION_HINTS
from test_app import make_app, press
from test_dashboard import monitor, view

C = [1, 0, 0, .031, 1, .02, -.042, .084, 1]
HARDWARE = dict(thruster_matrix=[1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1], decoupling=C,
                characterization=dict(identified_at='2026-10-22T14:05:11+00:00',
                                      speed_curves={'sway-': dict(d1=2.2, d2=5.8)}))


def fit(hi, lo):
    return dict(trust_hi=hi, trust_lo=lo, peak=.7, sigma=.006, age='2h', fit=91, fit_label='good',
                holds=[dict(a=.6, main=-.21, cross=.012, j=.31)])


FITS = {d: fit(96, 91) for d in correction.DIRECTIONS} | {'sway-': fit(71, None)}


def page(width, height=40):
    state = dict(fits=FITS, column=None)
    v = view(load_profile('boat'), tab='Correction', focus='table', hardware=HARDWARE, correction=state,
                focus_title=('Correction', 'sway− column'), hints=CORRECTION_HINTS)
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=False)
    console.print(render(monitor().snapshot(), v, width, height))
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
@pytest.mark.parametrize('width,height', [(120, 40), (100, 36)])
def test_correction_matches_its_golden(width, height):
    golden = Path(__file__).parent / 'golden' / 'correction' / f'tab_{width}.txt'
    actual = page(width, height)
    if os.environ.get('UPDATE_GOLDEN'):
        golden.write_text(actual)
    assert actual == golden.read_text(), f'{golden.name} differs; UPDATE_GOLDEN=1 to regenerate'


def test_data_reads_c_and_speed_curves_from_the_config():
    model = correction.tab_view(HARDWARE, dict(fits=FITS), 'mhseals.state/1')
    assert model['cursor'] == 3                                   # the least trusted column: sway-
    assert model['columns'][3]['cells']['drag'] == '2.2·5.8'
    assert model['header']['fitted'] == '10-22 14:05'


def tab_app(tmp_path):
    app, _ = make_app(tmp_path)
    app.args.hardware_config = dict(app.args.hardware_config, **HARDWARE)
    data.write_json(data.path('state.json'), dict(fits=FITS), 'mhseals.state/1')
    app.dispatch(']')
    assert app.tab == 'Correction'
    return app


def test_right_moves_the_stripe_from_the_worst_column(tmp_path):
    app = tab_app(tmp_path)
    app.dispatch('right')
    assert app.column == 4
    app.dispatch('h')
    app.dispatch('h')
    assert app.column == 2


def test_r_and_capital_r_start_characterize_with_the_direction(tmp_path):
    app = tab_app(tmp_path)
    seen = []
    app.run_procedure = lambda p: seen.append((p.title, app.settings.direction))
    app.dispatch('r')
    app.dispatch('right')
    app.dispatch('r')
    app.dispatch('R')
    assert seen == [('Characterize', 'sway-'), ('Characterize', 'yaw+'), ('Characterize', '')]
    assert app.settings.direction == ''



def test_enter_shows_the_holds_of_the_cursor_column(tmp_path):
    app = tab_app(tmp_path)
    seen = []
    app.show = lambda title, rows, width: seen.append((title, rows))
    app.dispatch('enter')
    assert seen[0][0].startswith('sway-')
    assert seen[0][1] == correction.hold_rows(correction.tab_view(app.args.hardware_config, dict(fits=FITS)))
    app.dispatch('left')
    app.dispatch('enter')
    assert seen[1][0].startswith('sway+')


def stage(app):
    """A staged identity C with the real C+ in the backup."""
    from mhseals_testing.harness.apply import Staging
    Staging(app.args.config).raw_correction()


def test_a_without_a_backup_says_nothing_pending(tmp_path):
    app = tab_app(tmp_path)
    app.dispatch('A')
    assert app.notice == 'Nothing pending'


def test_a_restores_and_reloads_the_config(tmp_path):
    from mhseals_hardware.configuration import load_config
    app, _ = make_app(tmp_path)
    config = Path(app.args.config)
    config.write_text(config.read_text() + 'decoupling: %s\n' % C)
    app.dispatch(']')
    stage(app)
    assert load_config(app.args.config)['decoupling'] != C
    press(app, 'y')
    with app.attached():
        app.dispatch('A')
    assert load_config(app.args.config)['decoupling'] == C
    assert app.args.hardware_config['decoupling'] == C
    assert app.staging().pending() is None
    assert any(c['source'] == 'tui:correction' for c in app.history.changes('hardware'))


def test_a_cancelled_confirm_keeps_the_backup(tmp_path):
    app, _ = make_app(tmp_path)
    app.dispatch(']')
    stage(app)
    press(app, 'escape')
    with app.attached():
        app.dispatch('A')
    assert app.staging().pending() is not None


def test_i_cancel_leaves_the_epoch(tmp_path):
    app = tab_app(tmp_path)
    press(app, 'escape')                      # the note popup
    with app.attached():
        app.dispatch('I')
        press(app, 'x', 'enter', 'escape')        # a note, then the confirm declined
        app.dispatch('I')
    assert app.args.hardware_config.get('hardware_epoch', 0) == 0


def test_i_confirmed_bumps_the_epoch_and_saves_through_save_hardware(tmp_path, monkeypatch):
    from mhseals_hardware import configuration
    defaults = configuration.load_defaults
    # the hardware package does not declare hardware_epoch yet (WIRE-LIST UIW20)
    monkeypatch.setattr(configuration, 'load_defaults', lambda: dict(defaults(), hardware_epoch=0))
    app = tab_app(tmp_path)
    saved = []
    real = app.save_hardware
    app.save_hardware = lambda **c: saved.append(c) or real(**c)
    press(app, 'p', 'enter', 'y')
    with app.attached():
        app.dispatch('I')
    assert saved == [dict(hardware_epoch=1)]
    assert app.args.hardware_config['hardware_epoch'] == 1
    assert app.history.head('hardware')['note'] == 'p'


def test_i_is_refused_while_moving(tmp_path):
    app = tab_app(tmp_path)
    app.manual = True
    app.dispatch('I')
    assert app.notice == 'Not while moving'
    assert app.args.hardware_config.get('hardware_epoch', 0) == 0
