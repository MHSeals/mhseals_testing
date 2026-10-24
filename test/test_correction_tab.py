"""The Correction tab in the app: body, goldens and keys (ui spec 4.1, 5.7, 6.7)."""

import io
import json
import os
import re
from pathlib import Path

import pytest
from rich.console import Console
from rich.text import Text

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


def test_r_and_capital_r_refuse_without_changing_direction(tmp_path):
    from mhseals_testing.harness.apply import CORRECTION_ACK_REQUIRED
    app = tab_app(tmp_path)
    before = app.settings.direction
    for key in ('r', 'right', 'r', 'R'):
        app.dispatch(key)
    assert CORRECTION_ACK_REQUIRED in app.notice
    assert app.settings.direction == before


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


def test_a_keeps_pending_backup_without_runtime_restoration_proof(tmp_path):
    from mhseals_testing.harness.apply import CORRECTION_ACK_REQUIRED
    app, _ = make_app(tmp_path)
    app.dispatch(']')
    stage(app)
    before = Path(app.args.config).read_bytes(), app.staging().state.read_bytes()
    app.dispatch('A')
    assert CORRECTION_ACK_REQUIRED in app.notice
    assert app.staging().pending() is not None
    assert (Path(app.args.config).read_bytes(), app.staging().state.read_bytes()) == before


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


SUMMARY = dict(verdict='marginal', drift=[.03, .04], ceiling=.82)
PROVENANCE = dict(sway=dict(staleness=dict(flag=True, reason='falling trend', slope=-.02)),
                  surge=dict(staleness=dict(flag=False, slope=.01)))


def side_page(width, blocking=('Thrusters: no 12 V',), height=40):
    state = dict(fits=FITS, column=None,
                 readiness=correction.readiness_data(SUMMARY, blocking, ['IMU: 40 Hz'], [.003, .012, .011]),
                 tracking=correction.tracking_data((.91, None, .74), dict(surge=[1.8, 2, 1.6]), PROVENANCE, 'v3'))
    v = view(load_profile('boat'), tab='Correction', focus='table', hardware=HARDWARE, correction=state,
             focus_title=('Correction', 'sway− column'), hints=CORRECTION_HINTS)
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=True, color_system='standard')
    console.print(render(monitor().snapshot(), v, width, height))
    return console.file.getvalue()


def plain(text):
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_side_column_matches_its_golden():
    golden = Path(__file__).parent / 'golden' / 'correction' / 'side_120.txt'
    actual = plain(side_page(120))
    if os.environ.get('UPDATE_GOLDEN'):
        golden.write_text(actual)
    assert actual == golden.read_text(), 'side_120.txt differs; UPDATE_GOLDEN=1 to regenerate'


def sgr(style, text):
    """``text`` in ``style`` on the same 16-colour console the page uses, so the codes follow the rich version."""
    console = Console(width=40, file=io.StringIO(), force_terminal=True, color_system='standard')
    console.print(Text(text, style=style), end='')
    return console.file.getvalue()


def test_failing_gate_shows_the_fail_glyph_in_the_fail_hue():
    page = side_page(120)
    assert f"{sgr(theme.BAD, '✗')} Thrusters: no 12 V" in page
    assert f"{sgr(theme.WARN, '!')} IMU: 40 Hz" in page


def test_correction_side_asks_the_characterize_gate_and_reads_feed_and_provenance(tmp_path):
    from types import SimpleNamespace
    from mhseals_testing.procedures import PROCEDURES
    app = tab_app(tmp_path)
    calls = []

    def gate(key, bench, armed):
        calls.append((key, bench, armed))
        return ['Thrusters: no 12 V'], ['IMU: 40 Hz']

    app.health = SimpleNamespace(engine=SimpleNamespace(gate=gate))
    data.write_json(data.path('calibration', 'current.json'), dict(provenance=dict(tracking=PROVENANCE)), 'mhseals.calibration/1')
    app.tracking.scores, app.tracking.axes, app.tracking.version = (.9, None, .7), dict(surge=[1.8, 2, 1.6]), 'v9'
    characterize = next(p for p in PROCEDURES if p.title == 'Characterize')
    side = app.correction_side()
    assert calls == [(characterize.key, app.args.bench_override, 'armed' in characterize.needs)]
    assert side['readiness']['gates'] == [('bad', 'Thrusters: no 12 V'), ('warn', 'IMU: 40 Hz')]
    assert side['tracking']['rows'][0][:3] == ('surge', '90%', '90%')
    assert side['tracking']['notes'] == ['stale: sway falling trend'] and side['tracking']['version'] == 'v9'


def test_no_side_column_at_100():
    text = plain(side_page(100, height=36))
    assert 'Readiness' not in text and 'Tracking ·' not in text


def test_readiness_and_tracking_models():
    r = correction.readiness_data(SUMMARY, [], [])
    assert r['gates'] == [('good', 'gates clear')] and r['rows'][0] == ('drift', '.050 m/s')
    t = correction.tracking_data((.91, None, .74), dict(surge=[1.8, 2, 1.6]), PROVENANCE)
    assert t['rows'][0][:3] == ('surge', '91%', '90%') and t['notes'] == ['stale: sway falling trend']


@pytest.mark.parametrize('source,bad', [
    (source, bad) for source in ('overall', 'direction', 'live', 'saved')
    for bad in (True, False, '0.9', [], {}, float('nan'), float('inf'), -float('inf'), 10 ** 400)
] + [(source, 1e308) for source in ('overall', 'direction', 'live')])
def test_app_correction_renders_unknown_for_malformed_percentages(tmp_path, bad, source):
    app = tab_app(tmp_path)
    water = dict(verdict='marginal', ceiling=.66)
    current = dict(directions={'surge+': dict(conditions=water)})
    live = {'surge+': .97}
    saved = {}
    if source == 'overall':
        water['ceiling'] = bad
    elif source == 'direction':
        water['ceilings'] = {'surge+': bad}
    elif source == 'live':
        live['surge+'] = bad
    else:
        saved['surge+'] = dict(fit=bad, ceiling=bad, trust_hi=bad, trust_lo=bad, tracking=bad)
    # Imported files can contain values the application's own JSON writer would reject.
    data.path('state.json').write_text(json.dumps(dict(fits=saved)))
    target = data.path('calibration', 'current.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(current))
    app.conditions._cache = dict(app.conditions.evaluate(), ceilings=live)
    app.conditions._at = app.conditions.clock()
    actual = app.view()
    model = correction.tab_view(app.args.hardware_config, actual['correction'])
    cells = model['columns'][0]['cells']
    assert cells['fit'] == (theme.DASH if source != 'live' else 'marg 66')
    assert cells['ceiling'] == (theme.DASH if source in ('live', 'saved') else '97')
    if source == 'saved':
        assert cells['trust'] == theme.DASH == cells['tracking']
        assert model['header']['min_trust'] is None
    assert [c['key'] for c in model['columns']] == list(correction.DIRECTIONS)
    console = Console(width=120, height=40, file=io.StringIO(), force_terminal=False)
    console.print(render(app.monitor.snapshot(), actual, 120, 40))
    assert 'Correction' in console.file.getvalue()


def test_app_correction_keeps_distinct_fit_and_live_ceilings(tmp_path):
    app = tab_app(tmp_path)
    live = {d: .80 + i / 100 for i, d in enumerate(correction.DIRECTIONS)}
    current = dict(directions={d: dict(conditions=dict(
        verdict='marginal', ceiling=.99, ceilings={d: .60 + i / 100}))
        for i, d in enumerate(correction.DIRECTIONS)})
    data.write_json(data.path('state.json'), dict(fits={}), 'mhseals.state/1')
    data.write_json(data.path('calibration', 'current.json'), current, 'mhseals.calibration/1')
    app.conditions._cache = dict(app.conditions.evaluate(), ceilings=live)
    app.conditions._at = app.conditions.clock()
    model = correction.tab_view(app.args.hardware_config, app.view()['correction'])
    for i, col in enumerate(model['columns']):
        assert col['cells']['fit'] == f'marg {60 + i}'
        assert col['cells']['ceiling'] == f'{80 + i}' + correction.fitrows.UP
        assert col['cells']['trust'] == theme.DASH == col['cells']['sigma']
