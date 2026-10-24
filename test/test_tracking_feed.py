"""Live tracking: the feed scores a series against the model, and Motion draws score and ghost."""

import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from mhseals_testing.analysis.predictor import Predictor
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.feeds.tracking import TrackingFeed
from mhseals_testing.ui.panels.motion import motion_panel
from test_dashboard import text, view
from test_motion import driven

HERE = Path(__file__).parent
CURVES = {f'{a}{s}': {'d1': 1.0, 'd2': 0.5} for a in ('surge', 'sway', 'yaw') for s in '+-'}
T95 = {'surge': 1.5, 'sway': 1.5, 'yaw': 1.0}
DT = 0.1


def app(verdict='good', units='effort'):
    char = {'speed_curves': CURVES, 't95': T95, 'wave_floor': [0.01, 0.01, 0.01]}
    return SimpleNamespace(
        profile=SimpleNamespace(command_units=units), args=SimpleNamespace(hardware_config={'characterization': char}),
        conditions=SimpleNamespace(evaluate=lambda: {'verdict': verdict, 'drift': None}))


def session(command, seconds=15, gain=1.0):
    """Plots for ``seconds`` of constant ``command`` with measurements ``gain`` times the model's."""
    model = Predictor(CURVES, T95)
    cmd, meas = [], {'vx': [], 'vy': [], 'wz': []}
    for n in range(1, int(seconds / DT) + 1):
        t = n * DT
        cmd.append((t, command))
        for key, v in zip(meas, model.step(command, DT)):
            meas[key].append((t, v * gain))
    return dict(now=cmd[-1][0], window=10.0, meas=meas, cmd=cmd)


def tracked(plots, **kwargs):
    feed = TrackingFeed()
    feed.tick(app(**kwargs), dict(plots=plots, sensors={}), plots['now'])
    return feed


def test_a_series_matching_the_model_scores_ok():
    feed = tracked(session((0.6, 0.0, 0.0)))
    surge = feed.view()['tracking']
    assert surge['score'][0] >= 0.8 and surge['colour'][0] == 'green'
    assert surge['score'][1] is None and surge['score'][2] is None     # nothing commanded: gated


def test_a_boat_that_ignores_the_command_scores_fail():
    feed = tracked(session((0.6, 0.0, 0.0), gain=0.0))
    assert feed.view()['tracking']['colour'][0] == 'red'


def test_a_poor_conditions_verdict_gates_every_axis():
    assert tracked(session((0.6, 0.0, 0.0)), verdict='poor').scores == (None, None, None)


def test_velocity_units_are_not_modelled():
    assert tracked(session((0.6, 0.0, 0.0)), units='velocity').view() == {}


def test_a_new_calibration_restarts_the_ghost():
    feed = tracked(session((0.6, 0.0, 0.0)))
    other = app()
    other.args.hardware_config['characterization']['t95'] = {'surge': 3.0}
    feed.tick(other, dict(plots=session((0.6, 0.0, 0.0), seconds=1), sensors={}), 1.0)
    assert len(feed.pred[0]) < 15


def panel(width, tracking, height=20, **overrides):
    profile = load_profile('boat')
    v = view(profile, units='effort', command=(0.6, 0.0, 0.0), tracking=tracking, **overrides)
    return text(motion_panel(driven().snapshot(), v, height), width, height)


def test_header_reads_track_percent_and_a_dash_when_gated():
    tracking = tracked(session((0.6, 0.0, 0.0))).view()['tracking']
    lines = panel(50, tracking).splitlines()
    surge = next(line for line in lines if 'Surge' in line)
    sway = next(line for line in lines if 'Sway' in line)
    assert f"track {tracking['score'][0]:.0%}  cmd +60%" in surge
    assert f'track {theme.DASH}  cmd' in sway


def test_short_header_below_44_columns_drops_the_label():
    tracking = dict(pred=[[], [], []], score=(0.91, None, 0.5), colour=['green', None, 'red'])
    out = panel(36, tracking)
    assert f"Surge 91%   +60% {theme.pick('→', '->')} +0.20" in out and f'Sway {theme.DASH}  ' in out and 'track' not in out


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_motion_golden_with_the_model_ghost():
    plots = session((0.6, 0.0, 0.0))
    feed = tracked(plots)
    out = text(motion_panel(driven().snapshot(), view(load_profile('boat'), units='effort',
                            command=(0.6, 0.0, 0.0), tracking=feed.view()['tracking'],
                            hardware={'characterization': dict(app().args.hardware_config['characterization'],
                                                                mechanical_bias=dict(surge=0.0, sway=0.0, yaw=0.0))}), 20), 46, 20)
    assert 'model' in out and 'pred' not in out
    golden = HERE / 'golden' / 'motion' / 'track_120.txt'
    if os.environ.get('UPDATE_GOLDEN'):
        golden.parent.mkdir(exist_ok=True)
        golden.write_text(out)
    assert out == golden.read_text()


def test_axes_without_a_fit_keep_the_command_ghost_and_score_none():
    one = app()
    one.args.hardware_config['characterization']['speed_curves'] = {'surge+': CURVES['surge+']}
    plots = session((0.6, 0.0, 0.0))
    feed = TrackingFeed()
    feed.tick(one, dict(plots=plots, sensors={}), plots['now'])
    tracking = feed.view()['tracking']
    assert tracking['pred'][0] and not tracking['pred'][1] and not tracking['pred'][2]
    assert tracking['score'][1:] == (None, None)


def test_an_uncharacterized_boat_contributes_nothing():
    bare = app()
    bare.args.hardware_config['characterization'] = {'wave_floor': [0.01, 0.01, 0.01]}
    plots = session((0.6, 0.0, 0.0))
    feed = TrackingFeed()
    feed.tick(bare, dict(plots=plots, sensors={}), plots['now'])
    assert feed.view() == {} and feed.scores == (None, None, None)


def test_missing_plots_or_velocity_units_clear_the_stale_ghost():
    feed = tracked(session((0.6, 0.0, 0.0)))
    assert feed.view()['tracking']['pred'][0]
    feed.tick(app(), dict(plots=None, sensors={}), 0.0)
    assert not any(feed.pred) and not any(feed.cmd) and feed.scores == (None, None, None)
    feed = tracked(session((0.6, 0.0, 0.0)))
    feed.tick(app(units='velocity'), dict(plots=session((0.6, 0.0, 0.0)), sensors={}), 15.0)
    assert not any(feed.pred) and not any(feed.cmd)


@pytest.mark.parametrize('inner', range(18, 28))
def test_short_header_always_shows_the_whole_measurement(inner):
    """Motion has 18-27 inner columns at 90-99: the score, then cmd and arrow, yield before the measurement."""
    from mhseals_testing.ui.panels.motion import _axis
    tracking = dict(pred=[[], [], []], score=(0.91, 0.5, 0.5), colour=['green', 'red', 'red'])
    v = view(load_profile('boat'), units='effort', command=(0.6, 0.0, 0.0), tracking=tracking)
    for index in range(3):
        header = _axis(index, driven().snapshot(), v, inner, 1)[0].plain
        assert ('+0.20', '+0.00', '+0.10')[index] in header
        assert len(header) <= inner


def test_ticks_feed_the_session_summary_and_close_records_it(tmp_path):
    from mhseals_testing import data
    path = tmp_path / 'current.json'
    data.write_json(path, {'directions': {'forward': {'fit': {'id': 'C3'}}}}, 'mhseals.calibration/1')
    feed = TrackingFeed(path=path)
    plots = session((0.6, 0.0, 0.0))
    feed.tick(app(), dict(plots=plots, sensors={}), plots['now'])
    assert feed.axes['surge'][1] == 1 and 'sway' not in feed.axes and feed.version == 'forward:C3'
    verdicts = feed.close()
    sessions = data.read_json(path)['provenance']['tracking']['surge']['sessions']
    assert sessions[0]['version'] == 'forward:C3' and sessions[0]['n'] == 1 and set(verdicts) == {'surge'}


def test_a_recalibration_mid_session_records_one_session_per_version(tmp_path):
    from mhseals_testing import data
    path = tmp_path / 'current.json'
    data.write_json(path, {'directions': {'forward': {'fit': {'id': 'C3'}}}}, 'mhseals.calibration/1')
    feed = TrackingFeed(path=path)
    plots = session((0.6, 0.0, 0.0))
    snapshot = dict(plots=plots, sensors={})
    feed.tick(app(), snapshot, plots['now'])
    feed.tick(app(), snapshot, plots['now'])
    data.write_json(path, {**data.read_json(path), 'directions': {'forward': {'fit': {'id': 'C4'}}}}, 'mhseals.calibration/1')
    feed.tick(app(), snapshot, plots['now'])
    feed.close()
    sessions = data.read_json(path)['provenance']['tracking']['surge']['sessions']
    assert [(s['version'], s['n']) for s in sessions] == [('forward:C3', 2), ('forward:C4', 1)]


def test_banking_a_superseded_version_keeps_the_cleared_staleness(tmp_path):
    from mhseals_testing import calibration, data
    from mhseals_testing.analysis import tracking
    from mhseals_testing.health import probes
    path = tmp_path / 'current.json'
    data.write_json(path, {'directions': {'forward': {'fit': {'id': 'C3'}}}}, 'mhseals.calibration/1')
    feed = TrackingFeed(path=path)
    for _ in range(3):
        tracking.record_session(path, 'surge', dict(mean=0.1, n=5, trust=1.0, version='forward:C3'), feed.settings)
    assert probes.staleness_facts(calibration.load(path)) != {}
    feed.add('surge', 0.1, 1.0, 'forward:C3')
    calibration.record('forward', dict(id='C4'), {}, path=path)
    assert probes.staleness_facts(calibration.load(path)) == {}
    feed.add('surge', 0.6, 1.0, 'forward:C4')            # banks the C3 sums without re-judging them
    assert probes.staleness_facts(calibration.load(path)) == {}
