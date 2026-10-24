"""The RTK feed (UIW32): status ingest, gating, the baseline, the Status row, the strip item, the converge fact."""

import dataclasses
import io
import json
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.analysis import geo
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.probes import Facts
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.ingest import ingest
from mhseals_testing.ui import layout
from mhseals_testing.ui.feeds.rtk import RtkFeed, build
from mhseals_testing.ui.panels.status import status_panel, status_rows
from mhseals_testing.ui.rtk_row import rtk_row, strip_item

from test_dashboard import monitor, view
from test_rtk_row import GOLDEN, STATES, goldens

SPEC = SimpleNamespace(kind='gpsraw', name='GPS raw')
BASE = geo.ecef(43.0, -76.0, 100.0)


def gps_snapshot(fix=5, status='LIVE', lat=43.0001, lon=-76.0, alt=100.0):
    row = dict(status=status, values=dict(fix=fix, lat=lat, lon=lon, alt=alt))
    return dict(sensors={'GPS raw': row})


def app_with(profile=None):
    facts = Facts(lambda: 0.0)
    return SimpleNamespace(engine=None, health=SimpleNamespace(facts=facts)), facts


def boat(fcu=True):
    return SimpleNamespace(uses_fcu=fcu, sensors=(SPEC,))


def test_ingest_reads_json_and_legacy_diagnostics():
    m = monitor()
    ingest(m, load_profile('boat'), '/mhseals_testing/rtk/status', 'rtk',
           SimpleNamespace(data=json.dumps(dict(base='streaming', age=0.4))), 10)
    assert m.snapshot()['rtk']['base'] == 'streaming'
    legacy = SimpleNamespace(status=[SimpleNamespace(values=[
        SimpleNamespace(key='age', value='0.5'), SimpleNamespace(key='base', value='surveying')])])
    ingest(m, load_profile('boat'), '/mhseals_testing/rtk/status', 'rtk', legacy, 11)
    assert m.snapshot()['rtk'] == dict(age=0.5, base='surveying', received_age=0)
    ingest(m, load_profile('boat'), '/mhseals_testing/rtk/status', 'rtk', SimpleNamespace(data='{'), 12)
    assert any('unreadable rtk' in e[1] for e in m.snapshot()['events'])


def test_baseline_is_ecef_from_the_gps_raw_position():
    got = build(dict(base='streaming', age=0.4, base_position=dict(latitude=43.0, longitude=-76.0, height=100.0)),
                gps_snapshot(), (SPEC,))
    assert got.fix == 'FLOAT' and got.baseline == pytest.approx(geo.baseline(BASE, 43.0001, -76.0, 100.0))
    assert 10 < got.baseline < 12                                  # 0.0001 deg of latitude
    assert build({}, gps_snapshot(status='STALE'), (SPEC,)).fix == 'none'


def test_row_shows_only_with_a_base_or_the_topic():
    app, _ = app_with()
    feed = RtkFeed(boat(fcu=False))
    feed.tick(app, gps_snapshot(), 1.0)
    assert feed.current is None
    feed.tick(app, dict(gps_snapshot(), rtk=dict(base='streaming', age=0.3)), 2.0)
    assert feed.current.base == 'streaming'
    quiet = RtkFeed(boat())
    quiet.tick(app, gps_snapshot(fix=3), 1.0)
    assert quiet.current.base == 'none' and quiet.current.fix == '3D'


def test_converge_fact_counts_below_fixed_and_drives_the_shipped_check():
    app, facts = app_with()
    feed = RtkFeed(boat())
    snap = lambda fix, age=0.4: dict(gps_snapshot(fix), rtk=dict(base='streaming', age=age, received_age=0))
    seconds = lambda: facts.read()[0].get('rtk.seconds_below_fixed')
    feed.tick(app, snap(5), 100.0)
    feed.tick(app, snap(5), 290.0)
    assert seconds().value == 190
    spec = next(s for s in load(user=False).specs if s.id == 'rtk.converge')
    assert Engine([spec]).tick(*facts.read()).results['rtk.converge'].status == 'warn'
    feed.tick(app, snap(5, age=3.0), 300.0)         # stale link: not counted
    assert seconds() is None
    feed.tick(app, snap(5), 310.0)
    feed.tick(app, snap(6), 320.0)                  # FIXED resets it
    assert seconds() is None


def render_status(state, width):
    snapshot = monitor().snapshot()
    snapshot['rtk_view'] = state
    console = Console(file=io.StringIO(), width=width, height=40, color_system=None)
    console.print(status_panel(snapshot, view(load_profile('boat')), 40))
    line = next(l for l in console.file.getvalue().splitlines() if 'RTK' in l)
    return line[2:-2]                                # inside the border and its padding


@pytest.mark.parametrize('inner', (32, 28))
def test_status_column_matches_the_row_goldens(inner):
    rows = next(r for w, ascii, r in goldens() if w == inner and not ascii)
    for state, want in zip(STATES, rows):
        assert render_status(state, inner + 4) == want


def test_status_row_state_and_group():
    snapshot = monitor().snapshot()
    snapshot['rtk_view'] = STATES[9]
    row = next(r for r in status_rows(snapshot, load_profile('boat').sensors) if r['name'] == 'RTK')
    assert (row['group'], row['state']) == ('Localization', 'ERROR')
    assert all(r['name'] != 'RTK' for r in status_rows(monitor().snapshot(), load_profile('boat').sensors))


STRIP = ['LIVE rtk off', 'CHECKING rtk base', 'CHECKING rtk svin', 'CHECKING rtk svin', 'CHECKING rtk DGPS',
         'CHECKING rtk FLOAT', 'WARN rtk FLOAT', 'LIVE rtk FIXED', 'WARN rtk stale', 'FAIL rtk 14s', 'FAIL rtk 2 pubs', 'FAIL rtk none']


def test_strip_item_per_state():
    assert [' '.join(strip_item(s)) for s in STATES] == STRIP


def test_header_strip_carries_the_item():
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    assert 'rtk' not in layout.health_strip(snapshot, view(profile)).plain
    snapshot['rtk_view'] = STATES[9]
    assert '✗ rtk 14s' in layout.health_strip(snapshot, view(profile)).plain
    snapshot['rtk_view'] = STATES[5]
    assert ' rtk FLOAT' in layout.health_strip(snapshot, view(profile)).plain


POSITION = dict(latitude=43.0, longitude=-76.0, height=100.0)


def uiw31(**over):
    """The exact StatusTracker.status() shape of fix/UIW31: RtkView fields flat, NAV-SVIN detail under svin."""
    base = dict(base='streaming', age=0.4, drops=0, pubs=1, survey=0.0, survey_limit='time', survey_left=0.0,
                survey_acc=None, survey_target=None, svin=None, base_ecef=list(BASE), base_position=POSITION)
    return dict(base, **over)


SURVEYING = uiw31(base='surveying', age=None, survey=0.42, survey_limit='time', survey_left=72.0, survey_acc=1.9,
                  survey_target=2.0, base_ecef=None, base_position=None,
                  svin=dict(seconds=48.0, min_seconds=120.0, accuracy=1.9, observations=48, valid=False, active=True))


def test_uiw31_status_shapes_build_and_render():
    got = build(SURVEYING, gps_snapshot(fix=3), (SPEC,))
    assert (got.survey, got.survey_left, got.survey_acc, got.survey_target, got.survey_limit) == (
        0.42, 72.0, 1.9, 2.0, 'time')
    row = rtk_row(got, 42).plain
    assert got.baseline is None and '72s' in row and '█' in row
    slow = build(dict(SURVEYING, survey_limit='accuracy', survey_acc=5.0), gps_snapshot(fix=3), (SPEC,))
    assert slow.survey_limit == 'accuracy'
    live = build(uiw31(survey=1.0, survey_acc=1.5, survey_target=2.0), gps_snapshot(), (SPEC,))
    assert live.survey_left == 0 and 10 < live.baseline < 12 and '11.1m' in rtk_row(live, 42).plain
    assert strip_item(got) == ('CHECKING', 'rtk svin')


def test_unknown_base_value_reads_as_no_base():
    assert 'no base' in rtk_row(build(dict(base='idle'), gps_snapshot(fix=3), (SPEC,)), 42).plain


def test_busy_row_keeps_localization_unfolded_state():
    snapshot = monitor().snapshot()
    snapshot['rtk_view'] = STATES[2]
    row = next(r for r in status_rows(snapshot, load_profile('boat').sensors) if r['name'] == 'RTK')
    assert row['state'] == 'CHECKING'


def test_cursor_is_drawn_on_the_rtk_row_only_when_focused():
    def line(focused, at):
        snapshot = monitor().snapshot()
        snapshot['rtk_view'] = STATES[7]
        v = view(load_profile('boat'))
        names = [r['name'] for r in status_rows(snapshot, v['specs'])]
        v['focus'] = 'status' if focused else 'drive'
        v['cursors'] = {'status': names.index('RTK') if at else 0}
        console = Console(file=io.StringIO(), width=40, height=40, color_system=None)
        console.print(status_panel(snapshot, v, 40))
        return next(l for l in console.file.getvalue().splitlines() if 'RTK' in l)
    assert '›' in line(True, True) or '>' in line(True, True)
    assert '›' not in line(True, False) and '›' not in line(False, True)


def test_dead_corrections_show_while_the_base_reconnects_and_error_reads_bad():
    reconnect = lambda age: build(dict(base='identifying', age=age), gps_snapshot(fix=6), (SPEC,))
    assert 'RTK ⠇' not in rtk_row(reconnect(11.3), 42).plain         # no spinner once dead
    assert strip_item(reconnect(11.3)) == ('FAIL', 'rtk 11s')
    assert strip_item(reconnect(3.0)) == ('WARN', 'rtk stale')
    assert strip_item(reconnect(1.0))[0] == 'CHECKING'                # fresh frames: still just connecting
    assert strip_item(reconnect(None))[0] == 'CHECKING'               # never streamed: spinner
    error = build(dict(base='error'), gps_snapshot(fix=3), (SPEC,))
    assert strip_item(error) == ('FAIL', 'rtk error')
    assert 'error' in rtk_row(error, 42).plain and 'no base' not in rtk_row(error, 42).plain


def test_age_and_flow_facts_drive_the_shipped_checks():
    app, facts = app_with()
    feed = RtkFeed(boat())
    specs = [s for s in load(user=False).specs if s.id in ('rtk.age', 'rtk.flow')]
    assert len(specs) == 2
    specs = [dataclasses.replace(spec, raise_after=0.0) for spec in specs]     # the 2 s debounce is not under test

    def run(**status):
        feed.tick(app, dict(gps_snapshot(), rtk=dict(received_age=0, **status)), 1.0)
        return Engine(specs).tick(*facts.read()).results

    ok = run(base='streaming', age=0.5, rates=dict(bytes_per_s=900))
    assert ok['rtk.age'].status == 'ok' and ok['rtk.flow'].status == 'ok'
    dead = run(base='identifying', age=11.3, rates=dict(bytes_per_s=0))
    assert dead['rtk.age'].status == 'fail'
    assert 'rtk.flow' not in dead                                    # not streaming: no flow fact, check skips
    assert run(base='streaming', age=0.5, rates=dict(bytes_per_s=10))['rtk.flow'].status == 'warn'
    feed.tick(app, gps_snapshot(), 2.0)
    assert 'rtk.correction_age' not in facts.read()[0]


def test_power_cycled_base_with_an_age_reads_dead_and_never_streamed_reads_no_base():
    gone = build(dict(base='none', age=11.3), gps_snapshot(fix=6), (SPEC,))
    assert strip_item(gone) == ('FAIL', 'rtk 11s')
    never = build(dict(base='none', age=None), gps_snapshot(fix=3), (SPEC,))
    assert strip_item(never)[0] == 'LIVE' and 'no base' in rtk_row(never, 42).plain
