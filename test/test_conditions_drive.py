"""The Conditions . Command panel on Drive (ui.md 6.3, 4.5): goldens at 120, 100 and 80 columns, the
verdict word's hue and the yaw anchor swapping the sea slot."""

import io
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.feeds.conditions import compass, heading_sd
from mhseals_testing.ui.layout import render
from mhseals_testing.ui.panels import conditions
from mhseals_testing.validation import validate

from test_dashboard import monitor, view

GOLDEN = Path(__file__).parent / 'golden' / 'conditions'
INFO = dict(verdict='good', ceiling=0.94, drift=[0.0849, 0.0849], se=0.02, drift_trust=0.88, neutral_s=240.0,
            floor=math.radians(0.14), sea=math.radians(0.9), heading=30.0, yaw_ok=True)


def drive_view(**conditions_override):
    return view(load_profile('boat'), conditions={**INFO, **conditions_override}, command=(0.6, 0.0, 0.2),
                authority=('human', 'KEYS'), chords=True, speed=0.75)


def page(width, height, **override):
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=False)
    console.print(render(monitor().snapshot(), drive_view(**override), width, height))
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
@pytest.mark.parametrize('width,height', ((120, 40), (100, 36), (80, 30)))
def test_drive_conditions_matches_its_golden(width, height):
    golden = GOLDEN / f'drive_{width}.txt'
    actual = page(width, height)
    if os.environ.get('UPDATE_GOLDEN'):
        golden.parent.mkdir(exist_ok=True)
        golden.write_text(actual)
    assert actual == golden.read_text(), f'{golden.name} differs; UPDATE_GOLDEN=1 to regenerate'


def plain(renderable, width=40):
    console = Console(width=width, file=io.StringIO(), force_terminal=False)
    console.print(renderable)
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode text')
def test_title_and_bottom_rows_come_from_the_summary():
    out = plain(conditions.conditions_panel(drive_view()), 43)   # the 120 column panel
    assert 'Conditions · good 94% · 4 min' in out
    assert 'drift .12 m/s →045° ±.02 · trust 88%' in out
    assert 'floor .14°/s · sea .9°/s' in out and '↕' not in out
    assert ' │ keyboard·chords' in out and 'surge      +60%' in out and 'hdg 030° NNE' in out
    assert len(out.splitlines()) == 13


def title_style(info):
    title = conditions.conditions_title(info)
    return {str(span.style) for span in title.spans if title.plain[span.start:span.end] == info['verdict']}


def test_marginal_is_a_warn_word_poor_a_fail_word_and_good_is_plain():
    assert title_style({**INFO, 'verdict': 'marginal'}) == {theme.WARN}
    assert title_style({**INFO, 'verdict': 'poor'}) == {theme.BAD}
    assert not title_style(INFO) - {''}
    assert conditions.conditions_title({'verdict': 'unknown', 'reason': 'measuring 12/24 s'}).plain == 'measuring 12/24 s'


def test_unreliable_yaw_takes_the_sea_slot_and_flags_the_heading():
    ok, bad = conditions.sea_row(INFO), conditions.sea_row({**INFO, 'yaw_ok': False, 'yaw_note': '4°'})
    assert 'sea' in ok.plain and bad.plain == '! yaw 4°' and str(bad.style) == theme.WARN
    rows = plain(conditions.conditions_panel(drive_view(yaw_ok=False)), 43)
    assert '! yaw' in rows and 'sea ' not in rows and theme.pick('! hdg 030° NNE', '! hdg 030 NNE') in rows


def test_compass_bearing_from_odometry_yaw():
    snapshot = lambda yaw: {'sensors': {'odometry': {'values': {'yaw': yaw}}}}
    assert compass(snapshot(math.pi / 2)) == 0 and round(compass(snapshot(0.0))) == 90
    assert compass({'sensors': {}}) is None


def test_heading_sigma_from_the_odometry_yaw_variance():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id='odom', stamp=SimpleNamespace(sec=9, nanosec=0)), child_frame_id='base_link',
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0., y=0., z=0.),
                                                    angular=SimpleNamespace(x=0., y=0., z=0.))),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=0.),
                                                  orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)),
                             covariance=[0.0] * 35 + [math.radians(4) ** 2]))
    values = validate('odometry', message, 10.0).values
    snapshot = lambda var: {'sensors': {'odometry': {'values': {'yaw_var': var}}}}
    assert heading_sd(snapshot(values['yaw_var'])) == pytest.approx(4)
    assert [heading_sd(snapshot(v)) for v in (None, 0.0, -1.0, math.nan, math.inf, 10.0)] == [None] * 6
    assert heading_sd({'sensors': {}}) is None


def test_motion_no_longer_draws_a_drift_line():
    assert 'Drift' not in page(120, 40) and 'fit not characterized' not in page(120, 40)


@pytest.mark.skipif(theme.ASCII, reason='the Unicode text')
def test_compact_form_names_heading_drift_and_command_and_drops_what_does_not_fit():
    ok = plain(conditions.conditions_panel(drive_view(), True), 70)
    assert 'good 94% · hdg 030° NNE · drift .12→045° ±.02' in ok
    assert 'Cmd ↑ surge   │█▎+60% · yaw ↺  ▋│  +20% · keyboard·chords' in ok
    assert '! hdg 030° NNE' in plain(conditions.conditions_panel(drive_view(yaw_ok=False), True), 60)
    narrow = plain(conditions.conditions_panel(drive_view(), True), 30).splitlines()
    assert narrow[1] == '│ good 94% · hdg 030° NNE    │' and narrow[2] == '│ Cmd ↑ surge   │█▎+60%      │'
    assert conditions.conditions_title({**INFO, 'state': 'trending'}).plain == 'good 94% · 4 min · trending'
    assert conditions.conditions_title({**INFO, 'state': 'changed'}).plain.endswith('· changed')
    assert conditions.conditions_title({**INFO, 'state': 'steady'}).plain == 'good 94% · 4 min'
