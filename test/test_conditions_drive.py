"""The Conditions . Command panel (ui.md 6.3): rows, the verdict hue, the yaw anchor swapping the sea slot."""

import io
import math

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.feeds.conditions import compass
from mhseals_testing.ui.panels import conditions

from test_dashboard import view

INFO = dict(verdict='good', ceiling=0.94, drift=[0.0849, 0.0849], se=0.02, drift_trust=0.88, neutral_s=240.0,
            floor=math.radians(0.14), sea=math.radians(0.9), heading=30.0, yaw_ok=True)


def drive_view(**conditions_override):
    return view(load_profile('boat'), conditions={**INFO, **conditions_override}, command=(0.6, 0.0, 0.2),
                authority=('human', 'KEYS'), chords=True)


def plain(renderable, width=40):
    console = Console(width=width, file=io.StringIO(), force_terminal=False)
    console.print(renderable)
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode text')
def test_title_and_bottom_rows_come_from_the_summary():
    out = plain(conditions.conditions_panel(drive_view()))
    assert 'Conditions · good 94% · 4 min' in out
    assert 'drift .12 m/s →045° ±.02 · trust 88%' in out
    assert 'floor .14°/s · sea .9°/s' in out and '↕' not in out
    assert ' │ Command' in out
    assert len(out.splitlines()) == 13


def title_style(info):
    title = conditions.conditions_title(info)
    return {str(span.style) for span in title.spans if title.plain[span.start:span.end] == info['verdict']}


def test_marginal_is_a_warn_word_poor_a_fail_word_and_good_is_plain():
    assert title_style({**INFO, 'verdict': 'marginal'}) == {theme.WARN}
    assert title_style({**INFO, 'verdict': 'poor'}) == {theme.BAD}
    assert not title_style(INFO) - {''}
    assert conditions.conditions_title({'verdict': 'unknown', 'reason': 'measuring 12/24 s'}).plain == 'measuring 12/24 s'


def test_unreliable_yaw_takes_the_sea_slot_and_turns_the_heading_tick_warn():
    ok, bad = conditions.sea_row(INFO), conditions.sea_row({**INFO, 'yaw_ok': False, 'yaw_note': '4°'})
    assert 'sea' in ok.plain and bad.plain == '! yaw 4°' and str(bad.style) == theme.WARN
    rows = conditions.conditions_panel(drive_view(yaw_ok=False))
    assert '! yaw' in plain(rows) and 'sea' not in plain(rows)
    ticks = {str(s.style) for row in conditions.rose(None, 30, yaw_ok=False) for s in row.spans}
    assert theme.WARN in ticks


def test_compass_bearing_from_odometry_yaw():
    snapshot = lambda yaw: {'sensors': {'odometry': {'values': {'yaw': yaw}}}}
    assert compass(snapshot(math.pi / 2)) == 0 and round(compass(snapshot(0.0))) == 90
    assert compass({'sensors': {}}) is None
