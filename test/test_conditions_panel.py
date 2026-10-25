import pytest

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import conditions

ARROWS = set(conditions.ARROWS + conditions.ARROWS_ASCII)


def plain(rows):
    return [row.plain for row in rows]


def changed(base, rows):
    """Cells of ``rows`` that differ from ``base``."""
    base, rows = plain(base), plain(rows)
    return [(r, c) for r, row in enumerate(rows) for c, ch in enumerate(row) if ch != base[r][c]]


def styles(rows, chars=None):
    return {str(span.style) for row in rows for span in row.spans
            if chars is None or set(row.plain[span.start:span.end]) & set(chars)}


@pytest.mark.skipif(theme.ASCII, reason='the Unicode rose')
def test_rose_is_9_by_21_north_up_with_the_hull_at_the_centre():
    rows = plain(conditions.rose(heading=0))
    assert len(rows) == 9 and all(len(row) == 21 for row in rows)
    assert rows[0][10] == 'N' and rows[8][10] == 'S' and rows[4][2] == 'W' and rows[4][19] == 'E'
    assert rows[4][9:12] == '⢸⣿⣿'         # the filled hull, not a cross
    assert plain(conditions.rose(heading=0)) != plain(conditions.rose(heading=90))   # it turns


@pytest.mark.skipif(theme.ASCII, reason='the Unicode rose')
def test_unknown_heading_is_bow_up_and_hides_the_world_drift():
    rows = plain(conditions.rose((0.15, 0.0), None))
    assert rows[0][9:13] == '↑bow' and not {'E', 'S', 'W'} & set(''.join(rows))
    assert rows == plain(conditions.rose(None, None))


def test_drift_stays_east_while_the_heading_rotates():
    for heading in (0, 90, 200):
        cells = changed(conditions.rose(None, heading), conditions.rose((0.15, 0.0), heading))
        assert cells and {r for r, _ in cells} == {4} and min(c for _, c in cells) > 11


def test_long_drift_is_clipped_to_the_ring_and_stale_drift_is_dim():
    assert plain(conditions.rose((0.0, 2.0), 90)) == plain(conditions.rose((0.0, 0.3), 90))
    assert min(r for r, _ in changed(conditions.rose(None, 90), conditions.rose((0.0, 2.0), 90))) <= 1
    assert min(r for r, _ in changed(conditions.rose(None, 90), conditions.rose((0.0, 0.1), 90))) >= 2
    fresh, old = conditions.rose((0.1, 0.0), 0), conditions.rose((0.1, 0.0), 0, age=61)
    assert f'dim {theme.MUTED}' in styles(old) and f'dim {theme.MUTED}' not in styles(fresh)


def test_unreliable_yaw_turns_the_hull_warn():
    ok, bad = conditions.rose(None, 45), conditions.rose(None, 45, yaw_ok=False)
    assert plain(ok) == plain(bad)
    assert [row.spans for row in ok] != [row.spans for row in bad]
    assert not theme.WARN or (theme.WARN not in styles(ok) and theme.WARN in styles(bad))   # mono: plain


@pytest.mark.parametrize('heading', (None, 0, 30, 135, 247))
@pytest.mark.parametrize('command', ((0.1, 0, 0), (0.25, 0, 0), (0.5, 0, 0), (0.6, 0.0, 0.2), (0.5, 0.29, 0),
                                     (0, -0.5, 0), (-0.4, 0.4, -0.5), (2.0, 0, 0)))
def test_exactly_one_arrowhead_at_any_size(command, heading):
    """The old glyph copied the arrow above 40 % of amplitude; now there is one, in the driver's hue."""
    rows = conditions.rose(heading=heading, command=command, amplitude=0.5, hue='magenta')[1:]   # row 0: ↑bow
    heads = [ch for row in plain(rows) for ch in row if ch in ARROWS]
    assert len(heads) == 1 and styles(rows, heads) == {'magenta'}


@pytest.mark.skipif(theme.ASCII, reason='the Unicode arrows')
@pytest.mark.parametrize('command,heading,arrow', (((0.5, 0, 0), 0, '↑'), ((0.5, 0, 0), 90, '→'),
                                                   ((0, 0.5, 0), 0, '←'), ((0, 0.5, 0), None, '←'),
                                                   ((-0.5, 0, 0), 180, '↑'), ((0.5, -0.5, 0), 0, '↗')))
def test_the_arrow_is_the_command_turned_into_the_world_by_the_heading(command, heading, arrow):
    rows = plain(conditions.rose(heading=heading, command=command, amplitude=0.5))
    assert arrow in ''.join(rows)


def test_a_bigger_command_draws_a_longer_line():
    def reach(surge):
        return min(r for r, _ in changed(conditions.rose(heading=0), conditions.rose(heading=0, command=(surge, 0, 0))))
    assert reach(0.1) > reach(0.5) >= 1


def test_yaw_is_an_arc_on_the_turning_side_without_an_arrowhead():
    base = conditions.rose(heading=0)
    ccw = changed(base, conditions.rose(heading=0, command=(0, 0, 0.3), amplitude=0.5))
    cw = changed(base, conditions.rose(heading=0, command=(0, 0, -0.3), amplitude=0.5))
    assert max(c for _, c in ccw) <= 10 < max(c for _, c in cw) and min(c for _, c in ccw) < 10
    assert not ARROWS & set(''.join(plain(conditions.rose(heading=0, command=(0, 0, 0.3)))))


@pytest.mark.skipif(theme.ASCII, reason='the Unicode bars')
def test_bars_fill_in_eighths_grow_port_left_and_mark_a_clip():
    assert conditions.bar(0.6, 1.0, 6, '').plain == '      │███▋  '
    assert conditions.bar(0.0, 1.0, 6, '').plain == '      │      '
    left = conditions.bar(0.2, 1.0, 6, 'cyan', grows_left=True)
    assert left.plain == '    ▊█│      '       # 1.2 cells; the partial is a reversed left fill
    assert any('reverse' in str(s.style) for s in left.spans)
    assert conditions.bar(-0.5, 1.0, 6, '', grows_left=True).plain == '      │███   '
    assert conditions.bar(1.4, 1.0, 6, '').plain == '      │█████▶'
    assert conditions.bar(-1.4, 1.0, 2, '').plain == '◀█│  '


def test_driver_names_who_and_how_in_the_authority_hue():
    view = dict(authority=('human', 'KEYS'), chords=True)
    assert conditions.driver(view) == ('keyboard' + theme.MIDDOT + 'chords', theme.auth_style('human'))
    assert conditions.driver({'authority': ('nav', 'NAV')}) == ('nav', theme.auth_style('nav'))
    assert conditions.driver({'authority': ('ours', 'C=I'), 'command_mode': 'proc · C=I'})[0] == 'proc · C=I'
    assert conditions.driver({'chords': False})[1] == theme.MUTED


def test_command_column_labels_each_axis_and_mutes_zero():
    view = dict(authority=('human', 'KEYS'), chords=True, command=(0.6, 0.0, 0.2), amplitude=1.0)
    rows = conditions.command_column(view, {'heading': 30.0}, 15)
    assert len(rows) == 9 and all(len(row) <= 15 for row in rows)
    assert rows[2].plain == 'surge      +60%' and rows[4].plain.rstrip().endswith('0')
    assert rows[6].plain.startswith('yaw') and rows[6].plain.endswith('+20%')
    assert str(rows[4].spans[-1].style) == theme.MUTED
    assert rows[8].plain == theme.pick('hdg 030° NNE', 'hdg 030 NNE')
    narrow = conditions.command_column(view, {}, 14)
    assert all(len(row) <= 14 for row in narrow) and narrow[0].plain.startswith('keys')


def test_heading_row_has_its_cardinal_sigma_and_yaw_trust():
    assert conditions.heading_row({'heading': 247.0}).plain == theme.pick('hdg 247° WSW', 'hdg 247 WSW')
    sigma = conditions.heading_row({'heading': 247.0, 'heading_sd': 4})
    assert sigma.plain == theme.pick('247° WSW ±4°', '247 WSW +-4')
    bad = conditions.heading_row({'heading': 5.0, 'yaw_ok': False})
    assert bad.plain.startswith('! hdg 005') and str(bad.style) == theme.WARN
    assert conditions.heading_row({}).plain == f'hdg {theme.DASH}'


def test_label_names_the_mode_with_the_tmux_caveat():
    assert conditions.mode_label(True, tmux=True) == 'chords'
    assert conditions.mode_label(False, tmux=True) == 'single key (tmux)'
    assert conditions.mode_label(False, tmux=False) == 'single key'
    assert conditions.mode_label(False, mode='rc · mux') == 'rc · mux'
