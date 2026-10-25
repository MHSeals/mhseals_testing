import io
import math

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
@pytest.mark.parametrize('command', ((0.1, 0, 0), (0.5, 0, 0), (0.6, 0.0, 0.2), (0.5, 0.29, 0),
                                     (0, -0.5, 0), (-0.4, 0.4, -0.5), (2.0, 0, 0)))
def test_the_command_is_one_icon_in_the_drivers_hue(command, heading):
    """No shaft: braille dots in the hue, or one arrow glyph in ASCII; size never changes the icon."""
    rows = conditions.rose(heading=heading, command=command, amplitude=0.5, hue='magenta')[1:]   # row 0: ↑bow, spin
    heads = [ch for row in plain(rows) for ch in row if ch in ARROWS]
    if theme.ASCII:
        assert len(heads) == 1 and styles(rows, heads) == {'magenta'}
    else:
        assert not heads and 'magenta' in styles(rows)
    small = conditions.rose(heading=heading, command=tuple(v / 4 for v in command[:2]) + (0,), amplitude=0.5)
    assert plain(small) == plain(conditions.rose(heading=heading, command=command[:2] + (0,), amplitude=0.5))


def _letters():
    cx, cy = conditions.CENTRE
    return {((cx + round(dx)) // 2, (cy + round(dy)) // 4)
            for dx, dy in (conditions._polar(t, conditions.RADIUS) for t in (0, 90, 180, 270))}


@pytest.mark.parametrize('k', range(16))
def test_every_icon_stays_inside_the_ring_clear_of_letters_and_hull(k):
    """All 16 bearings: each dot inside the canvas and the tick ring, none in N/E/S/W or the corner."""
    shape, (ox, oy) = conditions.command_icon(k)
    cx, cy = conditions.CENTRE
    cells = {((cx + ox + x) // 2, (cy + oy + y) // 4) for x, y in shape}
    assert all(0 < col < conditions.ROSE_COLS - 1 and 0 < row < conditions.ROSE_ROWS - 1 for col, row in cells)
    assert all(math.hypot(ox + x + 0.5, oy + y + 0.5) <= conditions.RADIUS - 3 for x, y in shape)
    assert not cells & _letters() and (conditions.ROSE_COLS - 2, 0) not in cells
    hull = {(x, y) for x, y, _ in conditions.hull.mask(90.0 - k * 22.5, minimum=conditions.HULL_DOTS)}
    assert not hull & {(ox + x, oy + y) for x, y in shape}


@pytest.mark.parametrize('k', range(16))
def test_icon_is_whole_in_the_rendered_rose_and_panel(k):
    """The icon's dots all reach the screen at the narrowest full panel; ASCII puts its glyph inside too."""
    rel = math.radians(k * 22.5)
    rows = conditions.rose(heading=0, command=(0.5 * math.cos(rel), -0.5 * math.sin(rel), 0), amplitude=0.5, hue='magenta')
    from rich.console import Console
    view = {'command': (0.5 * math.cos(rel), -0.5 * math.sin(rel), 0), 'amplitude': 0.5}
    console = Console(width=34, file=io.StringIO(), color_system=None)   # the body at 90 columns, the narrowest full form
    lines = console.render_lines(conditions._Body(view, {'heading': 0}, False), pad=False)
    shown = [''.join(seg.text for seg in line)[:conditions.ROSE_COLS] for line in lines[:conditions.ROSE_ROWS]]
    assert shown == plain(rows)
    if not theme.ASCII:
        shape, (ox, oy) = conditions.command_icon(k)
        cx, cy = conditions.CENTRE
        for x, y in shape:
            col, row = (cx + ox + x) // 2, (cy + oy + y) // 4
            bit = conditions.BITS[((cx + ox + x) % 2, (cy + oy + y) % 4)]
            assert (ord(rows[row].plain[col]) - 0x2800) & bit


def test_icons_of_one_kind_sit_at_one_radius():
    radius = {k: math.hypot(*conditions.command_icon(k)[1]) for k in range(16)}
    for kind in ((0, 4, 8, 12), (2, 6, 10, 14), (1, 3, 5, 7, 9, 11, 13, 15)):
        assert max(radius[k] for k in kind) - min(radius[k] for k in kind) < 1.5


def test_yaw_is_a_spin_glyph_in_the_corner_dim_when_small():
    base = plain(conditions.rose(heading=0))
    ccw = conditions.rose(heading=0, command=(0, 0, 0.4), amplitude=0.5, hue='magenta')
    cw = conditions.rose(heading=0, command=(0, 0, -0.1), amplitude=0.5, hue='magenta')
    assert plain(ccw)[1:] == base[1:] == plain(cw)[1:]          # only the top row changes
    if theme.ASCII:
        assert plain(ccw)[0].endswith('ccw⠀'.replace('⠀', ' ')) or 'ccw' in plain(ccw)[0]
        assert ' cw' in plain(cw)[0] or 'cw' in plain(cw)[0]
    else:
        assert plain(ccw)[0][-2] == '↺' and plain(cw)[0][-2] == '↻'
        assert 'bold magenta' in styles(ccw, '↺') and 'dim magenta' in styles(cw, '↻')
    assert plain(conditions.rose(heading=0, command=(0, 0, 0))) == base


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


def test_speed_gauge_lights_cells_calm_to_hot_and_stays_off_the_status_hues():
    half, fast = conditions.speed_gauge(0.5), conditions.speed_gauge(0.75)
    on, off = theme.pick('▰', '#'), theme.pick('▱', '-')
    assert half.plain.count(on) == half.plain.count(off) == 5 and fast.plain.count(on) == 8
    assert fast.plain.endswith(' 75%')
    if not theme.ASCII:
        used = {str(span.style) for span in fast.spans} - {theme.MUTED}
        assert len(used) >= 3                                   # a gradient, not one colour
        assert not used & {theme.GOOD, theme.WARN, theme.BAD}   # fast is not a warning


def test_command_column_shows_the_speed_gauge_under_the_driver():
    view = dict(command=(0.0, 0.0, 0.0), speed=0.75, chords=True)
    rows = conditions.command_column(view, {}, 30)
    assert len(rows) == 9 and rows[1].plain.endswith('75%')
    assert conditions.command_column(dict(view, speed=None), {}, 30)[1].plain == ''


def test_label_names_the_mode_with_the_tmux_caveat():
    assert conditions.mode_label(True, tmux=True) == 'chords'
    assert conditions.mode_label(False, tmux=True) == 'single key (tmux)'
    assert conditions.mode_label(False, tmux=False) == 'single key'
    assert conditions.mode_label(False, mode='rc · mux') == 'rc · mux'
