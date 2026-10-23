from types import SimpleNamespace

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import conditions


def plain(rows):
    return [row.plain for row in rows]


def test_rose_is_9_by_21_with_north_up_and_a_centre_mark():
    rows = plain(conditions.rose())
    assert len(rows) == 9 and all(len(row) == 21 for row in rows)
    assert rows[0][10] == 'N' and rows[8][10] == 'S' and rows[4][2] == 'W' and rows[4][19] == 'E'
    assert rows[4][10] == '┼'


def arrow_cells(*args, **kw):
    """Cells that differ from the bare rose: where the drift arrow is drawn."""
    bare, rows = plain(conditions.rose()), plain(conditions.rose(*args, **kw))
    return [(r, c) for r, row in enumerate(rows) for c, ch in enumerate(row) if ch != bare[r][c]]


def test_drift_arrow_stays_fixed_while_the_heading_rotates():
    drift = (0.15, 0.0)  # east
    def arrow(heading):
        rows = plain(conditions.rose(drift, heading))
        return [(r, c) for r, row in enumerate(rows) for c, ch in enumerate(row)
                if ch != plain(conditions.rose(None, heading))[r][c]]

    assert arrow(None) == arrow(0) == arrow(90) == arrow(200)
    cells = arrow(None)
    assert {r for r, _ in cells} == {4} and min(c for _, c in cells) > 10   # due east of the centre
    assert not any(g in ''.join(plain(conditions.rose(drift))) for g in '●○◆')
    assert plain(conditions.rose(drift, 0)) != plain(conditions.rose(drift, 90))  # the tick moves


def test_tick_alone_changes_with_heading_and_follows_yaw_reliability():
    ok, bad = conditions.rose(None, 45), conditions.rose(None, 45, yaw_ok=False)
    assert plain(ok) == plain(bad)
    assert {str(span.style) for row in ok for span in row.spans} >= {theme.ACCENT}
    assert {str(span.style) for row in bad for span in row.spans} >= {theme.WARN}


def test_long_drift_is_clipped_to_the_ring_and_stale_drift_is_dim():
    assert plain(conditions.rose((0.0, 2.0), None)) == plain(conditions.rose((0.0, 0.3), None))
    assert arrow_cells((0.0, 2.0))[0][0] == 0           # the line reaches the ring at the top
    assert not any(g in ''.join(plain(conditions.rose((0.0, 2.0)))) for g in '●○◆')
    assert arrow_cells((0.0, 0.1))[0][0] > 0            # a short one does not
    fresh, old = conditions.rose((0.1, 0.0), None), conditions.rose((0.1, 0.0), None, age=61)
    assert [r.spans for r in fresh] != [r.spans for r in old]


def test_arrow_sector_and_doubling_wear_the_authority_hue():
    forward = conditions.command_glyph((0.25, 0, 0), 0.5, owner='nav')
    assert forward[3].plain[6] == '↑' and forward[2].plain[6] == '↑'
    left = conditions.command_glyph((0, 0.5, 0), 0.5)
    assert left[3].plain[5:7] == '←←'
    hue = theme.AUTHORITY_HUES['nav']
    assert any(str(s.style) == hue for s in forward[3].spans)
    assert theme.AUTHORITY_HUES['agent'] != theme.AUTHORITY_HUES['nav']


def test_doubled_arrow_follows_the_sector_for_an_off_axis_command():
    rows = conditions.command_glyph((0.5, 0.29, 0), 0.5)   # about 30 degrees: the sector is up-left
    assert rows[3].plain[6] == '↖' and rows[2].plain[5] == '↖' and rows[2].plain[6] == ' '


def test_hull_has_no_bow_marker():
    assert '▲' not in ''.join(row.plain for row in conditions.command_glyph((0, 0, 0), 0.5))


def test_yaw_marker_sits_beside_the_hull_and_is_bold_above_half_amplitude():
    ccw = conditions.command_glyph((0, 0, 0.4), 0.5)
    assert ccw[3].plain[0] == '↩'
    assert 'bold' in str(ccw[3].spans[0].style)
    cw = conditions.command_glyph((0, 0, -0.1), 0.5)
    assert cw[3].plain[12] == '↪' and 'bold' not in str(cw[3].spans[-2].style)


def test_label_names_the_mode_with_the_tmux_caveat():
    assert conditions.mode_label(True, tmux=True) == 'chords'
    assert conditions.mode_label(False, tmux=True) == 'single key (tmux)'
    assert conditions.mode_label(False, tmux=False) == 'single key'
    assert conditions.mode_label(False, mode='rc · mux') == 'rc · mux'
    rows = conditions.command_glyph((0.25, 0, 0), 0.5, tmux=True)
    assert rows[-2].plain == '+50% 0 0' and rows[-1].plain == 'single key (tmux)'


def test_command_view_reads_chords_and_falls_back_to_event_mode():
    new = SimpleNamespace(chords=True, command=(0.1, 0, 0))
    assert conditions.command_view(new) == ((0.1, 0, 0), True)
    old = SimpleNamespace(event_mode=True)
    assert conditions.command_view(old) == ((0.0, 0.0, 0.0), True)
