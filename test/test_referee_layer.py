"""Referee marks on the canvas, the strip at 120/80/48 columns and run comparison."""
import json
from types import SimpleNamespace

import pytest

from mhseals_testing.referee import compare
from mhseals_testing.ui import referee_strip
from mhseals_testing.ui.minimap import canvas, referee_layer


def mark(kind, points, state, text=''):
    return SimpleNamespace(kind=kind, points=points, state=state, text=text)


def scale(p):
    return p[0] * 2, p[1] * 4


@pytest.mark.parametrize('cols', [120, 80, 48])
def test_a_gate_is_a_segment_coloured_by_state_at_every_width(cols):
    c = canvas.Canvas(cols, 6)
    referee_layer.draw(c, [mark('gate', ((1, 1), (9, 1)), 'passed', '41.2 s')], scale, ascii_only=False)
    layers = {cell.fg for row in c.cells() for cell in row if cell.fg}
    assert layers == {'ref_passed'}
    assert any('41.2 s' in line for line in c.text())


def test_state_colours_differ_and_ascii_adds_a_glyph():
    def top(state, ascii_only):
        c = canvas.Canvas(10, 3)
        referee_layer.draw(c, [mark('mark', ((2, 1),), state)], scale, ascii_only)
        return c

    assert top('next', False).cells()[1][2].fg == 'ref_next'
    assert top('failed', False).cells()[1][2].fg == 'ref_failed'
    assert top('pending', True).cells()[1][2].text == '.'
    assert top('passed', True).cells()[1][2].text == '+'


def test_a_bay_is_an_outline_and_empty_marks_are_skipped():
    c = canvas.Canvas(10, 4)
    referee_layer.draw(c, [mark('bay', ((1, 1), (6, 1), (6, 3), (1, 3)), 'next'), mark('gate', (), 'next')], scale, False)
    assert c.text()[2][3] != canvas.BLANK and c.text()[2][1] != canvas.BLANK


CHALLENGES = [
    {'name': 'speed', 'verdict': None, 'text': '41.2 s', 'active': True, 'best': 39.8},
    {'name': 'gates', 'verdict': 'PASS', 'text': '2/2'},
    {'name': 'follow', 'verdict': 'FAIL', 'text': '3/4'},
]


def test_strip_fits_each_width_and_keeps_the_active_challenge():
    parts = referee_strip.segments('2025', CHALLENGES, contacts=0)
    full = referee_strip.fit(parts, 120).plain
    assert 'speed 41.2 s (best 39.8)' in full and 'gates 2/2' in full and 'contacts 0' in full
    for width in (120, 80, 48, 20):
        text = referee_strip.fit(parts, width).plain
        assert len(text) <= width
        if width >= 48:
            assert 'speed 41.2 s' in text and 'contacts 0' in text
    assert 'follow' not in referee_strip.fit(parts, 48).plain


def test_contacts_turn_the_strip_segment_red():
    parts = referee_strip.segments('2025', CHALLENGES, contacts=2)
    assert parts[-1] == ('contacts 2', 'FAIL')


def row(i, started, verdict, seconds, name='speed'):
    return {'id': i, 'started': started,
            'referee': {'challenges': {name: {'verdict': verdict, 'details': {'window_s': seconds}}}}}


def test_best_median_and_last_n_come_from_index_rows_with_deltas(tmp_path):
    index = [row('a', '1', 'PASS', 50.0), row('b', '2', 'FAIL', 20.0), row('c', '3', 'PASS', 44.0),
             row('d', '4', 'PASS', 41.0), row('e', '5', 'PENDING', None), row('x', '6', 'PASS', 9.0, 'dock')]
    path = tmp_path / 'INDEX.json'
    path.write_text(json.dumps({'sessions': index}))
    hist = compare.rows(compare.load(path), 'speed')
    assert [h[0] for h in hist] == ['a', 'b', 'c', 'd']
    got = compare.compare(hist, 40.0, n=2)
    assert got['best'] == 41.0 and got['median'] == 44.0 and got['last'] == [44.0, 41.0]
    assert got['delta_best'] == -1.0 and got['delta_last'] == -1.0
    assert got['failures'] == 0 and got['runs'] == 4
    assert compare.compare(hist, 40.0, n=3)['failures'] == 1


def test_no_history_or_a_broken_index_gives_empty_references(tmp_path):
    assert compare.load(tmp_path / 'missing.json') == []
    (tmp_path / 'INDEX.json').write_text('{')
    assert compare.load(tmp_path / 'INDEX.json') == []
    got = compare.compare([], 40.0)
    assert got['best'] is None and got['delta_best'] is None
