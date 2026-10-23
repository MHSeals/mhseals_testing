"""Goldens for the natural config diff at 120, 80 and 60 columns (set UPDATE_GOLDENS=1 to rewrite them)."""

import io
import os
from pathlib import Path

from rich.console import Console
import pytest

from mhseals_testing.history.render import auto_name, diff, render

GOLDEN = Path(__file__).parent / 'fixtures' / 'config_diff'
PINS = {1: (7, '', '', ''), 2: (12, '', '', ''), 3: (15, '', '', ''), 4: (33, '', '', '')}
BEFORE = {'channel_map': [1, 2, 3, 4], 'reversed': [False] * 4, 'command_timeout': 0.5, 'output_backend': 'odroid',
          'decoupling': [1, 0, 0, 0, 1, 0, -0.0375, 0.1, 1]}
AFTER = {**BEFORE, 'channel_map': [3, 2, 3, 4], 'reversed': [False, False, True, False], 'command_timeout': 0.3,
         'output_backend': 'topic', 'decoupling': [1, 0, 0, 0, 1, 0, 0, 0.08, 1], 'note_key': 'x'}
NAV_BEFORE = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.4, 'wz_max': 1.2}}}}
NAV_AFTER = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.38, 'wz_max': 0.9}}}}
CASES = {
    'hardware': lambda: diff('hardware', {'effective': BEFORE}, {'effective': AFTER}, PINS),
    'nav2': lambda: diff('nav2', NAV_BEFORE, NAV_AFTER),
}


def check(name, text):
    path = GOLDEN / name
    if os.environ.get('UPDATE_GOLDENS'):
        path.write_text(text)
    assert text == path.read_text()


@pytest.mark.parametrize('ascii_', [False, True], ids=['unicode', 'ascii'])
@pytest.mark.parametrize('width', [120, 80, 60])
@pytest.mark.parametrize('case', CASES)
def test_golden(case, width, ascii_):
    text = render(CASES[case](), width, meta='hardware · tui:pins · c0142', ascii_=ascii_).plain + '\n'
    assert max(len(line) for line in text.splitlines()) <= width
    if ascii_:
        assert text.isascii()
    check(f'{case}_{width}{"_ascii" if ascii_ else ""}.txt', text)


def paint(text, **console):
    out = io.StringIO()
    Console(width=100, force_terminal=True, color_system='standard', file=out, **console).print(text)
    return out.getvalue()


def test_no_color_keeps_the_marks_and_drops_the_colors():
    text = render(CASES['hardware'](), 80, ascii_=False)
    assert '33m' in paint(text)                                       # changed cells are yellow
    plain = paint(text, no_color=True)
    assert '33m' not in plain and '•0.000' in plain and '+ note_key' in plain


def test_precision_widens_only_to_show_the_difference():
    [row] = diff('hardware', {'effective': {'command_timeout': 0.5}}, {'effective': {'command_timeout': 0.5004}})
    assert render([row], 80, ascii_=True).plain == 'Timing\n  Command timeout  0.5 -> 0.5004 s'
    assert 'no differences' == render([], 80).plain


def test_auto_names():
    rows = CASES['hardware']()
    assert auto_name({'source': 'tui:pins'}, rows, ascii_=False) == 'FL output 1→3, RR reversed'
    assert auto_name({'source': 'external'}) == 'changed outside boat_test'
    assert auto_name({'source': 'cli:revert', 'revert_of': 'c0131'}, target={'name': 'dock baseline'},
                     ascii_=False) == '↶ revert to c0131 "dock baseline"'


def test_first_version_diffs_against_nothing():
    rows = diff('hardware', None, {'effective': {'command_timeout': 0.5}})
    assert [(r.kind, r.after) for r in rows] == [('added', 0.5)]
    assert diff('hardware', {'effective': {}}, {'effective': {'command_timeout': 0.5}})[0].kind == 'added'
    assert diff('nav2', None, NAV_AFTER)[0].kind == 'added'
    assert diff('nav2', None, None) == []


@pytest.mark.parametrize('width', [60, 80])
def test_matrix_with_a_null_side_renders_as_a_grid(width):
    ident = [1.0, 0, 0, 0, 1.0, 0, 0, -0.0123456, 1.0]
    for before, after in ((None, ident), (ident, None)):
        rows = diff('hardware', {'effective': {'decoupling_negative': before}},
                    {'effective': {'decoupling_negative': after}})
        assert [r.kind for r in rows] == ['matrix']
        lines = render(rows, width, ascii_=True).plain.splitlines()
        assert max(len(line) for line in lines) <= width
        assert any('none' in line for line in lines) and any('-0.012' in line for line in lines)


def test_short_row_is_not_wrapped_by_the_padding_for_a_long_one():
    rows = diff('hardware', {'effective': {'pwm_channels': [0, 0, 0, 0], 'output_backend': 'a' * 30}},
                {'effective': {'pwm_channels': [0, 1, 0, 0], 'output_backend': 'b' * 30}})
    lines = render(rows, 80, ascii_=True).plain.splitlines()
    assert all(len(line) <= 80 for line in lines)
    assert any('[0, 0, 0, 0] -> [0, 1, 0, 0]' in line and line.startswith('  ') and 'pwm' in line.lower() for line in lines), lines
