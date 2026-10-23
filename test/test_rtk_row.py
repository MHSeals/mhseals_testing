"""RTK row: 12 states x 3 width tiers x Unicode/ASCII goldens (gps.md 7.4), plus colour and math."""

from pathlib import Path

import pytest

from mhseals_testing.ui.rtk_row import RtkView, rtk_row, survey_fraction

GOLDEN = Path(__file__).parent / 'fixtures' / 'rtk_row.txt'
V = RtkView
STREAM = dict(base='streaming', age=0.4)
STATES = [                       # the twelve rows of the mockups, in order
    V(fix='3D'),
    V(base='identifying', fix='3D', tick=2),
    V(base='surveying', fix='3D', survey=0.4, survey_left=72, survey_acc=1.9, survey_target=2.0),
    V(base='surveying', fix='3D', survey=0.83, survey_limit='accuracy', survey_left=10,
      survey_acc=2.4, survey_target=2.0),
    V(**STREAM, fix='DGPS'),
    V(**STREAM, fix='FLOAT', baseline=12.34),
    V(**STREAM, fix='FLOAT', baseline=12.34, converge_warn=True),
    V(**STREAM, fix='FIXED', baseline=12.34),
    V(base='streaming', age=3.1, drops=14, fix='FLOAT', baseline=12.34),
    V(base='streaming', age=14.0, fix='FIXED', baseline=12.34),
    V(**STREAM, fix='FLOAT', baseline=12.34, pubs=2),
    V(**STREAM, fix='none'),
]


def goldens():
    """(width, ascii, [rows]) per block of the fixture; rows are the exact row text."""
    block = None
    for line in GOLDEN.read_text(encoding='utf-8').splitlines():
        if line.startswith('=='):
            _, width, mode = line.split()
            block = (int(width), mode == 'ascii', [])
            yield block
        elif line:
            block[2].append(line)


@pytest.mark.parametrize('width,ascii,rows', list(goldens()), ids=lambda v: str(v) if isinstance(v, (int, bool)) else '')
def test_rows_match_the_goldens(width, ascii, rows):
    assert len(rows) == len(STATES)
    for number, (view, want) in enumerate(zip(STATES, rows), 1):
        got = rtk_row(V(**{**view.__dict__, 'tick': 0 if ascii else view.tick}), width, ascii).plain
        assert got == want, f'state {number}: {got!r}'
        assert len(got) == width


def test_rows_are_pure_ascii_in_ascii_mode_and_nothing_below_26_cols():
    assert all(rtk_row(view, 42, True).plain.isascii() for view in STATES)
    assert rtk_row(STATES[0], 25, False).plain == ''


def style_of(view, text, width=42):
    """Style covering ``text`` in the Unicode row (they never straddle spans)."""
    row = rtk_row(view, width, False)
    at = row.plain.index(text)
    return str(row.get_style_at_offset(__import__('rich.console').console.Console(), at))


def test_colours_follow_the_state():
    assert style_of(STATES[7], '✓') == 'green' and style_of(STATES[7], 'FIXED') == 'bold green'
    assert style_of(STATES[8], '╌') == 'yellow' and style_of(STATES[8], '3.1s') == 'yellow'
    assert style_of(STATES[9], '14s') == 'red' and style_of(STATES[9], '✗') == 'red'
    assert style_of(STATES[5], 'FLOAT') == 'none' and style_of(STATES[5], '━') == 'none'   # converging is plain
    assert style_of(STATES[6], 'FLOAT') == 'yellow'                                       # stuck is not


def test_survey_never_shows_complete_before_valid():
    assert survey_fraction(200, 120, 1.0, 5.0, 2.0, valid=False) == 0.99
    assert survey_fraction(200, 120, 1.0, 5.0, 2.0, valid=True) == 1.0
    assert survey_fraction(60, 120, 1.0, 5.0, 2.0, valid=True) == 0.5     # time-limited
    assert 0 < survey_fraction(200, 120, 5.0, 8.0, 2.0, valid=True) < 0.5  # accuracy-limited
