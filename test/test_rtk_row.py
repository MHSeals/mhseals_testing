"""RTK row: 12 states x 3 width tiers x Unicode/ASCII goldens (gps.md 7.4), plus colour and math."""

from pathlib import Path

import pytest
from rich.console import Console
from rich.style import Style

from mhseals_testing.ui import theme
from mhseals_testing.ui.rtk_row import RtkView, rtk_row, strip_item, survey_fraction

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
    return str(row.get_style_at_offset(Console(), at))


def hue(name, bold=False):
    """The rendered style string of a theme style name (GOOD, WARN, BAD, ACCENT)."""
    return str(Style.parse(f"{'bold ' if bold else ''}{getattr(theme, name)}"))


def test_colours_follow_the_state():
    assert style_of(STATES[7], '✓') == hue('GOOD') and style_of(STATES[7], 'FIXED') == 'bold'   # healthy: no hue
    assert style_of(STATES[8], '╌') == hue('WARN') and style_of(STATES[8], '3.1s') == hue('WARN')
    assert style_of(STATES[9], '14s') == hue('BAD') and style_of(STATES[9], '✗') == hue('BAD')
    assert style_of(STATES[5], 'FLOAT') == 'none' and style_of(STATES[5], '━') == 'none'   # converging is plain
    assert style_of(STATES[6], 'FLOAT') == hue('WARN')                                     # stuck is not


def test_survey_never_shows_complete_before_valid():
    assert survey_fraction(200, 120, 1.0, 5.0, 2.0, valid=False) == 0.99
    assert survey_fraction(200, 120, 1.0, 5.0, 2.0, valid=True) == 1.0
    assert survey_fraction(60, 120, 1.0, 5.0, 2.0, valid=True) == 0.5     # time-limited
    assert 0 < survey_fraction(200, 120, 5.0, 8.0, 2.0, valid=True) < 0.5  # accuracy-limited


@pytest.mark.parametrize('ascii', [False, True], ids=['unicode', 'ascii'])
def test_every_state_is_exactly_the_width_at_every_width(ascii):
    for width in range(26, 61):
        for number, view in enumerate(STATES, 1):
            assert len(rtk_row(view, width, ascii).plain) == width, (number, width)


def plain(view, width=32, ascii=False):
    return rtk_row(view, width, ascii).plain


def test_external_corrections_show_the_source_and_judge_by_the_fcu_fix():
    """Mission Planner injects over the radio: no base here, no sender age; the FCU fix decides."""
    fixed, floating = V(source='external', fix='FIXED'), V(source='external', fix='FLOAT')
    assert 'RTK ext ' in plain(fixed, 28) and 'RTK ext ' in plain(fixed, 32) and 'RTK external' in plain(fixed, 42)
    assert style_of(fixed, '✓') == hue('GOOD') and style_of(fixed, 'external') == hue('ACCENT')
    assert '◌' in plain(floating) and style_of(V(source='external', fix='FLOAT', converge_warn=True), '!') == hue('WARN')
    assert strip_item(fixed) == ('LIVE', 'rtk ext FIXED') and strip_item(V(source='external', fix='3D'))[0] == 'CHECKING'
    assert 'no base' not in plain(V(source='external', fix='3D'))


def test_no_correction_source_is_quiet_and_never_a_failure():
    view = V(source='none', fix='3D')
    assert 'no corr' in plain(view) and plain(view).lstrip().startswith('·') and 'off' in plain(view, 28)
    assert strip_item(view) == ('LIVE', 'rtk off')


def test_our_base_running_beside_another_source_is_a_failure():
    view = V(source='external', base='streaming', age=0.4, fix='FLOAT', pubs=2)
    assert '✗' in plain(view) and '2 pubs' in plain(view) and strip_item(view)[0] == 'FAIL'


@pytest.mark.parametrize('source', ['external', 'none'])
def test_other_sources_fill_the_width_exactly(source):
    for width in range(26, 61):
        for ascii in (False, True):
            row = rtk_row(V(source=source, fix='FLOAT'), width, ascii)
            assert row.cell_len == width and (not ascii or row.plain.isascii())
