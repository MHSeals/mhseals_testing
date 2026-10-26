"""The status grammar (ui.md §1): glyphs mean one thing and carry the matching hue.

Object glyphs (● ○ and the buoy shapes) belong to the map canvas only, so no
dashboard cell, popup or row may show them; ``! `` is never red and ``✗`` never yellow.
"""

import io
import re

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import PANELS
from mhseals_testing.ui.layout import render
from test_dashboard import monitor, view
from test_ui_health import engine
from mhseals_testing.ui.feeds.health import summary

OBJECT_GLYPHS = '●○▲▴▼■▪⊠◉◆◇△⊕'
SGR = re.compile(r'\x1b\[([0-9;]*)m')
STATUS = {'✓': 'GOOD', '!': 'WARN', '✗': 'BAD', '◌': 'BUSY'}   # read at call time: conftest pins the theme


def hue(glyph):
    """The SGR colour codes the palette token for ``glyph`` renders as in 16 colours."""
    console = Console(file=io.StringIO(), color_system='standard', force_terminal=True)
    console.print(glyph, style=getattr(theme, STATUS[glyph]), end='')
    return {c for group in SGR.findall(console.file.getvalue()) for c in group.split(';')} - {'0', '1'}


def test_status_glyph_table():
    got = {state: theme.icon(state).plain for state in ('good', 'warn', 'bad', 'busy', 'off', 'unknown')}
    assert got == dict(good='✓', warn='!', bad='✗', busy='◌', off='·', unknown='·')
    assert theme.icon('LIVE').style == theme.GOOD and theme.icon('FAIL').style == theme.BAD
    assert theme.icon('WARN').style == theme.WARN


def pages(color):
    profile = load_profile('boat')
    for width, height in ((120, 40), (80, 24)):
        for focus in PANELS:
            if focus == 'motion' and width < 100:
                continue
            console = Console(file=io.StringIO(), width=width, height=height,
                              color_system='standard' if color else None, force_terminal=color)
            counts = summary(engine(40, warn=('compass innovation',)))   # the health row and strip counts
            snapshot = {**monitor().snapshot(profile.required_tf), 'health': counts}
            console.print(render(snapshot, view(profile, focus=focus, armed=True, health=counts), width, height))
            yield console.file.getvalue()


def test_no_object_glyphs_in_chrome():
    for page in pages(color=False):
        assert not set(page) & set(OBJECT_GLYPHS)


def test_status_glyph_hue_matches_meaning():
    seen = set()
    for page in pages(color=True):
        for match in re.finditer(r'((?:\x1b\[[0-9;]*m)*)([✓!✗◌])\x1b', page):
            codes = {c for group in SGR.findall(match.group(1)) for c in group.split(';')}
            glyph = match.group(2)
            assert hue(glyph) <= codes, (glyph, codes)
            seen.add(glyph)
    assert {'✓', '✗'} <= seen


@pytest.mark.parametrize('glyph', '●○')
def test_theme_exports_no_dot_constant(glyph):
    assert glyph not in ''.join(str(v) for v in vars(theme).values() if isinstance(v, str))
