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


@pytest.mark.parametrize('width,height', ((120, 30), (80, 24)))
def test_no_object_glyphs_in_the_labels_tab(width, height):
    from test_labels_panel import model, text
    from mhseals_testing.ui.panels import labels
    for cursor in (0, 4):
        page = text(labels.render(model(cursor=cursor), width, height), width, height)
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


# --- every tab page byte for byte (the layout/, fusion/ and inspect splits must not move a cell) ----------
PAGE_SIZES = ((120, 30), (80, 24))


def tab_page(tab, width, height):
    """The whole page of ``tab`` in 16 colours, with the warn-health snapshot of ``pages``."""
    from mhseals_testing.ui import layout
    profile = load_profile('boat')
    counts = summary(engine(40, warn=('compass innovation',)))
    snapshot = {**monitor().snapshot(profile.required_tf), 'health': counts}
    state = {**view(profile, armed=True, health=counts), 'tab': tab, 'focus': layout.DEFAULT_FOCUS[tab]}
    if tab == 'Labels':
        from test_labels_panel import model
        state['labels'] = model()
    if tab == 'Fusion':   # the panel's drawings in braille text, from a fresh text cache
        import test_fusion_panel as fusion_test
        from mhseals_testing.ui import imaging
        from mhseals_testing.ui.panels import fusion
        imaging.use('text')
        fusion._TEXT.clear()
        state.update(fusion=fusion_test.model(), fusion_facts=fusion_test.facts())
    console = Console(file=io.StringIO(), width=width, height=height, color_system='standard', force_terminal=True)
    console.print(render(snapshot, state, width, height))
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
@pytest.mark.parametrize('width,height', PAGE_SIZES)
@pytest.mark.parametrize('tab', ('Drive', 'Correction', 'Map', 'Bags', 'Labels', 'Fusion'))
def test_every_tab_page_matches_its_golden(tab, width, height, request):
    from pathlib import Path
    golden = Path(__file__).parent / 'golden' / 'layout' / f'page_{tab.lower()}_{width}x{height}.ansi'
    actual = tab_page(tab, width, height)
    if request.config.getoption('--update-goldens'):
        golden.write_text(actual)
    assert actual == golden.read_text(), f'{golden.name} differs; --update-goldens to regenerate and review'
