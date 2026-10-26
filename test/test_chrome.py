"""Header chips, health strip, agent banner, footer focus line and popup frame, as goldens.

Each golden holds the same scenes at 120/100/80/60 columns: plain Unicode, NO_COLOR (attributes
only, so the chips stay reverse video) and ASCII (a subprocess with MHS_ASCII). Regenerate with
``UPDATE_GOLDEN=1 pytest test/test_chrome.py`` and read the diff.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, theme
from test_dashboard import monitor, text, view

HERE = Path(__file__).parent
WIDTHS = (120, 100, 80, 60)
DEPLOY = dict(ahead=5, behind=39, dirty=2, untracked=7)
HEALTH = dict(bad=2, warn=3, worst='compass innovation')
POPUP = dict(kind='choice', title='Authority', message='Driving AGENT', options=['Revoke', 'Close'],
             keys=['E', 'Esc'], selected=0, hint='E revoke selected  U release hold  Esc close')


def scenes(profile):
    return {
        'idle': view(profile, host='blastoise-odroid', deploy=dict(), health=dict(ok=41)),
        'running': view(profile, host='blastoise-odroid', armed=True, deploy=DEPLOY, health=HEALTH,
                        authority=('ours', 'CHARACTERIZE'), recording=False),
        'diverged': view(profile, host='blastoise-odroid', deploy=dict(diverged=True, code_newer=True),
                         authority=('human', 'RC')),
        'agent': view(profile, host='blastoise', armed=True, authority=('agent', 'AGENT'),
                      agent=' | '.join(['AGENT DRIVING', 'tune_mppi', 'tranquility pid 5120', '"MPPI vx_max sweep 3/8"', '4m12s'])),
        'review': view(profile, review=True),
    }


def dump(no_color=False):
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    out = []
    for width in WIDTHS:
        for name, page in scenes(profile).items():
            out.append(f'== {name} {width}')
            for part in (layout.header(snapshot, page, width), layout.footer(page, width)):
                if no_color:
                    console = Console(width=width, no_color=True, force_terminal=True,
                                      color_system='standard', file=__import__('io').StringIO())
                    console.print(part)
                    out.append(console.file.getvalue().replace('\x1b', '<ESC>').rstrip('\n'))
                else:
                    out.append(text(part, width, 3).rstrip('\n'))
    modal, panel_width = layout.popup(dict(POPUP), 100)
    out.append('== popup')
    out.append(text(modal, panel_width, 8).rstrip('\n'))
    return '\n'.join(out) + '\n'


def check(name, actual):
    golden = HERE / 'golden' / name
    if os.environ.get('UPDATE_GOLDEN'):
        golden.write_text(actual)
    assert actual == golden.read_text(), f'{name} differs; UPDATE_GOLDEN=1 to regenerate'


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_chrome_unicode_golden():
    check('chrome.txt', dump())


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_chrome_no_color_golden_keeps_chips_as_reverse_video():
    # NO_COLOR picks the mono tier when the theme loads, so render in a fresh interpreter.
    env = {**os.environ, 'NO_COLOR': '1'}
    env.pop('MHS_ASCII', None)
    code = (f'import sys; sys.path.insert(0, {str(HERE)!r}); import test_chrome; '
            'sys.stdout.write(test_chrome.dump(no_color=True))')
    actual = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True,
                            check=True, cwd=HERE.parent).stdout
    check('chrome_nocolor.txt', actual)
    assert ';7m ARMED ' in actual and '<ESC>[31' not in actual  # reverse (bold in mono) stays, no colour


def test_chrome_ascii_golden_is_pure_ascii_with_bracket_chips():
    env = {k: v for k, v in os.environ.items() if k != 'TERM'}
    code = f'import sys; sys.path.insert(0, {str(HERE)!r}); import test_chrome; sys.stdout.write(test_chrome.dump())'
    done = subprocess.run([sys.executable, '-c', code], env={**env, 'MHS_ASCII': '1'},
                          capture_output=True, text=True, check=True, cwd=HERE.parent)
    done.stdout.encode('ascii')
    assert '[ARMED]' in done.stdout and '[CHARACTERIZE]' in done.stdout
    check('chrome_ascii.txt', done.stdout)


@pytest.mark.parametrize('width', WIDTHS)
def test_agent_banner_keeps_its_keys_and_the_row_width(width):
    page = scenes(load_profile('boat'))['agent']
    row = text(layout.header(monitor().snapshot(), page, width), width, 3).splitlines()[1]
    assert len(row) <= width and row.rstrip().endswith('E revoke · Space stop')


@pytest.mark.parametrize('width', WIDTHS)
def test_header_row_never_exceeds_the_width_and_keeps_the_arm_chip(width):
    page = scenes(load_profile('boat'))['running']
    row = text(layout.header(monitor().snapshot(), page, width), width, 3).splitlines()[0]
    assert len(row) <= width and row.rstrip().endswith('ARMED')


@pytest.mark.parametrize('width', WIDTHS)
def test_an_agents_arm_is_marked_after_the_armed_chip_and_the_operators_is_not(width):
    """Who armed: the operator's ARMED stands alone; an agent's adds ◆ who and sim-arm in the agent hue."""
    profile = load_profile('boat')
    agent = view(profile, armed=True, authority=('agent', 'AGENT claude'), armed_by={'who': 'claude', 'sim': True})
    row = text(layout.header(monitor().snapshot(), agent, width), width, 3).splitlines()[0]
    mark = f'{theme.AGENT_GLYPH} claude sim-arm' if width >= 100 else f'{theme.AGENT_GLYPH} sim-arm'
    flat = ' '.join(row.split()).replace('[', '').replace(']', '')     # ASCII brackets its chips
    assert len(row) <= width and flat.endswith(f'ARMED {mark}')
    own = text(layout.header(monitor().snapshot(), view(profile, armed=True), width), width, 3).splitlines()[0]
    assert own.rstrip().rstrip(']').endswith('ARMED') and f' {theme.AGENT_GLYPH} ' not in own


def test_the_agent_arm_mark_wears_the_agent_hue_not_the_armed_chips():
    mark = layout.chrome.armed_by({'who': 'claude', 'sim': False})
    assert mark.plain == f'{theme.AGENT_GLYPH} claude'
    assert str(mark.style) == f"bold {theme.PALETTE.style('auth.agent')}".strip()
