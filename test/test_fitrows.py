import os
import subprocess
import sys
from pathlib import Path

import pytest

from mhseals_testing.ui import fitrows as f
from mhseals_testing.ui import theme

ROOT = str(Path(__file__).parent.parent)
unicode_only = pytest.mark.skipif(theme.ASCII, reason='unicode goldens; ASCII mode is covered by a subprocess test')

DIRS = ['surge+', 'surge-', 'sway+', 'sway-']
FITS = {'surge+': {'verdict': 'good', 'ceiling': 91}, 'sway-': {'verdict': 'marg', 'ceiling': 66}}
NOW = {'surge+': {'ceiling': 99}, 'surge-': {'ceiling': 98, 'hdg': True}, 'sway+': {'ceiling': 93, 'up': True}}


@unicode_only
def test_fit_under_golden():
    assert f.fit_under(DIRS, FITS) == '  fit under     good 91  —        —        marg 66'


@unicode_only
def test_ceiling_now_golden():
    assert f.ceiling_now(DIRS, NOW) == '  ceiling now   99       98 hdg   93↑      —'


@unicode_only
def test_header_marks():
    states = {'surge-': 'invalid', 'sway+': 'verify'}
    assert f.header(DIRS, states, flags={'sway-'}) == ' ' * 16 + 'surge+   surge- × sway+ ?  sway- ↑'


def test_better_needs_gain_and_low_trust():
    assert f.better(66, 93, trust=71)
    assert not f.better(66, 93, trust=95)
    assert not f.better(90, 95)
    assert not f.better(None, 95)


@unicode_only
def test_legend_and_consequence_golden():
    assert f.legend() == '▪ hardware  △ calibration  (run|site|build)  no glyph: setting'
    assert f.consequence('c0150', [('C±', 'invalid'), ('curves', 'verify')]) == \
        '▪ change → △ C± · curves × (fitted under c0150)'


def test_consequence_verify_only_and_ascii_subprocess():
    assert f.consequence('c1', [('curves', 'verify')]).endswith('curves ? (fitted under c1)') or theme.ASCII
    code = ("from mhseals_testing.ui import fitrows as f\n"
            "print(f.consequence('c1', [('C+-', 'verify'), ('curves', 'invalid')]))")
    env = {k: v for k, v in os.environ.items() if k not in ('MHS_ASCII', 'TERM')}
    env['MHS_ASCII'] = '1'
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env, cwd=ROOT).stdout.strip()
    assert out.isascii()
    assert out == 'H change -> C C+- . curves x (fitted under c1)'
