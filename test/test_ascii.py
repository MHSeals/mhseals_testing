"""ASCII mode (MHS_ASCII or TERM=dumb): the whole page is pure ASCII at any size."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

TESTS = Path(__file__).parent

PAGE = """
import sys
sys.path.insert(0, sys.argv[1])
from test_dashboard import monitor, text, view
from mhseals_testing.ui import theme
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.dashboard import render
profile = load_profile('boat')
for width, height in ((160, 50), (120, 40), (100, 30), (80, 24), (40, 12), (10, 4)):
    snapshot = monitor().snapshot(profile.required_tf)
    sys.stdout.write(text(render(snapshot, view(profile, armed=True, test_pulses=[1600, 1500, 1400, 1500],
                                      pwm_readback=[theme.DASH] * 4),
                                 width, height), width, height))
sys.stdout.write(str(theme.ASCII))
"""


def run(**env):
    base = {key: value for key, value in os.environ.items() if key not in ('MHS_ASCII', 'TERM')}
    result = subprocess.run([sys.executable, '-c', PAGE, str(TESTS)], env={**base, **env},
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return result.stdout


@pytest.mark.parametrize('env', [dict(MHS_ASCII='1'), dict(TERM='dumb')])
def test_ascii_mode_emits_only_ascii(env):
    page = run(**env)
    assert page.endswith('True')
    assert page.isascii(), sorted({c for c in page if not c.isascii()})


@pytest.mark.parametrize('value', ['0', 'false', ''])
def test_ascii_zero_is_off(value):
    assert run(MHS_ASCII=value).endswith('False')
