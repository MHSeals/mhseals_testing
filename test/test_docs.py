"""The math notes must not teach the secant decoupling C = G^-1 D."""
import re
from pathlib import Path

SECTIONS = Path(__file__).resolve().parents[1] / 'docs' / 'sections'


def _text(name):
    return (SECTIONS / name).read_text()


def test_mixer_decoupling_is_local_slope_columns():
    s = _text('mixer.tex')
    assert 'C^{+}' in s and 'C^{-}' in s
    assert 'local slope' in s and 'trust' in s
    assert not re.search(r'C\s*=\s*G\^\{-1\}\s*D\b', s)
    assert 'GC = D' not in s


def test_characterization_does_not_build_decoupling_from_g():
    s = _text('characterization.tex')
    assert 'speed curve' in s and 'trust' in s
    assert 'C_\\text{old}' not in s
