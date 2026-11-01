"""The math notes must not teach the secant decoupling C = G^-1 D."""
import re
from pathlib import Path

SECTIONS = Path(__file__).resolve().parents[1] / 'docs' / 'sections'
SECANT = re.compile(r'C\s*=\s*G\^\{?-1\}?|GC\s*=\s*D|C_\\text\{old\}')


def _text(name):
    return (SECTIONS / name).read_text()


def test_no_section_teaches_the_secant_decoupling():
    for path in SECTIONS.glob('*.tex'):
        assert not SECANT.search(path.read_text()), path.name


def test_mixer_decoupling_is_signed_columns_found_as_a_root():
    thrust = _text('thrust.tex')
    assert re.search(r'C\^\{?\+\}?\\max\(\\bm e,\s*0\)\s*\+\s*C\^\{?-\}?\\min\(\\bm e,\s*0\)', thrust)
    assert 'affine map' in thrust and 'solves for its zero' in thrust
    assert re.search(r'c\^\{s_j\}_\{kj\}', _text('mixer.tex'))


def test_characterization_does_not_build_decoupling_from_g():
    s = ' '.join(_text('characterization.tex').lower().split())
    assert 'speed curves and signed correction columns' in s
