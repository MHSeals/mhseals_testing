"""Every package module must at least compile, so a bad paste fails here rather than at boat_test start."""
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent / 'mhseals_testing'


@pytest.mark.parametrize('path', sorted(ROOT.rglob('*.py')), ids=lambda p: str(p.relative_to(ROOT)))
def test_module_compiles(path):
    compile(path.read_text(), str(path), 'exec')
