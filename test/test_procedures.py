import pytest

from mhseals_testing.procedures import Procedure, unavailable
from mhseals_testing.profiles import load_profile


def procedure(*needs):
    return Procedure('x', 'test', run=lambda ctx: None, needs=frozenset(needs))


def test_unknown_needs_are_rejected():
    with pytest.raises(ValueError, match='unknown needs'):
        procedure('wings')


def test_availability_reasons():
    boat, sim = load_profile('boat'), load_profile('crane_direct')
    assert unavailable(procedure(), sim, armed=False) == ''
    assert 'arm' in unavailable(procedure('armed'), boat, armed=False)
    assert unavailable(procedure('armed', 'effort'), boat, armed=True) == ''
    assert 'effort' in unavailable(procedure('effort'), sim, armed=True)
    assert 'velocity' in unavailable(procedure('velocity'), boat, armed=True)
    assert unavailable(procedure('velocity'), boat.with_velocity_control(), armed=True) == ''
    assert 'ODROID' in unavailable(procedure('hardware'), load_profile('crane_full_stack'), True)
    assert 'bench' in unavailable(procedure('sensors'), boat, True, bench_override=True)
