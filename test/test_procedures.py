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
    assert 'command_mode=effort' in unavailable(procedure('effort'), sim, armed=True)
    assert 'command_mode=velocity' in unavailable(procedure('velocity'), boat, armed=True)
    assert unavailable(procedure('velocity'), boat.with_velocity_control(), armed=True) == ''
    assert 'ODROID' in unavailable(procedure('hardware'), load_profile('crane_full_stack'), True)
    assert 'bench' in unavailable(procedure('sensors'), boat, True, bench_override=True)


def test_thruster_balance_is_not_registered():
    from mhseals_testing.procedures import PROCEDURES
    assert 'b' not in {p.key for p in PROCEDURES}


def test_remaining_route_skips_reached_leading_waypoints():
    from mhseals_testing.procedures.nav import remaining_route
    route = [(4, 0, 0), (4, 4, 0), (0, 4, 0), (0, 0, 0)]
    assert remaining_route(route, [(3.5, 0.2), (4, 3.8)]) == route[2:]
    assert remaining_route(route, [(0, 4)]) == route   # later corner alone does not count
    assert remaining_route(route, []) == route


def test_thruster_set_refuses_unless_the_mux_is_auto():
    from types import SimpleNamespace

    from mhseals_testing.procedures import thrusters
    for state in ('rc', 'hold'):
        ctx = SimpleNamespace(services={'mux_input': lambda s=state: s})
        with pytest.raises(RuntimeError, match='needs the mux in auto'):
            thrusters.run(ctx)
