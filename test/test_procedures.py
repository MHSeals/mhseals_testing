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


def test_the_square_timeout_scales_with_its_length():
    """H5: a 4 m square (16 m) at the slowest accepted 0.1 m/s gets 60 + 160 s, not 120 s."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    from mhseals_testing.procedures import nav
    goal = SimpleNamespace(done=True, status='SUCCEEDED', cancel=lambda: None)
    fake = SimpleNamespace(pose=lambda: (0.0, 0.0, 0.0), navigate=lambda route: goal)
    seen = []
    runner = SimpleNamespace(paused=0.0, supervise=lambda timeout, **k: seen.append(timeout) or (0.0, 1.0, True))

    @contextmanager
    def supervise(*args, **kwargs):
        yield runner
    ctx = SimpleNamespace(services={'nav2': lambda: fake}, settings=SimpleNamespace(nav_distance=4.0),
                          prompt=lambda *a: None, supervise=supervise, record=lambda r: None)
    nav.waypoints(ctx)
    assert seen == [pytest.approx(220.0)]


def test_thruster_set_refuses_unless_the_mux_is_auto():
    from types import SimpleNamespace

    from mhseals_testing.procedures import thrusters
    for state in ('rc', 'hold'):
        ctx = SimpleNamespace(services={'mux_input': lambda s=state: s})
        with pytest.raises(RuntimeError, match='needs the mux in auto'):
            thrusters.run(ctx)


def test_thruster_set_accepts_auto_and_idle_mux():
    from types import SimpleNamespace

    from mhseals_testing.procedures import thrusters
    for state in ('auto', 'idle'):
        ctx = SimpleNamespace(services={'mux_input': lambda s=state: s})
        try:
            thrusters.run(ctx)
        except RuntimeError as error:
            assert 'needs the mux' not in str(error)
        except Exception:
            pass   # got past the guard; the stub context lacks the rest


def test_app_supplies_mux_input_from_the_arbiter():
    from types import SimpleNamespace

    from mhseals_testing.ui.app import App
    app = SimpleNamespace(arbiter=SimpleNamespace(mux={'input': 'rc'}))
    assert App.mux_input(app) == 'rc'
    app.arbiter.mux = None
    assert App.mux_input(app) == 'auto'
    assert App.mux_input(SimpleNamespace()) == 'auto'




def test_tree_puts_a_heading_before_each_group_and_keeps_item_indices():
    from mhseals_testing.procedures import tree
    assert tree(['A', 'A', 'B', '']) == [
        ('heading', 'A'), ('item', 0), ('item', 1), ('heading', 'B'), ('item', 2), ('item', 3)]


def test_every_procedure_has_a_group_and_groups_are_contiguous():
    from mhseals_testing.procedures import PROCEDURES
    groups = [p.group for p in PROCEDURES]
    assert all(groups)
    assert len(set(groups)) == len({g for i, g in enumerate(groups) if i == 0 or groups[i - 1] != g})


def test_groups_and_keys_follow_the_spec():
    from mhseals_testing.procedures import PROCEDURES
    by_group = {}
    for p in PROCEDURES:
        by_group.setdefault(p.group, []).append(p.key)
    assert by_group == {'Calibrate': ['f', 'g', 'e', 'v'], 'Drive': ['m', 'i'],
                        'Navigate': ['n', 'w', 's'], 'Mission': ['t'], 'Perception': ['p']}


def test_tree_headings_are_not_items_so_the_cursor_skips_them():
    from mhseals_testing.procedures import PROCEDURES, tree
    rows = tree([p.group for p in PROCEDURES])
    items = [i for kind, i in rows if kind == 'item']
    assert items == list(range(len(PROCEDURES)))
    assert sum(kind == 'heading' for kind, _ in rows) == 5


def run_square(statuses, side=4.0, pause_after=None):
    """waypoints() on a fake Nav2 whose goals end with ``statuses`` in turn; the goals it was sent."""
    from contextlib import contextmanager
    from types import SimpleNamespace

    from mhseals_testing.procedures import nav
    sent, ends = [], iter(statuses)

    def navigate(goals):
        sent.append(list(goals))
        return SimpleNamespace(done=True, status=next(ends), cancel=lambda: None)

    def supervise(timeout, done, on_pause, on_resume, sample, **kwargs):
        for tick in range(20):
            sample(tick)
            if tick == pause_after:
                on_pause()
                on_resume()
            if done():
                return 0.0, 1.0, True
        return 0.0, 1.0, False
    fake = SimpleNamespace(pose=lambda: (0.0, 0.0, 0.0), navigate=navigate)

    @contextmanager
    def runner_of(*args, **kwargs):
        yield SimpleNamespace(paused=0.0, supervise=supervise)
    ctx = SimpleNamespace(services={'nav2': lambda: fake}, settings=SimpleNamespace(nav_distance=side),
                          prompt=lambda *a: None, supervise=runner_of, record=lambda r: None)
    return nav.waypoints(ctx), sent


def test_the_square_drives_each_leg_as_its_own_goal_and_never_a_zero_length_loop():
    """H5: one NavigateThroughPoses ending at its start is a 0 m loop Nav2 ends at once."""
    import math
    result, sent = run_square(['SUCCEEDED'] * 4)
    assert result['status'] == 'SUCCEEDED' and len(sent) == 4
    position = (0.0, 0.0)
    for goals in sent:
        assert len(goals) == 1                         # NavigateToPose, never a through-poses loop
        assert math.dist(position, goals[0][:2]) >= 1.0
        position = goals[0][:2]
    assert sent[-1][0][:2] == pytest.approx((0.0, 0.0))  # it still returns home


def test_a_leg_that_fails_ends_the_square_without_sending_the_rest():
    result, sent = run_square(['SUCCEEDED', 'ABORTED', 'SUCCEEDED', 'SUCCEEDED'])
    assert result['status'] == 'ABORTED' and len(sent) == 2
