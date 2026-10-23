"""The startup menu: take over, refuse, watch, and a stale owner left by a dead terminal."""

import io

from rich.console import Console

from mhseals_testing.ui.takeover import decide

from test_registry import World, info


def run(world, answers, me=2):
    answers = list(answers)
    console = Console(file=io.StringIO(), width=100)
    return decide(world.registry(), console, info(me), ask=lambda _: answers.pop(0)), console


def owned(tmp_path, alive):
    world = World(tmp_path, alive=alive)
    registry = world.registry()
    registry.claim(info(1))
    registry.set_children({'bag': {'pid': 9, 'pgid': 9}})
    return world


def test_a_free_seat_is_claimed_without_asking(tmp_path):
    choice, _ = run(World(tmp_path, alive={2}), [])
    assert choice.mode == 'own'


def test_take_over_adopts_the_children_the_owner_hands_on(tmp_path):
    world = owned(tmp_path, {1, 2, 9})
    owner = world.registry()
    world.responder = lambda: owner.release(1)
    choice, _ = run(world, ['1'])
    assert choice.mode == 'adopt' and choice.children == {'bag': {'pid': 9, 'pgid': 9}}
    assert world.registry().owner()['pid'] == 2


def test_quit_and_watch_leave_the_owner_alone(tmp_path):
    world = owned(tmp_path, {1, 2, 9})
    assert run(world, ['4'])[0].mode == 'quit'
    choice, console = run(world, ['3'])
    assert choice.mode == 'watch' and choice.watching == 1
    assert world.registry().owner()['pid'] == 1 and 'pid 1 (running)' in console.file.getvalue()


def test_an_owner_that_ignores_the_handover_is_refused_and_asked_again(tmp_path):
    world = owned(tmp_path, {1, 2, 9})
    choice, console = run(world, ['1', '4'])
    assert choice.mode == 'quit' and 'did not let go' in console.file.getvalue()


def test_a_stale_owner_offers_its_children_for_adoption_or_stopping(tmp_path):
    world = owned(tmp_path, {2, 9})              # pid 1 died, its bag still runs
    choice, console = run(world, ['1'])
    assert choice.mode == 'adopt' and 'bag' in choice.children
    assert 'Stale session' in console.file.getvalue()
    world = owned(tmp_path / 'again', {2, 9})
    choice, _ = run(world, ['2'])
    assert choice.mode == 'own' and 'bag' in choice.children   # the caller stops them


def test_stopping_a_live_owner_waits_for_it_to_exit_then_claims(tmp_path):
    world = owned(tmp_path, {1, 2})
    world.responder = lambda: world.alive.discard(1)             # SIGTERM kills it
    choice, _ = run(world, ['2'])
    assert choice.mode == 'own' and choice.children == {}        # its close stopped the bag: start fresh
    assert world.registry().owner()['pid'] == 2


def test_a_stale_owner_with_no_live_child_starts_fresh_not_in_the_old_directory(tmp_path):
    world = owned(tmp_path, {2})                 # pid 1 and its bag are both gone
    choice, _ = run(world, ['1'])
    assert choice.mode == 'own' and choice.children == {}
