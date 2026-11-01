"""Manual's interlocks are evaluated off the key loop, and a stale or failed evaluation blocks."""

import threading

import pytest

from mhseals_testing.harness.components import Stack
from mhseals_testing.procedures import manual
from mhseals_testing.procedures.manual import CHECK_S, CHECK_STALE_S, GuardedPublisher, Interlocks
from mhseals_testing.runner import Aborted

MOVE = (0.5, 0.0, 0.0)
NEUTRAL = (0.0, 0.0, 0.0)


class Plant:
    def __init__(self):
        self.sent = []

    def publish(self, command, manual=False):
        self.sent.append(command)

    def neutral(self):
        self.sent.append('neutral')

    def hold(self, why):
        self.sent.append('hold')


class Blockers:
    """Counts evaluations (each one asks the graph in the app); answers ``reasons``."""

    def __init__(self, reasons=()):
        self.reasons, self.calls = list(reasons), 0

    def __call__(self):
        self.calls += 1
        if isinstance(self.reasons, Exception):
            raise self.reasons
        return list(self.reasons)


def guarded(blockers, spawned=None, clock=None):
    """A publisher over Interlocks whose spawn either queues the work (``spawned``) or runs it inline."""
    clock = clock or [0.0]
    spawn = spawned.append if spawned is not None else (lambda work: work())
    plant = Plant()
    return GuardedPublisher(plant, Interlocks(blockers, lambda: clock[0], spawn)), plant, clock


def test_publishes_answer_from_the_last_evaluation_not_a_new_one_each():
    blockers, spawned = Blockers(), []
    publisher, plant, clock = guarded(blockers, spawned)
    assert blockers.calls == 1                      # once when manual starts
    for step in range(20):                          # 20 publishes inside one CHECK_S
        clock[0] = step * CHECK_S / 40
        publisher.publish(MOVE)
    assert blockers.calls == 1 and spawned == [] and plant.sent == [MOVE] * 20


def test_the_next_evaluation_runs_beside_the_loop_one_at_a_time():
    blockers, spawned = Blockers(), []
    publisher, plant, clock = guarded(blockers, spawned)
    clock[0] = CHECK_S
    publisher.publish(MOVE)
    publisher.publish(NEUTRAL)                      # neutral publishes keep it current as well
    assert len(spawned) == 1 and blockers.calls == 1   # queued, not run on the key loop; no second
    spawned.pop()()
    assert blockers.calls == 2
    clock[0] = 2 * CHECK_S
    publisher.publish(NEUTRAL)
    assert len(spawned) == 1


def test_a_stale_evaluation_blocks_and_neutralizes():
    blockers, spawned = Blockers(), []          # the check never comes back (a stuck graph call)
    publisher, plant, clock = guarded(blockers, spawned)
    clock[0] = CHECK_STALE_S
    publisher.publish(MOVE)                     # at the bound: still current
    clock[0] = CHECK_STALE_S + 0.01
    with pytest.raises(Aborted, match='interlock check stale') as error:
        publisher.publish(MOVE)
    assert error.value.by == 'guard' and error.value.cause == 'guard:interlock check stale'
    assert plant.sent[-1] == 'neutral' and plant.sent.count(MOVE) == 1
    publisher.publish(NEUTRAL)                  # neutral always goes out
    assert plant.sent[-1] == NEUTRAL


def test_a_failed_evaluation_blocks_never_clears():
    blockers = Blockers()
    publisher, plant, clock = guarded(blockers)
    blockers.reasons = RuntimeError('graph gone')
    clock[0] = CHECK_S
    with pytest.raises(Aborted, match='interlock check failed: graph gone'):
        publisher.publish(MOVE)                 # this refresh ran inline and failed
    assert plant.sent == ['neutral']


def test_a_failing_first_evaluation_blocks_the_first_press():
    publisher, plant, _ = guarded(Blockers(RuntimeError('no node')), [])
    with pytest.raises(Aborted, match='interlock check failed'):
        publisher.publish(MOVE)


def test_a_trip_is_seen_at_the_next_evaluation_and_the_stamp_is_its_start():
    blockers = Blockers()
    clock = [0.0]

    def slow():
        clock[0] += 0.5                         # the check itself takes 0.5 s
        return blockers()
    publisher, plant, _ = guarded(slow, clock=clock)
    clock[0] = 0.5 + CHECK_S
    blockers.reasons = ['outputs are not armed']
    with pytest.raises(Aborted, match='outputs are not armed'):
        publisher.publish(MOVE)
    assert clock[0] - publisher.blockers.result[0] == pytest.approx(0.5)    # aged from when it began


def test_a_spawn_that_fails_leaves_the_next_refresh_free():
    def broken(work):
        raise RuntimeError("can't start new thread")
    interlocks = Interlocks(Blockers(), lambda: 1.0, broken)
    interlocks.result = (0.0, [])
    with pytest.raises(RuntimeError):
        interlocks.refresh()
    assert not interlocks.busy


def test_the_default_spawn_is_a_thread_so_a_slow_check_never_holds_the_publish():
    release, entered = threading.Event(), threading.Event()
    calls = []

    def blockers():
        calls.append(1)
        if len(calls) > 1:
            entered.set()
            release.wait(5)
        return []
    clock = [0.0]
    plant = Plant()
    publisher = GuardedPublisher(plant, Interlocks(blockers, lambda: clock[0]))
    clock[0] = CHECK_S
    publisher.publish(MOVE)                     # starts the check in a thread, answers from the last
    assert entered.wait(5) and plant.sent == [MOVE]
    clock[0] = CHECK_STALE_S + 0.01
    with pytest.raises(Aborted, match='stale'):  # still inside the slow check: blocked, not clear
        publisher.publish(MOVE)
    release.set()


def test_run_evaluates_the_manual_blockers_through_interlocks(monkeypatch):
    """The app's blockers (graph conflicts, arming fault) reach GuardedPublisher only through Interlocks."""
    seen = []
    monkeypatch.setattr(manual, 'drive', lambda keys, state, publisher, show, clock: seen.append(publisher))

    class Events:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False
    from mhseals_hardware.configuration import load_defaults
    plant = Plant()
    ctx = type('Ctx', (), {})()
    ctx.services = dict(hardware_config=load_defaults(), manual=dict(
        plant=plant, blockers=lambda: [], open_events=Events, show=lambda *a, **k: None, clock=lambda: 0.0))
    manual.run(ctx)
    assert isinstance(seen[0].blockers, Interlocks)


# --- the Stack's process scan -------------------------------------------------------------

def test_the_stack_scans_the_process_table_beside_the_page_after_the_first():
    clock, spawned, scans = [0.0], [], []

    def scan(specs):
        scans.append(clock[0])
        return {'first' if len(scans) == 1 else 'later': {}}
    stack = Stack([], None, lambda: clock[0], scan=scan, spawn=spawned.append)
    assert stack.here() == {'first': {}} and scans == [0.0]     # the first is read at once
    clock[0] = Stack.SCAN_S
    assert stack.here() == {'first': {}} and len(spawned) == 1  # the old view while the next is read
    assert stack.here() == {'first': {}} and len(spawned) == 1  # one scan at a time
    spawned.pop()()
    assert stack.here() == {'later': {}}


def test_a_stack_scan_that_stops_answering_is_no_view():
    clock, spawned = [0.0], []
    stack = Stack([], None, lambda: clock[0], scan=lambda specs: {'x': {}}, spawn=spawned.append)
    assert stack.here() == {'x': {}}
    clock[0] = Stack.SCAN_STALE_S + 0.1
    assert stack.here() is None


def test_a_failing_stack_scan_is_no_view_and_frees_the_next():
    clock, spawned = [0.0], []
    table = [{'x': {}}]

    def scan(specs):
        if not table:
            raise ValueError('bad cmdline')
        return table[0]
    stack = Stack([], None, lambda: clock[0], scan=scan, spawn=spawned.append)
    assert stack.here() == {'x': {}}
    table.clear()
    clock[0] = Stack.SCAN_S
    stack.here()
    spawned.pop()()
    assert stack.here() is None and not stack.scanning
