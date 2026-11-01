"""Manual's slow interlocks are evaluated beside the key loop; the cheap ones on every press.

A disarm or a ROS fault blocks the very next press; a stale or failed evaluation of the slow ones
(arming fault, a rival publisher, a pending correction) blocks like a tripped interlock.
"""

import threading
import time

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


class Checks:
    """Counts evaluations (each one asks the graph in the app); answers ``reasons`` or raises them."""

    def __init__(self, reasons=()):
        self.reasons, self.calls = reasons, 0

    def __call__(self):
        self.calls += 1
        if isinstance(self.reasons, Exception):
            raise self.reasons
        return list(self.reasons)


class App:
    armed, fault = True, ''

    def now(self):
        if not self.armed:
            return ['outputs are not armed']
        return ['ROS fault: ' + self.fault] if self.fault else []


def guarded(checks, now=lambda: [], clock=None):
    """A publisher over Interlocks entered with no checker thread: the test runs ``check`` itself."""
    clock = clock or [0.0]
    interlocks = Interlocks(now, checks, lambda: clock[0], spawn=lambda work: None)
    interlocks.__enter__()
    plant = Plant()
    return GuardedPublisher(plant, interlocks), plant, clock


def test_publishes_answer_from_the_last_evaluation_not_a_new_one_each():
    checks = Checks()
    publisher, plant, clock = guarded(checks)
    assert checks.calls == 1                        # once when manual starts
    for step in range(20):
        clock[0] = step * CHECK_STALE_S / 20
        publisher.publish(MOVE)
    assert checks.calls == 1 and plant.sent == [MOVE] * 20


def test_disarm_between_checks_still_blocks_next_press():
    """The adversarial review's case: a stop or disarm must not wait for the next evaluation."""
    app, checks = App(), Checks()
    publisher, plant, clock = guarded(checks, app.now)
    clock[0] = 0.05
    publisher.publish(MOVE)
    app.armed = False                               # the desk's stop / a fault disarms on the main thread
    clock[0] = 0.1
    with pytest.raises(Aborted, match='outputs are not armed'):
        publisher.publish(MOVE)
    assert plant.sent == [MOVE, 'neutral'] and checks.calls == 1


def test_a_ros_fault_blocks_the_next_press_without_an_evaluation():
    app, checks = App(), Checks()
    publisher, plant, _ = guarded(checks, app.now)
    app.fault = 'executor stopped'
    with pytest.raises(Aborted) as error:
        publisher.publish(MOVE)
    assert error.value.cause == 'guard:ROS fault' and checks.calls == 1


def test_a_stale_evaluation_blocks_and_neutralizes():
    publisher, plant, clock = guarded(Checks())     # the checker never comes back (a stuck graph call)
    clock[0] = CHECK_STALE_S
    publisher.publish(MOVE)                         # at the bound: still current
    clock[0] = CHECK_STALE_S + 0.01
    with pytest.raises(Aborted, match='interlock check stale') as error:
        publisher.publish(MOVE)
    assert error.value.by == 'guard' and error.value.cause == 'guard:interlock check stale'
    assert plant.sent == [MOVE, 'neutral']
    publisher.publish(NEUTRAL)                      # neutral always goes out
    assert plant.sent[-1] == NEUTRAL


def test_a_failed_evaluation_blocks_never_clears():
    checks = Checks()
    publisher, plant, clock = guarded(checks)
    checks.reasons = RuntimeError('graph gone')
    publisher.blockers.check()
    with pytest.raises(Aborted, match='interlock check failed: graph gone'):
        publisher.publish(MOVE)
    assert plant.sent == ['neutral']


def test_a_failing_first_evaluation_blocks_the_first_press():
    publisher, plant, _ = guarded(Checks(RuntimeError('no node')))
    with pytest.raises(Aborted, match='interlock check failed'):
        publisher.publish(MOVE)


def test_a_trip_is_seen_at_the_next_evaluation_and_the_stamp_is_its_start():
    checks, clock = Checks(), [0.0]

    def slow():
        clock[0] += 0.5                             # the check itself takes 0.5 s
        return checks()
    publisher, plant, _ = guarded(slow, clock=clock)
    checks.reasons = ['/cmd_vel/manual also published by /rival']
    publisher.blockers.check()
    with pytest.raises(Aborted, match='rival'):
        publisher.publish(MOVE)
    assert clock[0] - publisher.blockers.result[0] == pytest.approx(0.5)    # aged from when it began


def test_without_a_checker_thread_manual_never_starts_and_ends_neutral():
    """The review's nit: a thread that cannot start leaves nothing driving, and neutral goes out."""
    def spawn(work):
        raise RuntimeError("can't start new thread")
    plant, shown = Plant(), []
    ctx = context(plant, shown, spawn=spawn, open_events=lambda: pytest.fail('keys opened'))
    with pytest.raises(RuntimeError, match="can't start new thread"):
        manual.run(ctx)
    assert plant.sent == ['neutral'] and shown[-1][-1] == {'done': True}


def test_the_checker_reschedules_itself_and_stops_with_manual():
    calls, done = [], threading.Event()

    def checks():
        calls.append(time.monotonic())
        if len(calls) >= 4:
            done.set()
        return []
    interlocks = Interlocks(lambda: [], checks)
    with interlocks:
        assert done.wait(5)
    count = len(calls)
    time.sleep(3 * CHECK_S)
    assert len(calls) <= count + 1                  # at most the one under way when manual ended
    gaps = [b - a for a, b in zip(calls, calls[1:4])]
    assert all(gap >= CHECK_S * 0.9 for gap in gaps)


def test_a_slow_evaluation_is_followed_at_once_not_after_another_period():
    """Counted from each start: a check of 0.3 s is followed at once, so the answer stays under the bound."""
    calls, done = [], threading.Event()

    def checks():
        calls.append(time.monotonic())
        time.sleep(0.3)
        if len(calls) >= 3:
            done.set()
        return []
    interlocks = Interlocks(lambda: [], checks)
    with interlocks:
        assert done.wait(5)
        assert interlocks() == []                   # never stale while it keeps answering
    assert all(b - a < 0.3 + CHECK_S for a, b in zip(calls, calls[1:]))


def test_a_slow_check_in_the_thread_never_holds_the_publish():
    release, entered, calls = threading.Event(), threading.Event(), []

    def checks():
        calls.append(1)
        if len(calls) > 1:
            entered.set()
            release.wait(5)
        return []
    clock = [0.0]
    plant = Plant()
    interlocks = Interlocks(lambda: [], checks, lambda: clock[0])
    try:
        with interlocks:
            publisher = GuardedPublisher(plant, interlocks)
            assert entered.wait(5)                  # the thread is inside the slow check
            publisher.publish(MOVE)                 # answers from the last evaluation
            clock[0] = CHECK_STALE_S + 0.01
            with pytest.raises(Aborted, match='stale'):   # still inside it: blocked, not clear
                publisher.publish(MOVE)
    finally:
        release.set()
    assert plant.sent == [MOVE, 'neutral']


# --- run(): the app's services ------------------------------------------------------------

class Events:
    def __init__(self, keys, delay=0.0):
        self.keys, self.delay = list(keys), delay

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def read(self, timeout=None):
        if timeout == 0:
            return None
        time.sleep(self.delay)
        return self.keys.pop(0) if self.keys else 'x'


def context(plant, shown, **services):
    from mhseals_hardware.configuration import load_defaults
    ctx = type('Ctx', (), {})()
    ctx.services = dict(hardware_config=load_defaults(), manual=dict(
        dict(plant=plant, blockers=lambda: [], checks=lambda: [], open_events=lambda: Events([]),
             show=lambda *a, **k: shown.append((*a, k))), **services))
    return ctx


def test_run_drives_with_a_real_checker_thread_and_stops_on_a_rival_it_finds():
    """App level: the real thread evaluates the slow checks while keys drive; a rival publisher that
    appears mid-drive ends manual as a guard stop within a few evaluations, and neutral goes out last."""
    plant, shown, rival = Plant(), [], []
    calls = []

    def checks():
        calls.append(threading.current_thread().name)
        return [f'/cmd_vel/manual also published by {name}' for name in rival]
    keys = ['press:w'] + ['repeat:w'] * 400            # ~20 s of key repeats at 0.05 s

    class Rival(Events):
        def read(self, timeout=None):
            if timeout != 0 and len(self.keys) == 380:
                rival.append('/teleop')             # appears 1 s into the drive
            return super().read(timeout)
    ctx = context(plant, shown, checks=checks, open_events=lambda: Rival(keys, 0.05))
    began = time.monotonic()
    with pytest.raises(Aborted, match='/teleop') as error:
        manual.run(ctx)
    assert error.value.by == 'guard' and time.monotonic() - began < 1.0 + 4 * CHECK_S + 1.0
    moving = [c for c in plant.sent if c != 'neutral' and any(c)]
    assert moving and plant.sent[-1] == 'neutral'
    assert 'manual-interlocks' in calls             # evaluated in the thread, not the key loop


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

