import pytest

from mhseals_testing.runner import NEUTRAL, Aborted, Runner


class Plant:
    def __init__(self):
        self.sent = []

    def publish(self, command):
        self.sent.append(command)

    def neutral(self):
        self.sent.append('neutral')


class Keys:
    """Advances the fake clock by each read's timeout; returns scripted keys."""

    def __init__(self, clock, script=None):
        self.clock, self.script, self.reads = clock, dict(script or {}), 0

    def read(self, timeout):
        self.reads += 1
        self.clock[0] += timeout
        return self.script.get(self.reads)


def runner(script=None, blockers=lambda: [], rate=10.0):
    clock = [0.0]
    plant, events = Plant(), []
    keys = Keys(clock, script)
    run = Runner(plant, blockers, keys, emit=lambda phase, **details: events.append((phase, details)),
                 clock=lambda: clock[0], rate=rate)
    return run, plant, events, clock


def test_hold_publishes_at_rate_and_returns_window():
    run, plant, _, _ = runner(rate=4.0)  # binary-exact period keeps the tick count exact
    assert run.hold((0.2, 0, 0), 1.0) == (0.0, 1.0)
    assert plant.sent == [(0.2, 0.0, 0.0)] * 4


@pytest.mark.parametrize('key', ['space', 'q', 'Q', 'escape'])
def test_abort_key_stops_at_neutral(key):
    run, plant, _, _ = runner({3: key})
    with pytest.raises(Aborted, match='cancelled'):
        run.hold((0.2, 0, 0), 5.0, 'surge')
    assert plant.sent[-1] == 'neutral' and len(plant.sent) == 4
    assert run.command == NEUTRAL


def test_blocker_mid_hold_stops_before_publishing_again():
    ticks = []

    def blockers():
        ticks.append(1)
        return ['odometry: STALE'] if len(ticks) > 2 else []

    run, plant, _, _ = runner(blockers=blockers)
    with pytest.raises(Aborted, match='odometry: STALE'):
        run.hold((0, 0, 0.3), 5.0)
    assert plant.sent == [(0, 0, 0.3)] * 2 + ['neutral']


def test_unexpected_errors_still_neutralize():
    run, plant, _, _ = runner()
    run.show = lambda *args: (_ for _ in ()).throw(KeyboardInterrupt)
    with pytest.raises(KeyboardInterrupt):
        run.hold((0.1, 0, 0), 1.0)
    assert plant.sent[-1] == 'neutral'


def test_session_brackets_events_and_ends_neutral():
    run, plant, events, _ = runner()
    with run.session('axis', axis='surge'):
        run.hold((0.1, 0, 0), 0.2)
    assert [phase for phase, _ in events] == ['axis_start', 'axis_stop']
    assert events[-1][1] == {'outcome': 'complete'}
    assert plant.sent[-1] == 'neutral'


def test_session_records_abort_and_refuses_to_start_when_blocked():
    run, plant, events, _ = runner({1: 'space'})
    with pytest.raises(Aborted):
        with run.session('axis'):
            run.hold((0.1, 0, 0), 1.0)
    assert 'cancelled' in events[-1][1]['outcome']
    blocked, plant, events, _ = runner(blockers=lambda: ['TF map -> odom: missing'])
    with pytest.raises(Aborted):
        with blocked.session('axis'):
            pass
    assert events == [] and plant.sent == []


def test_supervise_never_publishes_and_ends_on_done_or_timeout():
    run, plant, _, clock = runner(rate=4.0)
    samples = []
    assert run.supervise(1.0, sample=samples.append) == (0.0, 1.0, False)
    assert plant.sent == [] and samples == [0.0, 0.25, 0.5, 0.75]
    done = lambda: clock[0] >= 1.5
    assert run.supervise(5.0, done=done) == (1.0, 1.5, True)


def test_supervise_aborts_on_keys_and_blockers_at_neutral():
    run, plant, _, _ = runner({2: 'space'})
    with pytest.raises(Aborted, match='cancelled'):
        run.supervise(5.0, label='waypoints')
    assert plant.sent == ['neutral']
    blocked, plant, _, _ = runner(blockers=lambda: ['TF odom -> base_link: missing'])
    with pytest.raises(Aborted, match='TF'):
        blocked.supervise(5.0)
    assert plant.sent == ['neutral']


def test_fold_lowers_single_letters_only():
    from mhseals_testing.runner import fold
    assert (fold('Q'), fold('press:M'), fold('Escape'), fold(None)) == ('q', 'press:m', 'Escape', None)


def test_manual_exit_keys_ignore_case():
    from mhseals_testing.procedures.manual import drive
    class Held:
        read = staticmethod(lambda timeout=None: 'press:M')
    drive(Held, None, None, None)
