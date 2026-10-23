from types import SimpleNamespace

import pytest

from mhseals_testing.runner import NEUTRAL, Aborted, Runner


class Plant:
    def __init__(self):
        self.sent, self.holds = [], []

    def publish(self, command):
        self.sent.append(command)

    def neutral(self):
        self.sent.append('neutral')

    def hold(self, cause='keys'):
        self.holds.append(cause)


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
    assert plant.holds == (['keys'] if key == 'space' else [])  # only Space asks for a hold


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
    assert events[-1][1] == {'outcome': 'complete', 'by': 'procedure'}
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


def overridden_runner(script, **kwargs):
    """A runner whose override cause follows ``script``: time -> cause, last entry at or before now wins."""
    run, plant, events, clock = runner(rate=4.0)
    run.override = lambda: next((c for t, c in sorted(script.items(), reverse=True) if t <= clock[0]), '')
    run.settle = 1.0
    for name, value in kwargs.items():
        setattr(run, name, value)
    return run, plant, events, clock


def test_override_pauses_invalidates_the_interrupted_hold_and_reruns_it():
    from mhseals_testing.runner import Overridden  # noqa: F401
    restored = []
    run, plant, events, clock = overridden_runner({3.0: 'rc', 6.0: ''},
                                                  on_pause=lambda: restored.append('C'),
                                                  on_resume=lambda: restored.append('identity'))
    early = run.hold((0.1, 0, 0), 2.0)            # finishes before the override
    window = run.hold((0.2, 0, 0), 2.0)           # interrupted at t=3, re-run from its start
    assert early == (0.0, 2.0) and not run.invalid(*early)
    assert window[0] >= 7.0 and window[1] - window[0] == 2.0 and not run.invalid(*window)
    assert run.invalid(2.0, 3.5)                  # the interrupted attempt
    assert restored == ['C', 'identity'] and 'neutral' in plant.sent
    assert [p for p, _ in events if p in ('override', 'pause', 'pause_end')] == ['override', 'pause', 'pause_end']
    entry = run.overrides[0]
    assert entry['by'] == 'rc' and entry['paused_s'] >= 3.0 and entry['holds_invalidated'] == []


def test_override_just_after_a_hold_invalidates_it_within_the_guard():
    run, _, _, _ = overridden_runner({2.25: 'rc', 3.0: ''})
    first = run.hold((0.1, 0, 0), 2.0)
    run.hold((0.1, 0, 0), 1.0)
    assert run.invalid(*first) and run.overrides[0]['holds_invalidated'] == [first]


def test_pause_waits_for_the_resume_gate():
    gate = iter([['moving'], ['moving'], []])
    run, _, _, clock = overridden_runner({1.0: 'rc', 1.5: ''}, gate=lambda: next(gate, []))
    run.hold((0.1, 0, 0), 2.0)
    assert run.overrides[0]['end'] >= 2.0


def test_pause_times_out_after_the_limit():
    run, plant, events, _ = overridden_runner({1.0: 'rc'}, pause_limit=600.0)
    with pytest.raises(Aborted, match='override_timeout'):
        run.hold((0.1, 0, 0), 5.0)
    assert plant.sent[-1] == 'neutral' and events[-1] == ('pause_end', {'outcome': 'override_timeout'})


def test_override_cause_reads_the_mux_state():
    from mhseals_testing.runner import override_cause
    assert override_cause({'input': 'auto'}) == '' and override_cause({'input': 'idle'}) == ''
    assert override_cause({'input': 'rc'}) == 'rc'
    assert override_cause({'input': 'auto'}, age_s=3.5) == 'unknown' and override_cause(None) == 'unknown'


def test_supervise_pauses_while_overridden_and_drops_the_paused_time():
    run, plant, events, clock = runner(rate=4.0)
    run.override = lambda: 'rc' if 2.0 <= clock[0] < 5.0 else ''
    calls, samples = [], []
    began, ended, finished = run.supervise(
        10.0, sample=samples.append, on_pause=lambda: calls.append(('pause', clock[0])),
        on_resume=lambda: calls.append(('resume', clock[0])))
    assert calls == [('pause', 2.0), ('resume', 5.0)] and not finished
    assert (began, ended) == (0.0, 10.0)           # 13 s on the clock, 3 s paused
    assert clock[0] == 13.0 and run.paused == 3.0
    assert plant.sent == []
    # Samples run on the clock minus the paused time: no gap where the pause was.
    assert samples[0] == 0.0 and all(b - a == 0.25 for a, b in zip(samples, samples[1:]))
    assert max(samples) < 10.0
    assert events == [('pause', dict(cause='rc')),
                      ('pause_end', dict(outcome='resume', paused_s=3.0))]


def test_a_pause_still_honours_the_abort_key():
    run, plant, _, clock = runner({12: 'space'}, rate=4.0)
    run.override = lambda: 'rc' if clock[0] >= 2.0 else ''
    with pytest.raises(Aborted, match='cancelled'):
        run.supervise(10.0)
    assert plant.sent == ['neutral']




def stop_of(events):
    return events[-1][1]


def test_session_stop_says_who_ended_it():
    run, _, events, _ = runner({2: 'space'})
    with pytest.raises(Aborted):
        with run.session('axis'):
            run.hold((0.1, 0, 0), 5.0)
    assert stop_of(events)['by'] == 'operator' and stop_of(events)['cause'] == 'space'

    run, _, events, _ = runner()
    with pytest.raises(ValueError):
        with run.session('axis'):
            raise ValueError('boom')
    assert stop_of(events)['by'] == 'error' and 'cause' not in stop_of(events)

    run, _, events, _ = runner()
    with run.session('axis'):
        pass
    assert stop_of(events)['by'] == 'procedure'


def test_guard_trip_mid_hold_is_stamped_guard():
    ticks = []

    def blockers():
        ticks.append(1)
        return ['odometry: STALE'] if len(ticks) > 2 else []

    run, _, events, _ = runner(blockers=blockers)
    with pytest.raises(Aborted):
        with run.session('axis'):
            run.hold((0.1, 0, 0), 5.0)
    assert stop_of(events)['by'] == 'guard' and stop_of(events)['cause'] == 'guard:odometry'


def test_manual_space_requests_a_hold_as_well_as_neutral():
    from mhseals_testing.procedures.manual import GuardedPublisher, drive
    from mhseals_hardware.manual_state import ManualState
    events = []
    plant = SimpleNamespace(hold=lambda cause: events.append(('hold', cause)),
                            publish=lambda c, manual=False: events.append(('pub', c, manual)),
                            neutral=lambda: events.append(('neutral',)))
    keys = iter(['space', 'x'])
    reader = SimpleNamespace(read=lambda timeout=None: next(keys))
    drive(reader, ManualState(0.5, 0.5, 0.5, 1.0), GuardedPublisher(plant, lambda: []),
          lambda *a: None)
    assert events == [('hold', 'keys'), ('pub', (0.0, 0.0, 0.0), True)]
