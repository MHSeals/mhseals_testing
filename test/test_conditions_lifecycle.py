"""Pure lifecycle regressions: polling time and explicit admission boundaries."""
from mhseals_testing.harness.conditions import Conditions
from test_harness_conditions import Clock, feed


def test_no_message_poll_expires_historical_good_and_detail():
    clock = Clock()
    conditions = Conditions(clock=clock, ceiling=lambda *_: .95)
    feed(conditions, clock, 40, .01)
    assert conditions.evaluate()['verdict'] == 'good'
    clock.now += conditions.s.drift_stale_s + .1
    assert conditions.facts()['conditions.verdict'].value == 'unknown'
    assert conditions.detail() is None
    assert not conditions.samples


def test_later_cut_does_not_retroactively_remove_admitted_windows():
    clock = Clock()
    conditions = Conditions(clock=clock)
    feed(conditions, clock, 40, .01)
    previous = conditions.detail()
    assert previous['pool'] >= 3
    conditions.cut(clock.now + .1)
    current = conditions.detail(clock.now + 2)
    assert current['pool'] == previous['pool']
    assert [row['T'] for row in current['windows']] == [row['T'] for row in previous['windows']]


def test_expiry_bypasses_refresh_and_clears_derived_direction_state():
    from dataclasses import replace
    from mhseals_testing.analysis.conditions import load_settings
    from mhseals_testing.analysis.readiness import Direction
    clock = Clock()
    conditions = Conditions(clock=clock, settings=replace(load_settings(), drift_stale_s=40),
                            planned=lambda: [Direction('surge+', 'surge', .4)])
    feed(conditions, clock, 40, .01)
    good = conditions.evaluate()
    assert good['ceilings']
    assert conditions.evaluate(clock.now + .21) is not good  # expiry beats REFRESH_S
    # Retain exactly the last sample at the boundary, then expire it inside REFRESH_S.
    clock.now = conditions.samples[-1][0] + 40
    first = conditions.evaluate()
    assert len(conditions.samples) == 1 and first['verdict'] == 'unknown'
    clock.now += .01
    assert conditions.evaluate() is not first
    assert not conditions.samples and not conditions._segments
    assert conditions.detail() is None and conditions.by_direction is None
    assert conditions.summary()['ceilings'] is None


def test_epoch_ticks_reorder_reset_and_poll_clock_are_independent():
    clock = Clock()
    conditions = Conditions(clock=clock)
    conditions.advance(0, epoch=1)
    feed(conditions, clock, 40, .01)
    first = conditions.evaluate()
    count = len(conditions.samples)
    conditions.advance(clock.now + .2, epoch=1)
    assert len(conditions.samples) == count
    assert conditions.evaluate(clock.now + .2) is first
    conditions.feed(1, (99, 99), 0, 0, True, .1)
    assert len(conditions.samples) == count and conditions.samples[-1][0] == clock.now
    conditions.advance(clock.now + .3, epoch=2, eligibility=None)
    assert not conditions.samples and conditions.detail(clock.now + .3) is None
    conditions.feed(clock.now + .4, (0, 0), 0, 0, False, .1)
    conditions.advance(clock.now + .5, epoch=2)
    assert conditions.samples
    conditions.advance(1, epoch=2)
    assert not conditions.samples and conditions.t_coast == -float('inf')
    conditions.feed(1, (0, 0), 0, 0, True, .1)
    conditions.reset()
    assert not conditions.samples and not conditions._segments and conditions.by_direction is None


def test_invalid_clock_never_refreshes_historical_fact_and_bad_epoch_refuses_before_mutation():
    import pytest
    clock = Clock()
    conditions = Conditions(clock=clock)
    feed(conditions, clock, 40, .01)
    assert conditions.evaluate()['verdict'] == 'good'
    before = list(conditions.samples)
    with pytest.raises(ValueError, match='epoch'):
        conditions.advance(clock.now + 1, epoch=[])
    assert list(conditions.samples) == before
    for bad in (float('nan'), float('inf'), 10**400, True):
        fact = conditions.facts(bad)['conditions.verdict']
        assert fact.value == 'unknown' and fact.time is None
        assert not conditions.samples and conditions.detail(bad) is None


def test_diagnostics_age_receipts_and_details_on_polling_time():
    from unittest.mock import patch
    from mhseals_testing.analysis import readiness
    clock = Clock()
    conditions = Conditions(clock=clock)
    feed(conditions, clock, 40, .01)
    previous = conditions.detail()
    current = conditions.detail(clock.now + .3)
    assert current['windows'][0]['ago'] == previous['windows'][0]['ago'] + .3
    with patch.object(readiness, 'drift_sigma', wraps=readiness.drift_sigma) as sigma:
        conditions.evaluate(clock.now + 2)
        assert sigma.call_args.args[1] == clock.now + 2
    fact = conditions.facts(clock.now + 2.1)['conditions.verdict']
    assert fact.time == clock.now + 2.1
    assert fact.data['source_last_time'] == clock.now
    assert abs(fact.data['source_age'] - 2.1) < 1e-10


def test_short_explicit_boundary_cannot_join_windows_or_accuracy_medians():
    from dataclasses import replace
    from mhseals_testing.analysis.conditions import load_settings
    # Remove coast only in this test so a 0.1s explicit gap, shorter than the
    # normal permitted jitter gap, is the sole reason these spans cannot join.
    settings = replace(load_settings(), coast_min_s=0)
    conditions = Conditions(settings=settings)
    for i in range(20):
        conditions.feed(i * .2, (0, 0), .3, 0, True, .1)
    conditions.advance(3.8, eligibility=False)
    conditions.advance(3.9, eligibility=True)
    for i in range(20):
        conditions.feed(3.9 + i * .2, (0, 0), .3, 0, True, .1)
    detail = conditions.detail(7.7)
    assert len(detail['windows']) == 2
    assert all(row['T'] < 4.1 for row in detail['windows'])
    # Bad accuracy after the boundary must not contaminate a prior segment's
    # centered median; it remains a usable 4s window.
    for i in range(20, 35):
        conditions.feed(3.9 + i * .2, (0, 0), .3, 0, True, 10)
    detail = conditions.detail(11)
    assert detail['windows'][0]['flag'] == '' and detail['windows'][0]['T'] < 4.1


def test_windows_survive_motion_and_new_conservative_coast():
    clock = Clock()
    conditions = Conditions(clock=clock)
    feed(conditions, clock, 40, .01)
    old = conditions.detail()
    conditions.advance(clock.now + .1, eligibility=False)
    clock.now += .2
    conditions.feed(clock.now, (1, 0), .3, 0, False, .1)
    conditions.advance(clock.now + .1, eligibility=True, t95=3)
    assert conditions.t_coast == clock.now + .1 + 1.33 * 3
    feed(conditions, clock, 40, .01, seed=0)
    new = conditions.detail()
    assert new['pool'] >= old['pool']
    old_windows = [(round(row['ago'] + 40.2, 6), row['T']) for row in old['windows']]
    assert [(round(row['ago'], 6), row['T'])
            for row in new['windows'][:len(old_windows)]] == old_windows
    assert all(row['ago'] + row['T'] < 100 for row in new['windows'])


class Orchestrator:
    """Exercise the unwired contract using actual source/status and evidence seams."""

    def __init__(self):
        from mhseals_testing.harness.conditions_feed import ConditionsFeed
        from mhseals_testing.harness.conditions_evidence import EvidenceGate
        from mhseals_testing.monitor import BoatMonitor
        from mhseals_testing.profiles import load_profile
        self.clock = Clock()
        self.clock.now = 10.0
        self.monitor = BoatMonitor(clock=self.clock)
        self.source = ConditionsFeed(load_profile('boat'))
        self.gate = EvidenceGate(output_ttl=2, command_ttl=2)
        self.conditions = Conditions(clock=self.clock)

    def tick(self, now, *, message=True, valid=True, command_known=True, snapshot=None):
        from dataclasses import replace
        from mhseals_testing.health.pwm import Output, Readback
        from mhseals_testing.harness.conditions_evidence import CommandObservation
        self.clock.now = now
        if message:
            self.monitor.observe('odometry', {'yaw': 0, 'wz': 0}, stamp_age=0)
            self.monitor.observe('GPS raw', {'ve': .05, 'vn': -.02, 'vel_acc': .1,
                                             'fix': 3 if valid else 2}, stamp_age=0)
        sample = self.source.poll(snapshot or self.monitor.snapshot(), now,
                                  eligible=True, commanding=False)
        epoch = self.source.status.epoch
        command = CommandObservation(now, (0, 0, 0), (0, 0, 0), False, epoch, now, now)
        readback = Readback('topic', now, tuple(Output(str(i), 'pulse', duty_ns=1500000)
                                                for i in range(4)), True)
        state = self.gate.update(afloat=True, bench=False, readback=readback,
                                 command=command if command_known else None,
                                 local=(0, 0, 0), raw_active=False, now=now, epoch=epoch)
        admitted = replace(state, eligible=state.eligible and self.source.status.valid)
        self.conditions.advance(now, epoch=epoch, eligibility=admitted)
        if sample:
            self.conditions.feed(sample.time, sample.velocity, sample.heading,
                                 sample.rate, sample.eligible, sample.accuracy)
        return sample, state


def test_actual_poll_status_duplicate_stale_and_invalid_restart_without_sample():
    flow = Orchestrator()
    for i in range(201):
        flow.tick(10 + i / 5)
    assert flow.conditions.evaluate()['verdict'] == 'good'
    previous = flow.conditions.detail()
    count = len(flow.conditions.samples)
    _, quiet = flow.tick(50.1, message=False)
    active = flow.conditions._active
    assert flow.source.status.valid and flow.conditions._active is not None
    assert len(flow.conditions.samples) == count
    _, next_quiet = flow.tick(50.2, message=False)
    assert next_quiet.coast_started == quiet.coast_started
    assert flow.conditions._active == active and len(flow.conditions.samples) == count
    flow.tick(52.1, message=False)
    assert not flow.source.status.valid and flow.conditions._active is None
    assert flow.conditions.detail()['pool'] == previous['pool']
    serial = flow.conditions._serial
    for i in range(10):
        flow.tick(52.2 + i / 10, message=False)
    assert flow.conditions._serial == serial
    assert len(flow.conditions.samples) == count
    # A valid source return with unchanged quiet evidence still starts its own
    # conservative coast; it must not inherit a pre-loss ready boundary.
    flow.tick(54)
    assert flow.conditions.t_coast == 54 + flow.conditions.s.coast_min_s
    assert flow.conditions.detail()['pool'] == previous['pool']
    old_epoch = flow.source.status.epoch
    flow.monitor.sensors.clear()
    sample, _ = flow.tick(55, valid=False)
    assert sample is None and not flow.source.status.valid
    assert flow.source.status.epoch == old_epoch + 1
    assert not flow.conditions.samples and flow.conditions.detail() is None
    assert flow.conditions.summary()['verdict'] == 'unknown'


def test_actual_command_unknown_to_zero_starts_one_new_coast_and_retains_history():
    flow = Orchestrator()
    for i in range(201):
        flow.tick(10 + i / 5)
    old = flow.conditions.detail()
    flow.tick(50.1, message=False, command_known=False)
    assert flow.conditions._active is None
    serial = flow.conditions._serial
    for i in range(10):
        _, state = flow.tick(50.11 + i / 100, message=False, command_known=False)
        assert state.cut
    assert flow.conditions._serial == serial
    _, state = flow.tick(50.3, message=False)
    assert state.coast_started == 50.3
    assert flow.conditions.t_coast == 52.3
    flow.tick(50.4, message=False)
    assert flow.conditions._serial == serial + 1
    assert flow.conditions.detail()['pool'] == old['pool']
    assert len(flow.conditions.samples) == 201
    # A swapped old row closes current admission conservatively but never resets
    # the source epoch or historical eligible segments.
    prior = flow.monitor.snapshot()
    flow.tick(50.6)
    epoch = flow.source.status.epoch
    flow.tick(50.7, message=False, snapshot=prior)
    assert flow.source.status.reason == 'reordered source'
    assert flow.source.status.epoch == epoch
    assert flow.conditions.detail()['pool'] >= old['pool']


def test_thinning_is_bounded_and_never_appends_from_advancement():
    conditions = Conditions()
    for i in range(1001):
        conditions.feed(i / 100, (0, 0), 0, 0, True, .1)
    assert len(conditions.samples) <= 10 * 8 + 1
    count = len(conditions.samples)
    for i in range(101):
        conditions.advance(10 + i / 100)
    assert len(conditions.samples) == count


def test_omitted_epoch_is_not_an_explicit_none_epoch_or_admission_reset():
    clock = Clock()
    conditions = Conditions(clock=clock)
    conditions.advance(0, epoch=None)
    feed(conditions, clock, 40, .01)
    assert conditions.evaluate()['verdict'] == 'good'
    conditions.advance(clock.now + .2)
    assert conditions.samples and conditions.evaluate(clock.now + .2)['verdict'] == 'good'
    conditions.advance(clock.now + .3, epoch='next', eligibility=False)
    assert not conditions.samples and conditions._active is None


def test_repeated_cut_event_is_bounded_and_same_coast_event_is_not_reapplied():
    from mhseals_testing.harness.conditions_evidence import EvidenceState
    conditions = Conditions()
    quiet = EvidenceState(True, True, True, True, 10., 1)
    conditions.advance(10, epoch=1, eligibility=quiet, t95=3)
    active = conditions._active
    for i in range(100):
        conditions.advance(10 + i / 100, epoch=1, eligibility=quiet, t95=3)
    assert conditions._active == active and conditions.t_coast == 10 + 1.33 * 3
    assert len(conditions._segments) == 1 and not conditions.samples
    for i in range(100):
        conditions.cut(12, t95=3)
    assert len(conditions._segments) == 1
