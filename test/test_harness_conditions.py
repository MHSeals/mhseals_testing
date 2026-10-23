"""Live conditions: fake clock, synthetic GPS at 5 Hz; the verdict gates the start of Characterize only."""
from types import SimpleNamespace

import numpy as np

from mhseals_hardware.configuration import update_config
from mhseals_testing.harness.conditions import Conditions
from mhseals_testing.procedures import Context, Settings, characterize
from mhseals_testing.runner import NEUTRAL

HZ = 5


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


def feed(cond, clock, seconds, sigma, drift=(0.05, -0.02), seed=0, ok=True):
    """Advance the fake clock, feeding AR(1) velocity noise about ``drift``."""
    rng = np.random.default_rng(seed)
    x = np.zeros(2)
    for _ in range(int(seconds * HZ)):
        clock.now += 1 / HZ
        x = 0.8 * x + rng.normal(0, sigma * 0.6, 2)
        cond.feed(clock.now, np.array(drift) + x, 0.3, rng.normal(0, 0.002), ok, 0.1)


def test_unknown_while_measuring_then_good_in_calm_water():
    clock = Clock()
    cond = Conditions(clock=clock)
    feed(cond, clock, 10, 0.01)
    assert cond.evaluate()['verdict'] == 'unknown' and 'measuring' in cond.evaluate()['reason']
    feed(cond, clock, 30, 0.01, seed=1)
    r = cond.summary()
    assert r['verdict'] == 'good', r
    assert abs(r['drift'][0] - 0.05) < 0.02 and r['neutral_s'] >= 24
    assert cond.facts()['conditions.verdict'].value == 'good'
    assert cond.facts()['conditions.ceiling'].value is None      # no plan: the ceiling is not limiting


def test_gusty_water_is_poor_with_a_reason():
    clock = Clock()
    cond = Conditions(clock=clock)
    feed(cond, clock, 40, 0.3)
    fact = cond.facts()['conditions.verdict']
    assert fact.value == 'poor' and 'drift_trust' in fact.data['reason']


def test_samples_that_are_not_neutral_never_rate():
    clock = Clock()
    cond = Conditions(clock=clock)
    feed(cond, clock, 40, 0.01, ok=False)
    assert cond.evaluate()['verdict'] == 'unknown' and cond.evaluate()['neutral_s'] == 0


def test_cut_holds_back_the_coast_and_old_samples_leave_the_ring():
    clock = Clock()
    cond = Conditions(clock=clock)
    cond.cut(0.0, t95=6.0)
    assert cond.t_coast == 1.33 * 6.0
    feed(cond, clock, 700, 0.01)
    assert cond.samples[0][0] >= clock.now - cond.s.drift_stale_s


def test_injected_ceiling_can_make_it_poor_or_marginal():
    clock = Clock()
    cond = Conditions(clock=clock, ceiling=lambda estimate, windows: 0.52)
    feed(cond, clock, 40, 0.01)
    r = cond.evaluate()
    assert r['verdict'] == 'poor' and 'ceiling 52%' in r['reason']


class FakeRunner:
    def __init__(self, clock, cond, sigma):
        self.clock, self.cond, self.sigma, self.holds, self.stopped = clock, cond, sigma, [], False

    def hold(self, command, seconds, label=''):
        self.holds.append((tuple(command), label))
        start = self.clock.now
        if tuple(command) == NEUTRAL:
            feed(self.cond, self.clock, seconds, self.sigma)
        else:
            self.clock.now += seconds
        return start, self.clock.now

    def stop(self):
        self.stopped = True

    def invalid(self, start, end):
        return False


def run_characterize(sigma, tmp_path):
    update_config(dict(decoupling=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]), tmp_path / 'hardware.yaml')
    clock = Clock()
    cond = Conditions(clock=clock)
    runner = FakeRunner(clock, cond, sigma)
    records = []
    from contextlib import contextmanager

    @contextmanager
    def motion(name, **details):
        runner.clock_fn = clock
        runner.clock = clock
        yield runner

    ctx = Context(profile=None, monitor=SimpleNamespace(history=lambda *a: []), settings=Settings(neutral=30.0), motion=motion,
                  prompt=lambda *a: '', choose=lambda *a: 0, record=records.append,
                  services=dict(conditions=cond, user_config=tmp_path / 'hardware.yaml'))
    return ctx, runner, records


def test_poor_water_ends_characterize_before_any_motion(tmp_path):
    ctx, runner, records = run_characterize(0.3, tmp_path)
    result = characterize.run(ctx)
    assert result['verdict'] == 'ABORTED' and result['reason'].startswith('conditions poor: ')
    assert runner.stopped and [h[1] for h in runner.holds] == ['conditions']     # only the neutral step ran
    assert records == [result] and result['conditions']['verdict'] == 'poor'


def test_good_water_does_not_stop_the_start_and_is_not_rechecked(tmp_path):
    ctx, runner, records = run_characterize(0.01, tmp_path)
    try:
        characterize.run(ctx)
    except Exception:
        pass                    # the fake runner has no history for the fit; only the gate matters here
    # The survey went on past the neutral step (it stops later only because the fake has no odometry).
    assert runner.holds[0][1] == 'conditions' and len(runner.holds) > 1 and runner.holds[1][1] == 'settle'
