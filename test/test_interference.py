import contextlib
import math
import random
from types import SimpleNamespace

import pytest

from mhseals_hardware.thruster_mixer import NEUTRAL_PWM
from mhseals_testing.analysis.interference import LiveFit, fit, heading_error, verdict
from mhseals_testing.procedures import PROCEDURES, Context, Settings, interference
from mhseals_testing.runner import Aborted, Runner

FIELD = (235.0, 0.0)
# mG per 100% effort, (x, y), per output; the negative sign is not a mirror of the positive one.
POS = [(0, 40), (0, -10), (5, 0), (0, 0)]
NEG = [(0, 12), (0, -30), (0, 8), (0, 0)]


def field_change(efforts):
    x = y = 0.0
    for u, p, n in zip(efforts, POS, NEG):
        k = p if u > 0 else n
        x, y = x + k[0] * abs(u), y + k[1] * abs(u)
    return x, y


def test_sign_split_fit_recovers_asymmetric_coefficients():
    rng = random.Random(1)
    samples = []
    for i in range(4):
        for u in (-0.75, -0.5, -0.25, 0.25, 0.5, 0.75):
            efforts = [0.0] * 4
            efforts[i] = u
            dx, dy = field_change(efforts)
            samples.append((efforts, (dx + rng.gauss(0, .2), dy + rng.gauss(0, .2))))
    rows = fit(samples, FIELD)
    for row, p, n in zip(rows, POS, NEG):
        assert row['pos'] == pytest.approx(p, abs=1.0)
        assert row['neg'] == pytest.approx(n, abs=1.0)
    # 40 mG across the field is atan(40/235) = 9.7 degrees; only the perpendicular part counts
    assert rows[0]['error_deg'] == pytest.approx(math.degrees(math.atan(40 / 235)), abs=0.5)
    assert rows[2]['error_deg'] == pytest.approx(math.degrees(math.atan(8 / 235)), abs=0.5)  # worst sign
    assert verdict(rows)[0] == 'REROUTE'


def test_fit_needs_samples_and_ignores_non_finite():
    assert fit([], FIELD) is None
    assert fit([([0.5, 0, 0, 0], (float('nan'), 0))] * 20, FIELD) is None
    assert heading_error((10, 0), (0, 0)) != heading_error((10, 0), (0, 0))  # nan


def test_live_fit_waits_then_predicts_the_error_at_current_effort():
    live = LiveFit()
    assert live.degrees([0.5] * 4, FIELD) is None
    for _ in range(5):
        for i in range(4):
            efforts = [0.0] * 4
            efforts[i] = 0.5
            live.add(efforts, field_change(efforts))
    degrees = live.degrees([1.0, 0, 0, 0], FIELD)
    assert degrees == pytest.approx(math.degrees(math.atan(40 / 235)), abs=0.3)
    assert live.degrees([0, 0, 0, 0], FIELD) == pytest.approx(0, abs=1e-6)


class Keys:
    def __init__(self, clock):
        self.clock = clock

    def read(self, timeout):
        self.clock[0] += timeout


def make_ctx(blockers, services_extra=None):
    clock = [0.0]
    pulses, checks, recorded = [], [], []
    state = dict(efforts=[0.0] * 4)

    def mag():
        # the pulse in force right now decides the field
        efforts = [(p - NEUTRAL_PWM) / 400 for p in pulses[-1]] if pulses and pulses[-1] else [0] * 4
        dx, dy = field_change(efforts)
        return (FIELD[0] + dx, FIELD[1] + dy, 400.0)

    def guard():
        checks.append(clock[0])
        return blockers(clock[0])

    @contextlib.contextmanager
    def motion(name, **details):
        yield Runner(SimpleNamespace(neutral=lambda: None), guard, Keys(clock), clock=lambda: clock[0])

    services = dict(mag=mag, pulses=pulses.append)
    services.update(services_extra or {})
    ctx = Context(profile=None, monitor=SimpleNamespace(clock=lambda: clock[0]), settings=Settings(),
                  motion=motion, prompt=lambda *a, **k: None, choose=None, record=recorded.append,
                  services=services)
    return ctx, pulses, checks, recorded


def test_calibrate_step_fits_and_every_tick_goes_through_the_guard():
    ctx, pulses, checks, recorded = make_ctx(lambda t: [])
    result = interference.run(ctx)
    assert len(checks) > 100                       # supervise checks the guard each tick
    assert pulses[-1] is None                      # always ends released
    assert result['verdict'] == 'REROUTE' and recorded == [result]
    assert result['outputs'][1]['neg'][1] == pytest.approx(-30, abs=2)


def test_guard_blocker_aborts_the_pulses_and_releases_them():
    ctx, pulses, checks, recorded = make_ctx(lambda t: ['TF odom -> base_link: stale'] if t > 7 else [])
    with pytest.raises(Aborted, match='interlock'):
        interference.run(ctx)
    assert pulses[-1] is None and recorded == []
    assert len([p for p in pulses if p and p != [NEUTRAL_PWM] * 4]) <= 2


def test_missing_compass_feed_refuses_before_any_pulse():
    ctx, pulses, _, _ = make_ctx(lambda t: [])
    del ctx.services['mag']
    with pytest.raises(RuntimeError, match='compass'):
        interference.run(ctx)
    assert pulses == []


def test_registered_with_a_free_key():
    keys = [p.key for p in PROCEDURES]
    assert keys.count('e') == 1 and len(keys) == len(set(keys))
