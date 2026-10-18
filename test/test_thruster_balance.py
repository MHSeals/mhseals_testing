from contextlib import contextmanager
import math

import numpy as np
import pytest
import yaml

from mhseals_hardware.configuration import load_config, load_defaults, update_config
from mhseals_hardware.thruster_mixer import IDENTITY, POSITIONS, THRUSTER_MIXER
from mhseals_testing.analysis.thruster_balance import baseline, fit, motion_window, response
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import Context, PROCEDURES, Settings, unavailable
from mhseals_testing.procedures.balance import run
from mhseals_testing.profiles import load_profile
from mhseals_testing.runner import Aborted, Runner

MATRIX = np.array(THRUSTER_MIXER, dtype=float)
GAINS = np.array([1., 1.44, .64, 1.21])


def trials(gains=GAINS, power=2., reverse=.8, repeats=2):
    return [dict(position=p, effort=effort, direction=direction,
                 response=float(gains[i] * effort ** power * (reverse if direction < 0 else 1)))
            for i, p in enumerate(POSITIONS) for direction in (1, -1)
            for effort in (.125, .25) for _ in range(repeats)]


@pytest.mark.parametrize('power', [1., 2., 2.5])
def test_balance_accounts_for_nonlinearity_and_cancels_one_weak_thruster(power):
    gains = np.sqrt(GAINS) if power == 1 else GAINS
    result = fit(trials(gains=gains, power=power), MATRIX.flatten())
    scales = np.array([r['scale'] for r in result['rows']])
    assert result['exponent'] == pytest.approx(power)
    assert gains * scales ** power == pytest.approx([min(gains)] * 4)
    assert max(scales) == 1 and min(scales) > 0
    mixed = np.array(result['thruster_matrix']).reshape(4, 3)
    assert np.sign(mixed) == pytest.approx(np.sign(MATRIX))
    # Through the actual nonlinear actuator law, balanced surge has zero sway/yaw.
    effort = mixed[:, 0] * .25
    forces = gains * np.sign(effort) * abs(effort) ** power
    assert MATRIX[:, 1:].T @ forces == pytest.approx([0., 0.], abs=1e-12)
    # Re-running with the saved matrix must not attenuate the rows again.
    again = fit(trials(gains=gains, power=power), result['thruster_matrix'])
    assert [r['scale'] for r in again['rows']] == pytest.approx([1.] * 4)


@pytest.mark.parametrize('fault,reason', [
    ('missing', 'incomplete'), ('zero', 'invalid'), ('nan', 'invalid'),
    ('no-growth', 'unusable'), ('asymmetric', 'forward/reverse'),
    ('nonlinear', 'curves differ'), ('repeats', 'inconsistent'), ('dead', 'measured effort range'),
])
def test_bad_balance_data_never_proposes_a_matrix(fault, reason):
    rows = trials()
    if fault == 'missing':
        rows = [r for r in rows if r['position'] != 'fl']
    elif fault in ('zero', 'nan'):
        rows[0]['response'] = 0 if fault == 'zero' else float('nan')
    elif fault == 'no-growth':
        for row in rows:
            if row['position'] == 'fl':
                row['response'] = .1
    elif fault == 'asymmetric':
        for row in rows:
            if row['position'] == 'fl' and row['direction'] < 0:
                row['response'] *= 1.8
    elif fault == 'nonlinear':
        for row in rows:
            if row['position'] == 'fl' and row['effort'] == .125:
                row['response'] *= .5
    elif fault == 'repeats':
        rows[0]['response'] *= 2
    elif fault == 'dead':
        rows = trials(gains=[1., 1., .01, 1.])
    with pytest.raises(ValueError, match=reason):
        fit(rows, MATRIX.flatten())


def history(acceleration=(.1, .1), turning=False, noise=0.):
    samples = []
    for i in range(1, 41):
        t = i / 20
        yaw = .1 * t if turning else 0.
        world = np.array([.2, -.1]) + t * np.array(acceleration)
        c, s = math.cos(yaw), math.sin(yaw)
        samples.append((t, dict(vx=c * world[0] + s * world[1] + (-1) ** i * noise,
                               vy=-s * world[0] + c * world[1], wz=.1 if turning else 0.,
                               yaw=yaw, frame='odom', child='base_link')))
    return samples


def test_turning_does_not_misidentify_world_current_as_thrust():
    before = baseline(history(acceleration=(0., 0.)), 0, 2)
    with pytest.raises(ValueError, match='weak/noisy'):
        response(history(acceleration=(0., 0.), turning=True), 0, 2, before, 1, [1., 0.])
    measured = response(history(), 0, 2, before, 1, np.array([1., 1.]) / math.sqrt(2))
    assert measured['response'] == pytest.approx(math.sqrt(.02))
    with pytest.raises(ValueError, match='weak/noisy'):
        response(history(noise=1.), 0, 2, before, 1, np.array([1., 1.]) / math.sqrt(2))
    with pytest.raises(ValueError, match='reversed'):
        response(history(), 0, 2, before, -1, [1., 0.])
    with pytest.raises(ValueError, match='expected thruster direction'):
        response(history(acceleration=(.1, .2)), 0, 2, before, 1, [1., 0.])


@pytest.mark.parametrize('fault', ['gap', 'frame', 'nan', 'few', 'rotation'])
def test_bad_odometry_cannot_be_used_for_balance(fault):
    rows = history()
    if fault == 'gap':
        rows = rows[:5] + rows[20:]
    elif fault == 'few':
        rows = rows[:4]
    elif fault == 'frame':
        rows[10][1]['child'] = 'camera'
    elif fault == 'nan':
        rows[10][1]['yaw'] = float('nan')
    else:
        rows[-1][1]['yaw'] = 1.
    with pytest.raises(ValueError):
        motion_window(rows, 0, 2)


def test_baseline_requires_coasting_and_rotation_to_settle():
    with pytest.raises(ValueError, match='still accelerating'):
        baseline(history(acceleration=(.2, .1)), 0, 2)
    rows = history(acceleration=(0., 0.))
    for _, values in rows:
        values['wz'] = .2
    with pytest.raises(ValueError, match='turning'):
        baseline(rows, 0, 2)


class Boat:
    """A damped boat with mismatched quadratic thrusters and a steady current."""
    def __init__(self, monitor, config):
        self.monitor, self.config = monitor, config
        self.time, self.velocity = 0., np.array([.2, -.1])
        self.raw, self.pulses, self.stopped = [1500] * 4, [], False
        self.abort = False

    def set_pulses(self, pulses):
        self.raw = list(pulses or [1500] * 4)
        self.pulses.append(list(self.raw))

    def neutral(self):
        self.raw = [1500] * 4

    def read(self, timeout):
        steps = max(1, round(timeout / .05))
        dt = timeout / steps
        for _ in range(steps):
            efforts = [(self.raw[out - 1] - 1500) / 400 * (-1 if flip else 1)
                       for out, flip in zip(self.config['channel_map'], self.config['reversed'])]
            forces = GAINS * np.sign(efforts) * np.array(efforts) ** 2
            force = 4 * MATRIX[:, :2].T @ forces / math.sqrt(2)
            decay = math.exp(-2 * dt)
            self.velocity = np.array([.2, -.1]) + (self.velocity - [.2, -.1]) * decay + force * (1 - decay) / 2
            self.time += dt
            self.monitor.observe('odometry', dict(vx=self.velocity[0], vy=self.velocity[1],
                                                  wz=0., yaw=0., frame='odom', child='base_link'))
        return 'space' if self.abort and any(p != 1500 for p in self.raw) else None


def context(tmp_path, save=False):
    config = dict(load_defaults(), channel_map=[2, 1, 4, 3], reversed=[True, False, True, False],
                  decoupling=[2 * v for v in sum(IDENTITY, ())],
                  characterization={'velocity_gain': {axis: {other: float(axis == other) for other in
                                                               ('surge', 'sway', 'yaw')}
                                                      for axis in ('surge', 'sway', 'yaw')},
                                    'motion_limits': {}})
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump({key: config[key] for key in
                                   ('channel_map', 'reversed', 'decoupling', 'characterization')}))
    monitor = BoatMonitor()
    boat = Boat(monitor, config)
    monitor.clock = lambda: boat.time
    records, choices = [], []
    blockers = []

    @contextmanager
    def motion(*args, **kwargs):
        runner = Runner(boat, lambda: blockers, boat, clock=lambda: boat.time)
        with runner.session('balance'):
            yield runner

    def choose(*args, **kwargs):
        choices.append(kwargs)
        assert boat.raw == [1500] * 4
        return 'save' if save else 'discard'

    def disarm(*args):
        boat.neutral()
        boat.stopped = True

    def save_hardware(**changes):
        assert boat.stopped  # configuration only changes after disarming
        update_config(changes, path)

    ctx = Context(load_profile('boat'), monitor,
                  Settings(amplitude=.25, baseline=1, command=1, settle=5), motion,
                  lambda *args, **kwargs: None, choose, records.append,
                  services=dict(hardware_config=config, user_config=path, pulses=boat.set_pulses,
                                disarm=disarm, save_hardware=save_hardware))
    return ctx, boat, records, choices, path


@pytest.mark.parametrize('save', [False, True])
def test_procedure_measures_all_outputs_and_only_saves_after_review(tmp_path, save):
    ctx, boat, records, choices, path = context(tmp_path, save)
    before = path.read_text()
    result = run(ctx)
    scales = np.array([r['scale'] for r in result['rows']])
    # Discrete sampling and damping perturb the acceleration fit; verify the
    # resulting physical imbalance stays below 5%, rather than exact coefficients.
    assert scales == pytest.approx(np.sqrt(min(GAINS) / GAINS), abs=.02)
    balanced = GAINS * scales ** 2
    assert max(balanced) / min(balanced) < 1.05
    pulses = [p for p in boat.pulses if any(value != 1500 for value in p)]
    assert len(pulses) == 16 and all(sum(value != 1500 for value in p) == 1 for p in pulses)
    assert [i + 1 for i, p in enumerate(pulses[0]) if p != 1500] == [2]
    assert pulses[0][1] < 1500  # FL is reversed and wired to output 2
    assert boat.raw == boat.pulses[-1] == [1500] * 4
    assert len(choices[0]['rows']) == 4
    if save:
        actual = load_config(path)
        assert actual['thruster_matrix'] == pytest.approx(result['thruster_matrix'])
        assert actual['channel_map'] == [2, 1, 4, 3] and actual['reversed'] == [True, False, True, False]
        assert actual['characterization'] == {} and actual['decoupling'] == list(sum(IDENTITY, ()))
        assert records[-1]['verdict'] == 'SAVED' and boat.stopped
    else:
        assert path.read_text() == before and not boat.stopped
        assert records[-1]['verdict'] == 'FITTED'


def test_abort_always_ends_raw_pulses_and_preserves_configuration(tmp_path):
    ctx, boat, records, _, path = context(tmp_path, save=True)
    before = path.read_text()
    boat.abort = True
    with pytest.raises(Aborted):
        run(ctx)
    assert boat.raw == boat.pulses[-1] == [1500] * 4
    assert path.read_text() == before and not records


def test_balance_uses_existing_arming_and_readiness_rules():
    procedure = next(p for p in PROCEDURES if p.key == 'b')
    boat = load_profile('boat')
    assert 'arm' in unavailable(procedure, boat, False)
    assert 'bench' in unavailable(procedure, boat, True, True)
    assert 'effort' in unavailable(procedure, boat.with_velocity_control(), True)
    assert unavailable(procedure, load_profile('crane_full_stack'), True) == ''


def test_cannot_save_over_a_changed_thruster_mapping(tmp_path):
    ctx, boat, records, _, path = context(tmp_path, save=True)
    choose = ctx.choose

    def changed(*args, **kwargs):
        update_config({'channel_map': [1, 2, 3, 4]}, path)
        return choose(*args, **kwargs)

    ctx.choose = changed
    with pytest.raises(RuntimeError, match='changed during balance'):
        run(ctx)
    assert load_config(path)['channel_map'] == [1, 2, 3, 4]
    assert not boat.stopped and boat.raw == [1500] * 4
    assert records[-1]['verdict'] == 'FITTED'
