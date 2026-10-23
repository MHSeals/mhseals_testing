import math

import numpy as np
import pytest

from mhseals_hardware.thruster_mixer import THRUSTER_MIXER
from mhseals_testing.analysis import thrust_columns as tc
from mhseals_testing.analysis.thrust import speed_curve

S = tc.load_settings()
CURVES = {f'{a}{s}': dict(d1=d1, d2=d2) for a, d1, d2 in (('surge', 0.5, 1.5), ('sway', 0.8, 2.0), ('yaw', 0.7, 1.0))
          for s in '+-'}
MASSES = (1.0, 1.4)
GAINS = np.array([1.0, 0.7, 1.2, 0.85])        # hidden per-thruster strength the nominal mixer does not know
HULL = np.array([0.15, -0.1])                  # hidden one-sided hull cross wrench per a^2


def plant(direction, amplitude, cross):
    """Steady (|main|, cross response) of a fixture boat whose thrusters differ from the mixer."""
    j, s, ks = tc.split(direction)
    m = np.asarray(THRUSTER_MIXER)
    t = m @ (s * amplitude * np.asarray(tc.full_column(direction, cross)))
    w = m.T @ (GAINS * np.where(t < 0, S.reverse_ratio, 1) * t * np.abs(t)) / 4
    d = CURVES[tc.AXES[j] + ('+' if s * w[j] >= 0 else '-')]
    main = (-d['d1'] + math.sqrt(d['d1'] ** 2 + 4 * d['d2'] * abs(w[j]))) / (2 * d['d2'])
    y = np.zeros(2)
    for _ in range(30):     # the damping block depends on the cross velocity itself
        a = tc.damping_block(direction, s * main, y, CURVES, MASSES)
        y = np.linalg.solve(a, w[ks] + HULL * amplitude ** 2 * (1 if s > 0 else 0.5))
    return main, y


def hold(direction, amplitude, cross):
    main, y = plant(direction, amplitude, cross)
    return dict(column=list(cross), cross=y.tolist(), sigma=[0.002, 0.001], main=main, main_sigma=0.002,
                amplitude=amplitude)


def truth(direction, amplitude):
    c = np.zeros(2)
    for _ in range(40):
        y = plant(direction, amplitude, c)[1]
        jac = np.column_stack([(plant(direction, amplitude, c + e * 1e-6)[1] - y) / 1e-6 for e in np.eye(2)])
        c = c - np.linalg.solve(jac, y)
    return c


@pytest.mark.parametrize('direction', tc.DIRECTIONS)
def test_fixture_plant_columns_are_recovered(direction):
    history = [hold(direction, 0.6, [0.0, 0.0])]
    for _ in range(S.refine_holds + 2):
        column, _ = tc.next_column(history, direction, CURVES, MASSES, S)
        history.append(hold(direction, 0.6, column))
    assert column == pytest.approx(truth(direction, 0.6), abs=0.03)
    assert all(state == 'ok' for state in tc.entry_states(history[-1], direction, S))
    # Amplitude free: the same column nulls the lower amplitude too.
    assert np.abs(plant(direction, 0.3, column)[1]).max() < 0.5 * np.abs(history[0]['cross']).max()


def test_broyden_only_fires_on_a_significant_miss():
    d = 'surge+'
    history = [dict(hold(d, 0.6, [0.0, 0.0]), sigma=[1e-5, 1e-5])]
    history.append(dict(hold(d, 0.6, tc.next_column(history, d, CURVES, MASSES, S)[0]), sigma=[1e-5, 1e-5]))
    model = tc._model_jacobian(history[0], d, CURVES, MASSES, S)
    quiet = dict(history[1], cross=(np.array(history[0]['cross']) + model @ history[1]['column']).tolist())
    assert np.allclose(tc.next_column([history[0], quiet], d, CURVES, MASSES, S)[1], model)
    assert not np.allclose(tc.next_column(history, d, CURVES, MASSES, S)[1], model)


def test_noise_limited_inside_and_dead_entries_hold_their_value():
    d = 'sway+'
    first = hold(d, 0.6, [0.0, 0.0])
    noisy = dict(first, sigma=[1.0, 0.001])       # entry 0 cannot be resolved
    column, _ = tc.next_column([noisy], d, CURVES, MASSES, S)
    assert column[0] == 0.0 and column[1] != 0.0
    assert tc.entry_states(noisy, d, S)[0] == 'limited'
    done = dict(first, cross=[0.0, 0.0])
    assert tc.entry_states(done, d, S) == ['ok', 'ok']
    assert tc.next_column([done], d, CURVES, MASSES, S)[0] == [0.0, 0.0]
    assert tc.next_column([dict(first, main_sigma=1.0)], d, CURVES, MASSES, S)[0] == [0.0, 0.0]


def test_step_respects_the_saturation_headroom():
    d = 'surge+'
    big = dict(hold(d, 0.9, [0.0, 0.0]), cross=[5.0, 5.0])
    column, _ = tc.next_column([big], d, CURVES, MASSES, S)
    assert 0.9 * (1 + sum(abs(x) for x in column)) <= S.headroom + 1e-9


def test_stability_falls_toward_zero_with_speed():
    dets = [tc.stability('surge+', v, CURVES, MASSES) for v in (0.1, 0.5, 1.0)]
    assert dets[0] > dets[1] > dets[2]


def test_speed_curves_fitted_from_the_plant_are_monotone():
    for d in ('surge+', 'sway-', 'yaw+'):
        points = [(plant(d, a, [0, 0])[0], a) for a in (0.3, 0.6)]
        curve = speed_curve(points)
        assert curve['d1'] >= 0 and curve['d2'] >= 0
        assert tc._slope(curve, 0.5) > 0


def test_attribution_shares_and_flag():
    curves = CURVES
    nominal = lambda d, a: abs(tc.wrench(tc.split(d)[1] * a * np.array(tc.full_column(d, [0, 0])))[2])
    unit = 3.0
    survey = {'yaw+': dict(amplitude=0.3, column=[0, 0, 1], onset=unit * nominal('yaw+', 0.3), cut=-unit * nominal('yaw+', 0.3),
                           steady=0.2, sigma=0.01),
              'surge+': dict(amplitude=0.6, column=[1, 0.1, 0], onset=unit * 0.01, cut=-unit * 0.01 - 0.5,
                             steady=0.2, sigma=0.01)}
    out = tc.attribution(survey, curves, S)
    assert out['surge+']['onset_vs_cut'] and not out['yaw+']['onset_vs_cut']
    assert out['yaw+']['hull_share'] is None and 0.9 < out['surge+']['hull_share'] <= 1.0


def test_record_builds_both_matrices_and_keeps_identity_elsewhere():
    fit = dict(column=[1.0, 0.2, -0.1], J=[[1, 0], [0, 1]], sigma=[0.01, 0.02], trust=0.95, holds=3, limited=[False, True])
    rec = tc.record({'surge+': fit, 'sway-': dict(fit, column=[0.3, 1.0, 0.0])}, CURVES, {'vector': [0.1, 0.0]},
                    [0.01, 0.01, 0.02], stamp='t')
    assert rec['decoupling'][1][0] == 0.2 and rec['decoupling'][0][0] == 1.0
    assert rec['decoupling_negative'][0][1] == 0.3 and rec['decoupling_negative'][1][0] == 0.0
    assert set(rec['directions']) == {'surge+', 'sway-'} and rec['provenance']['stamp'] == 't'


def test_plan_surveys_then_refines_only_weak_directions():
    survey = tc.plan({}, S)
    assert [h['direction'] for h in survey[:6]] == list(tc.DIRECTIONS) and survey[6]['amplitude'] == 0.3
    fits = {d: dict(column=tc.full_column(d, [0.5, 0.5]), trust=0.95) for d in tc.DIRECTIONS}
    fits['sway+']['trust'] = 0.5
    rec = {'directions': fits}
    refine = tc.plan(rec, S)
    assert [(h['direction'], h['kind']) for h in refine] == [('sway+', 'refine'), ('sway+', 'verify')]
    assert refine[0]['amplitude'] * sum(abs(x) for x in refine[0]['column']) <= S.headroom + 1e-9
