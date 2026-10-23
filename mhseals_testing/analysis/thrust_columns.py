"""Iterate the per-direction coupling columns, attribute the yaw and record the fit (thrust math note, T21).

ROS-free. A direction is an axis and a sign, e.g. ``sway-``. Its column ``c`` has ``c_jj = 1``
and two cross entries (the other axes, in index order): a hold drives the thrusters with the
efforts ``s * a * c``. The fit drives the steady cross response ``y`` to zero. The sensitivity of
``y`` to the column entries is ``J = A_c^-1 dW/dc``, with ``dW/dc`` the local slope of the
thruster wrench and ``A_c`` the cross block of the boat's linearized damping and Coriolis
matrix at the MEASURED state; Broyden updates correct it when a step disagrees with it by more
than the gate. All velocities are in the body frame; speed curves are the dicts of
``thrust.speed_curve`` (wrench = d1 v + d2 v|v|).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from mhseals_hardware.thruster_mixer import AXES, THRUSTER_MIXER

DIRECTIONS = tuple(axis + sign for axis in AXES for sign in '+-')


@dataclass(frozen=True)
class Settings:
    amplitude: dict
    headroom: float
    reverse_ratio: float
    tolerance: dict
    gate: dict
    max_step: float
    trust_target: float
    refine_holds: int


def _config_path():
    source = Path(__file__).resolve().parent.parent.parent / 'config' / 'thrust.yaml'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'thrust.yaml'


def load_settings(path=None) -> Settings:
    """Settings from config/thrust.yaml; one ValueError names the file and the missing key."""
    path = Path(path or _config_path())
    raw = yaml.safe_load(path.read_text())
    try:
        return Settings(**{k: raw[k] for k in Settings.__dataclass_fields__})
    except (KeyError, TypeError) as error:
        raise ValueError(f'{path}: unusable thrust settings ({error!r})') from None


def split(direction):
    """('sway-') -> (axis index 1, sign -1, the two cross axis indices)."""
    j, s = AXES.index(direction[:-1]), 1 if direction[-1] == '+' else -1
    return j, s, [k for k in range(3) if k != j]


def full_column(direction, cross):
    """The 3-vector column with 1 on the main axis and ``cross`` on the other two."""
    j, _, ks = split(direction)
    column = [0.0] * 3
    column[j] = 1.0
    for k, value in zip(ks, cross):
        column[k] = float(value)
    return column


def wrench(effort, mixer=THRUSTER_MIXER, rho=0.78):
    """Normalized wrench M^T phi(M e) / 4 of an axis-effort vector (reverse thrust scaled by rho)."""
    m = np.asarray(mixer, float)
    t = m @ np.asarray(effort, float)
    return m.T @ (np.where(t < 0, rho, 1.0) * t * np.abs(t)) / 4


def wrench_slope(direction, amplitude, cross, mixer=THRUSTER_MIXER, rho=0.78):
    """dW_cross/dc (2x2): the local slope, not the secant, of the cross wrench in the column entries."""
    j, s, ks = split(direction)
    m = np.asarray(mixer, float)
    t = m @ (s * amplitude * np.asarray(full_column(direction, cross)))
    slope = 2 * np.abs(t) * np.where(t < 0, rho, 1.0)
    return (s * amplitude) * (m[:, ks].T * slope) @ m[:, ks] / 4


def _slope(curve, v):
    """Drag slope d1 + 2 d2 |v| of a speed curve at speed v."""
    return curve['d1'] + 2 * curve['d2'] * abs(v)


def damping_block(direction, main, cross_velocity, curves, masses):
    """A_c (2x2): the cross block of damping slopes plus the Coriolis/Munk Jacobian at the measured state.

    ``main`` is the signed main-axis velocity, ``masses`` is (m_u, m_v). For a surge hold the block is
    [[d_v, m_u V], [(m_v - m_u) V, d_r]], whose determinant goes to 0 as the hull turns directionally unstable.
    """
    j, _, ks = split(direction)
    v = np.zeros(3)
    v[j] = main
    v[ks] = cross_velocity
    (u, w, r), (mu, mv) = v, masses
    coriolis = np.array([[0, -mv * r, -mv * w],
                         [mu * r, 0, mu * u],
                         [(mv - mu) * w, (mv - mu) * u, 0]])
    a = coriolis.copy()
    for i in range(3):
        a[i, i] += _slope(curves[AXES[i] + ('-' if v[i] < 0 else '+')], v[i])
    return a[np.ix_(ks, ks)]


def stability(direction, main, curves, masses):
    """det A_c at a hold's main speed with no cross motion (reported as the directional stability)."""
    return float(np.linalg.det(damping_block(direction, main, [0.0, 0.0], curves, masses)))


def entry_states(entry, direction, settings):
    """Per cross entry: 'ok' (inside tolerance), 'limited' (noise: 2 sigma > tol |V|) or 'step'."""
    tol = np.asarray(settings.tolerance[direction[:-1]], float) * abs(entry['main'])
    y, sigma = np.abs(entry['cross']), np.asarray(entry['sigma'], float)
    return ['limited' if settings.gate['noise_limited'] * sg > tl else 'ok' if yy <= tl else 'step'
            for yy, sg, tl in zip(y, sigma, tol)]


def _model_jacobian(entry, direction, curves, masses, settings):
    state = (entry['main'] * (1 if split(direction)[1] > 0 else -1), entry['cross'])
    block = damping_block(direction, state[0], state[1], curves, masses)
    slope = wrench_slope(direction, entry['amplitude'], entry['column'], rho=settings.reverse_ratio)
    return np.linalg.solve(block, slope)


def next_column(history, direction, curves, masses, settings):
    """(column, J): the next cross entries to try after the holds in ``history``, and the sensitivity used.

    ``history`` lists this direction's holds at one amplitude, oldest first; each has ``column`` (the 2
    cross entries tried), ``cross`` (the 2 cross responses), ``sigma`` (their SE), ``main`` (|main response|),
    ``main_sigma`` and ``amplitude``. J starts from the measured-state model and takes a rank-1 Broyden
    correction for a step only when it misses the prediction by more than ``broyden_sigma`` combined
    sigmas. Entries that are inside tolerance or noise-limited keep their value; a response not above
    ``dead_main_sigma`` sigmas leaves the whole column unchanged (the wiring is wrong or the hold dead).
    """
    last = history[-1]
    column = np.asarray(last['column'], float)
    j = _model_jacobian(history[0], direction, curves, masses, settings)
    for old, new in zip(history, history[1:]):
        dc = np.asarray(new['column'], float) - np.asarray(old['column'], float)
        miss = (np.asarray(new['cross'], float) - np.asarray(old['cross'], float)) - j @ dc
        sigma = np.hypot(new['sigma'], old['sigma'])
        if dc @ dc > 1e-12 and np.max(np.abs(miss) / sigma) > settings.gate['broyden_sigma']:
            j = j + np.outer(miss, dc) / (dc @ dc)
    if last['main'] <= settings.gate['dead_main_sigma'] * last['main_sigma']:
        return column.tolist(), j.tolist()
    active = [i for i, state in enumerate(entry_states(last, direction, settings)) if state == 'step']
    if not active:
        return column.tolist(), j.tolist()
    try:
        step = np.linalg.solve(j[np.ix_(active, active)], -np.asarray(last['cross'], float)[active])
    except np.linalg.LinAlgError:
        return column.tolist(), j.tolist()
    step = np.clip(step, -settings.max_step, settings.max_step)
    new = column.copy()
    new[active] += step
    budget = settings.headroom / last['amplitude'] - 1       # a (1 + sum |c|) <= headroom
    if np.abs(new).sum() > budget:
        new = new * max(budget, 0.0) / np.abs(new).sum()
    return new.tolist(), j.tolist()


def attribution(survey, curves, settings, mixer=THRUSTER_MIXER):
    """Hull share of each direction's yaw balance, from the gyro steps at thrust onset and cut (diagnostic only).

    ``survey`` maps direction -> {'amplitude', 'column' (3-vector), 'onset', 'cut' (yaw acceleration steps,
    rad/s^2), 'steady' (yaw rate), 'sigma' (SE of one step)}. The yaw holds give the scale between the
    nominal yaw wrench and the measured onset step; a translation hold's actuator torque is then
    onset / scale, and what is left of the yaw drag (from ``curves``) at its steady rate is the hull's.
    ``onset_vs_cut`` marks steps that disagree by more than 3 sigma (prop inflow or hull interaction).
    """
    def nominal(direction, hold):
        effort = split(direction)[1] * hold['amplitude'] * np.asarray(hold['column'])
        return abs(wrench(effort, mixer, settings.reverse_ratio)[2])
    yaws = [(nominal(d, h), abs(h['onset'])) for d, h in survey.items() if d.startswith('yaw')]
    scale = sum(w * o for w, o in yaws) / sum(w * w for w, _ in yaws)       # onset per unit of wrench
    out = {}
    for direction, hold in survey.items():
        flag = bool(abs(abs(hold['onset']) - abs(hold['cut'])) > 3 * hold['sigma'] * np.sqrt(2))
        share = None
        if not direction.startswith('yaw'):
            curve = curves['yaw+' if hold['steady'] >= 0 else 'yaw-']
            drag = curve['d1'] * abs(hold['steady']) + curve['d2'] * hold['steady'] ** 2
            share = float((drag - abs(hold['onset']) / scale) / drag) if drag > 0 else None
        out[direction] = dict(hull_share=share, onset_vs_cut=flag)
    return out


def record(fits, curves, drift, floor, stamp=None):
    """The dict ``update_config`` stores: C+/C- (rows are axes), curves, drift, and per-direction provenance.

    ``fits`` maps direction -> {'column' (3), 'J' (2x2), 'sigma' (2), 'trust', 'holds', 'limited' (2 bools)};
    a direction without a fit keeps the identity column.
    """
    matrices = {'+': np.eye(3), '-': np.eye(3)}
    directions = {}
    for direction in DIRECTIONS:
        fit = fits.get(direction)
        j = split(direction)[0]
        if fit:
            matrices[direction[-1]][:, j] = fit['column']
            directions[direction] = {k: fit[k] for k in ('column', 'J', 'sigma', 'trust', 'holds', 'limited')}
    return dict(decoupling=matrices['+'].tolist(), decoupling_negative=matrices['-'].tolist(),
                speed_curves=curves, drift=dict(drift), wave_floor=[float(x) for x in floor],
                directions=directions, provenance=dict(method='per-direction columns', stamp=stamp))


def plan(rec, settings):
    """The next holds: the survey (current columns, a_hi then a_lo, +/- paired) or the refine list.

    With no per-direction trust in ``rec`` the survey is returned. Otherwise only directions whose trust
    is under ``trust_target`` are refined: up to ``refine_holds`` at a_hi, then one a_lo check.
    Amplitudes shrink where the corrected column would saturate a thruster.
    """
    def amplitude(direction, level):
        a = settings.amplitude[('yaw_' if direction.startswith('yaw') else '') + level]
        return min(a, settings.headroom / sum(abs(x) for x in column(direction)))

    def column(direction):
        fit = rec.get('directions', {}).get(direction)
        return fit['column'] if fit else full_column(direction, [0, 0])

    def hold(direction, kind, level):
        return dict(direction=direction, kind=kind, amplitude=amplitude(direction, level),
                    column=column(direction))
    trusts = {d: f['trust'] for d, f in rec.get('directions', {}).items()}
    if not trusts:
        return [hold(d, 'survey', level) for level in ('hi', 'lo') for d in DIRECTIONS]
    weak = [d for d in DIRECTIONS if trusts.get(d, 0.0) < settings.trust_target]
    return ([dict(hold(d, 'refine', 'hi'), max_holds=settings.refine_holds) for d in weak]
            + [hold(d, 'verify', 'lo') for d in weak])
