"""Per axis-sign speed curves from steady holds (the model the controller inverts).

A hold drives one axis at a normalized effort e and settles at speed v. Drag in
normalized wrench units is w = e|e| = d1*v + d2*v|v| (thrust goes with e|e|, so the
effort itself is sqrt of that), and a boat with quadratic drag is
not described by one linear gain: the secant gain errs by tens of percent away
from the amplitudes it was fitted at. Two or more levels per axis-sign give the
curve. ROS-free; the maths is in docs/math.pdf, "Speed curves".

Also the measurement of one hold (thrust spec 4): ``step_response``, ``body_velocity`` and
``residual``, which turns a hold into main and cross responses with a trust.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy

from mhseals_hardware.thruster_mixer import AXES

ACTIVE = 0.02  # an axis counts as commanded above this effort


def speed_curve(points):
    """Fit w = e^2 = d1*v + d2*v|v| to (speed, effort) points; both terms stay >= 0.

    Returns None without a usable point. A negative term (noise, or a single
    level) is dropped and the other refitted, so the curve is always monotone.
    """
    points = [(v, e) for v, e in points if math.isfinite(v) and math.isfinite(e) and v > 0 and e > 0]
    if not points:
        return None
    v, e = (numpy.array(column, dtype=float) for column in zip(*points))
    e = e * e  # wrench w = e|e|, the quantity that is linear in drag
    basis = (v, v * v)
    d1, d2 = numpy.linalg.lstsq(numpy.column_stack(basis), e, rcond=None)[0] \
        if len(points) > 1 and len(set(v)) > 1 else (0.0, -1.0)
    if d2 < 0 or d1 < 0:
        # One term only: linear unless the linear term is the one that went negative.
        index = 1 if d1 < 0 and d2 >= 0 else 0
        coefficient = float(basis[index] @ e / (basis[index] @ basis[index]))
        d1, d2 = (coefficient, 0.0) if index == 0 else (0.0, coefficient)
    return dict(d1=float(d1), d2=float(d2), max_speed=float(v.max()),
                points=[[float(a), float(b)] for a, b in sorted(points)])


def body_drift(drift, yaw):
    """The odom-frame drift (x, y) seen in the body frame at ``yaw``: (surge, sway, 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (drift[0] * c + drift[1] * s, -drift[0] * s + drift[1] * c, 0.0)


def speed_curves(samples, drift=None):
    """{'surge+'..'yaw-': curve} (the keys hardware configuration expects) from single-axis samples.

    ``samples`` are (command, velocity, yaw) tuples; samples where another axis
    is commanded are skipped, so cross coupling never enters the curve. ``drift``
    is the odom-frame current (x, y) from the same fit: it is projected onto the
    body at each sample's yaw and removed, so a hold's speed is the boat's
    through the water, not over the ground.
    """
    levels = {}
    for command, velocity, yaw in samples:
        if drift and yaw is not None:
            velocity = [v - d for v, d in zip(velocity, body_drift(drift, yaw))]
        active = [i for i, value in enumerate(command) if abs(value) > ACTIVE]
        if len(active) != 1:
            continue
        i = active[0]
        sign = 1 if command[i] > 0 else -1
        levels.setdefault((i, sign, round(abs(command[i]), 2)), []).append(sign * velocity[i])
    result = {}
    for (i, sign, level), values in sorted(levels.items()):
        result.setdefault((i, sign), []).append((float(numpy.mean(values)), level))
    curves = {}
    for (i, sign), points in result.items():
        curve = speed_curve(points)
        if curve:
            # Only the fitted terms go to hardware.yaml; points and max_speed stay in speed_curve.
            curves[AXES[i] + ('+' if sign > 0 else '-')] = dict(d1=curve['d1'], d2=curve['d2'])
    return curves


# --- one hold: step response, body velocity, residual trust ---------------------------------

TAU_YAW = 0.015    # rad/s of yaw rate per m/s of translation (about 1 deg/s at field speed)
TAU_CRAB = 0.05    # m/s of cross translation per m/s of main
TAU_LEVER = 0.10   # m: rotation-centre offset |v|/|r| a yaw hold may show
EDGE_S = 0.5       # window after thrust on and cut for the onset and cut rates


@dataclass
class Hold:
    """One steady hold on ``direction`` ('surge+'..'yaw-') at effort ``amplitude``.

    ``vel_world`` is (E, N) in m/s and ``gyro`` the yaw rate in rad/s (left positive) at times
    ``t``. ``yaw`` is the compass heading (rad, clockwise from north) at the preceding neutral,
    thrust off; the heading during the hold is that plus the gyro integral from ``t_on``.
    The measurement window is ``t_settled`` to ``t_end``; thrust is cut at ``t_cut``.
    """
    direction: str
    amplitude: float
    column: tuple
    t: numpy.ndarray
    gyro: numpy.ndarray
    vel_world: numpy.ndarray
    yaw: float
    t_on: float
    t_settled: float
    t_cut: float
    t_end: float


@dataclass
class Step:
    t95: float | None       # s after t_on to reach 95 % of the steady change, None if never
    steady: float           # mean over the last third of the hold
    onset_rate: float       # slope in the first EDGE_S after t_on (yaw accel for a gyro)
    cut_rate: float         # slope in the first EDGE_S after t_cut


@dataclass
class Residual:
    main: float             # signed-positive response of the commanded axis (m/s or rad/s)
    cross: numpy.ndarray    # (2,) mean of the other axes, in index order, body frame
    se: numpy.ndarray       # (2,) sigma_k: wave floor plus heading or lever error
    trust: float            # P(every cross entry inside tolerance)
    limited: numpy.ndarray  # (2,) bool: 2*sigma_k > tau_k*|main|, so noise sets the entry


def _slope(t, y, low, high):
    inside = (t >= low) & (t <= high)
    return float(numpy.polyfit(t[inside], y[inside], 1)[0]) if inside.sum() >= 2 else math.nan


def step_response(t, y, t_on, t_cut):
    """Rise time, steady value and edge rates of ``y`` for thrust on at ``t_on`` and cut at ``t_cut``.

    The change is measured from the mean before ``t_on`` to the mean over the last third of the hold.
    """
    t, y = numpy.asarray(t, float), numpy.asarray(y, float)
    before = y[t < t_on]
    base = float(before.mean()) if before.size else 0.0
    hold = (t >= t_on) & (t <= t_cut)
    tail = hold & (t >= t_cut - (t_cut - t_on) / 3)
    steady = float(y[tail].mean())
    t95 = None
    if steady != base:
        reached = numpy.nonzero(hold & ((y - base) / (steady - base) >= 0.95))[0]
        t95 = float(t[reached[0]] - t_on) if reached.size else None
    return Step(t95, steady, _slope(t, y, t_on, t_on + EDGE_S), _slope(t, y, t_cut, t_cut + EDGE_S))


def body_velocity(hold: Hold, drift, lever_arm=(0.0, 0.0)):
    """(n, 3) surge, sway-left, yaw rate of the rotation centre, with the world drift removed.

    ``drift`` is a Drift or an (E, N) vector. The antenna at ``lever_arm`` (x forward, y left, m)
    moves at v_c + r x lever, which is taken off before the result is the centre's velocity.
    """
    vector = numpy.asarray(getattr(drift, 'vector', drift), float)
    t, r = hold.t, hold.gyro
    area = numpy.concatenate([[0.0], numpy.cumsum(numpy.diff(t) * (r[1:] + r[:-1]) / 2)])
    heading = hold.yaw - (area - numpy.interp(hold.t_on, t, area))   # clockwise heading falls as the boat turns left
    east, north = (hold.vel_world - vector).T
    forward = east * numpy.sin(heading) + north * numpy.cos(heading)
    left = -east * numpy.cos(heading) + north * numpy.sin(heading)
    return numpy.column_stack([forward + r * lever_arm[1], left - r * lever_arm[0], r])


def _phi(x):
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))


def residual(hold: Hold, drift, floor, sigma_yaw, lever_arm=(0.0, 0.0), sigma_lever=0.05):
    """Main and cross response of a hold over its window, with the trust of the column (thrust spec 2).

    ``floor`` is conditions.wave_floor (yaw, surge, sway); ``sigma_yaw`` the heading error in rad,
    which leaks the main translation into the other translation entry; ``sigma_lever`` (m) does the
    same for yaw holds. The trust is the product over the two cross entries of
    P(|y_k| < tau_k |main|) for y_k ~ N(measured, sigma_k).
    """
    axis = [*AXES].index(hold.direction[:-1])
    sign = 1 if hold.direction[-1] == '+' else -1
    window = (hold.t >= hold.t_settled) & (hold.t <= hold.t_end)
    mean = body_velocity(hold, drift, lever_arm)[window].mean(axis=0)
    main = sign * float(mean[axis])
    wave = [floor[1], floor[2], floor[0]]            # wave_floor is (yaw, surge, sway); axes are (surge, sway, yaw)
    others = [i for i in range(3) if i != axis]
    tau, var = [], []
    for i in others:
        if axis == 2:                                # yaw hold: a surge or sway speed is a rotation-centre offset
            tau.append(TAU_LEVER * abs(main))
            var.append(wave[i] ** 2 + (sigma_lever * main) ** 2)
        elif i == 2:                                 # translation hold: yaw rate
            tau.append(TAU_YAW * abs(main))
            var.append(wave[i] ** 2)
        else:                                        # translation hold: crab, heading error rotates the main speed
            tau.append(TAU_CRAB * abs(main))
            var.append(wave[i] ** 2 + (sigma_yaw * main) ** 2)
    cross, se, tau = mean[others], numpy.sqrt(var), numpy.array(tau)
    trust = 1.0
    for y, s, bound in zip(cross, se, tau):
        trust *= _phi((bound - y) / s) - _phi((-bound - y) / s) if s > 0 else float(abs(y) <= bound)
    return Residual(main, cross, se, float(trust), 2 * se > tau)
