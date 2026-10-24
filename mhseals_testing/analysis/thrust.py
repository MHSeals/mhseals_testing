"""Per axis-sign speed curves from steady holds (the model the controller inverts).

A hold drives one axis at a normalized effort e and settles at speed v. Drag in
normalized wrench units is w = e|e| = d1*v + d2*v|v| (thrust goes with e|e|, so the
effort itself is sqrt of that), and a boat with quadratic drag is
not described by one linear gain: the secant gain errs by tens of percent away
from the amplitudes it was fitted at. Two or more levels per axis-sign give the
curve. ROS-free; the maths is in docs/math.pdf, "Speed curves".

Also ``step_response``, the rise time and edge rates of one hold's signal.
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


# --- one hold: step response ----------------------------------------------------------------

EDGE_S = 0.5       # window after thrust on and cut for the onset and cut rates


@dataclass
class Step:
    t95: float | None       # s after t_on to reach 95 % of the steady change, None if never
    steady: float           # mean over the last third of the hold
    onset_rate: float       # slope in the first EDGE_S after t_on (yaw accel for a gyro)
    cut_rate: float         # slope in the first EDGE_S after t_cut


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
