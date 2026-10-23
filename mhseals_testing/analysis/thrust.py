"""Per axis-sign speed curves from steady holds (the model the controller inverts).

A hold drives one axis at a normalized effort e and settles at speed v. Drag in
normalized wrench units is e = d1*v + d2*v|v|, so a boat with quadratic drag is
not described by one linear gain: the secant gain errs by tens of percent away
from the amplitudes it was fitted at. Two or more levels per axis-sign give the
curve. ROS-free; see thrust.md section 1 ("Drag and gain").
"""
from __future__ import annotations

import numpy

from mhseals_hardware.thruster_mixer import AXES

ACTIVE = 0.02  # an axis counts as commanded above this effort


def speed_curve(points):
    """Fit e = d1*v + d2*v|v| to (speed, effort) points; both terms stay >= 0.

    Returns None without a usable point. A negative term (noise, or a single
    level) is dropped and the other refitted, so the curve is always monotone.
    """
    points = [(v, e) for v, e in points if v > 0 and e > 0]
    if not points:
        return None
    v, e = (numpy.array(column, dtype=float) for column in zip(*points))
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


def speed_curves(samples):
    """{axis: {'positive'|'negative': curve}} from single-axis (command, velocity) samples.

    ``samples`` are normalized (command, velocity) tuples; samples where another
    axis is commanded are skipped, so cross coupling never enters the curve.
    """
    levels = {}
    for command, velocity in samples:
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
            curves.setdefault(AXES[i], {})['positive' if sign > 0 else 'negative'] = curve
    return curves
