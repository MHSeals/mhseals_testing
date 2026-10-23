"""Compass interference from the thrusters: field change against thruster effort.

ArduRover has no compassmot and its current sensor reads 0, so the suite fits
the Here4 field change dB (mG, body xy) against each output's effort itself
(gps spec 4.4). ROS-free.

``fit`` is the calibration fit: dB = sum k_i * u_i per axis, sign-split (u+ and
u- get their own k, because current follows effort magnitude but a reversed
thruster does not mirror). ``LiveFit`` is the rolling version behind the live
row: dB against |u| only, since it sees whatever the operator is doing.
"""
from __future__ import annotations

import math
from collections import deque

import numpy

HORIZONTAL_MG = 235.0  # horizontal Earth field at the site; 10 mG is about 2.4 degrees
OUTPUTS = 4


def heading_error(k, field):
    """Degrees of heading error from a field change ``k`` (mG, xy) given the field (xy)."""
    norm = math.hypot(*field)
    if norm == 0 or not all(math.isfinite(v) for v in (*k, *field)):
        return float('nan')
    perpendicular = abs(field[0] * k[1] - field[1] * k[0]) / norm
    return math.degrees(math.atan2(perpendicular, HORIZONTAL_MG))


def _solve(columns, db):
    """Least squares per axis; returns a (columns, 2) array."""
    return numpy.linalg.lstsq(numpy.asarray(columns, float), numpy.asarray(db, float), rcond=None)[0]


def fit(samples, field):
    """Sign-split fit of ``samples`` = [(efforts[4], (dBx, dBy))] against ``field`` = (Bx, By).

    Returns per output ``{'pos': (kx, ky), 'neg': (kx, ky), 'error_deg': worst heading
    error at full effort}`` with k in mG per 100% effort, or None without enough samples.
    A sign that was never driven gets k = 0 (lstsq's minimum-norm answer).
    """
    samples = [(u, b) for u, b in samples
               if len(u) == OUTPUTS and all(math.isfinite(v) for v in (*u, *b))]
    if len(samples) < 2 * OUTPUTS:
        return None
    columns = [[max(v, 0.0) for v in u] + [max(-v, 0.0) for v in u] for u, _ in samples]
    solution = _solve(columns, [b for _, b in samples])
    rows = []
    for i in range(OUTPUTS):
        pos, neg = tuple(solution[i]), tuple(solution[OUTPUTS + i])
        rows.append(dict(pos=pos, neg=neg,
                         error_deg=max(heading_error(pos, field), heading_error(neg, field))))
    return rows


def verdict(rows):
    """gps 4.4 thresholds on the worst output: accept under 3 degrees, route 3-10, move the compass above."""
    worst = max(row['error_deg'] for row in rows)
    if worst < 3:
        return 'ACCEPT', worst
    return ('REROUTE' if worst <= 10 else 'MOVE COMPASS'), worst


class LiveFit:
    """Rolling dB-vs-|effort| fit for the live row (window of the last ``size`` samples)."""

    def __init__(self, size=300):
        self.samples = deque(maxlen=size)

    def add(self, efforts, db):
        if len(efforts) == OUTPUTS and all(math.isfinite(v) for v in (*efforts, *db)):
            self.samples.append(([abs(v) for v in efforts], tuple(db)))

    def degrees(self, efforts, field):
        """Predicted heading error at ``efforts`` now, or None until the fit is determined.

        Needs more samples than unknowns and every output seen at some effort; an
        output never driven would make the fit meaningless, not zero.
        """
        if len(self.samples) < 3 * OUTPUTS:
            return None
        columns = [u for u, _ in self.samples]
        if min(max(column) for column in zip(*columns)) < 0.05:
            return None
        k = _solve(columns, [b for _, b in self.samples])
        predicted = numpy.abs(efforts) @ k
        return heading_error(tuple(predicted), field)
