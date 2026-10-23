"""Per-axis forward model that runs beside the boat on the commanded effort (thrust addendum).

ROS-free. Each axis holds a first-order lag toward the steady through-water speed the fitted
speed curve gives for the command, and the prediction adds the world drift seen in the body
frame, so it is comparable with GPS or odometry velocity. Feed ``predicted`` and the measured
speed to ``tracking.tracking_score``.
"""
from __future__ import annotations

import math

from mhseals_testing.analysis.thrust import body_drift

AXES = ('surge', 'sway', 'yaw')
LAG_PER_T95 = 1.0 / 3.0   # a first-order lag reaches 95% at ln(20) ~ 3 time constants


def steady_speed(curve, effort):
    """Signed speed where d1*v + d2*v|v| = e|e| (the inverse of speed_curve); 0 without a curve."""
    if not curve or effort == 0:
        return 0.0
    d1, d2 = curve['d1'], curve['d2']
    denom = d1 + math.sqrt(d1 * d1 + 4 * d2 * effort * effort)
    speed = 2 * effort * effort / denom if denom > 0 else 0.0   # cancellation-free root
    return math.copysign(speed, effort)


class AxisPredictor:
    """One axis: curves {'+': .., '-': ..}, lag time constant from the measured t95 (s)."""

    def __init__(self, curves, t95):
        self.curves = curves
        self.tau = max(t95 * LAG_PER_T95, 0.0)
        self.water = 0.0     # predicted through-water speed

    def step(self, effort, dt, drift=0.0):
        """Advance dt seconds on ``effort``; returns the predicted ground speed with ``drift``."""
        if not (math.isfinite(effort) and math.isfinite(dt)) or dt <= 0:
            return self.water + drift   # a bad sample must not poison the state
        target = steady_speed(self.curves.get('+' if effort >= 0 else '-'), effort)
        alpha = 1.0 if self.tau <= 0 else 1.0 - math.exp(-dt / self.tau)
        self.water += alpha * (target - self.water)
        return self.water + drift


class Predictor:
    """Surge, sway and yaw together. ``speed_curves`` is the characterization's
    {'surge+': {d1, d2}, ...}; ``t95`` maps axis name to seconds (missing means no lag)."""

    def __init__(self, speed_curves, t95):
        self.axes = [AxisPredictor({s: speed_curves.get(a + s) for s in '+-'}, t95.get(a, 0.0))
                     for a in AXES]

    def step(self, command, dt, drift=(0.0, 0.0), yaw=0.0):
        """Predicted (surge, sway, yaw) velocity; ``drift`` is the odom-frame (x, y) current."""
        body = body_drift(drift, yaw)
        return tuple(a.step(c, dt, d) for a, c, d in zip(self.axes, command, body))
