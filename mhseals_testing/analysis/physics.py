"""Fits for the physics characterization steps (procedures/physics.py). ROS-free.

Every fit returns its numbers with the counts, residuals and covariance behind them, so the crane
gate can judge it; none of them drops a term or guesses an attribution (crane-physics-design 4).
"""
from __future__ import annotations

import math

import numpy

G = 9.80665


def bifilar_iz(mass_kg, period_s, separation_m, length_m):
    """Yaw inertia from a bifilar swing: I = m g d^2 T^2 / (16 pi^2 L), d the string separation."""
    return mass_kg * G * separation_m ** 2 * period_s ** 2 / (16 * math.pi ** 2 * length_m)


def channel_noise(times, values):
    """Per-sample SD of one channel after removing a straight line (a tied boat swings slowly).

    Returns {sigma, n, lag1}: ``lag1`` is the lag-1 autocorrelation of the residual, so a reader can
    tell white sensor noise (near 0) from slow motion left in it. None with fewer than 10 samples.
    """
    pairs = [(t, v) for t, v in zip(times, values) if v is not None and math.isfinite(v)]
    if len(pairs) < 10:
        return None
    t, v = (numpy.array(column, float) for column in zip(*pairs))
    residual = v - numpy.polyval(numpy.polyfit(t - t[0], v, 1), t - t[0])
    sigma = float(numpy.sqrt(residual @ residual / (len(residual) - 2)))
    lag1 = float(residual[1:] @ residual[:-1] / (residual @ residual)) if residual.any() else 0.0
    return dict(sigma=sigma, n=len(pairs), lag1=lag1)


def env_floor(wave_floor, sigma):
    """(environment SD, ok): sqrt(wave_floor^2 - sigma^2); ok is False when the sensor alone exceeds the floor."""
    excess = wave_floor ** 2 - sigma ** 2
    return math.sqrt(max(excess, 0.0)), excess >= 0
