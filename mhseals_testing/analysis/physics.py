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


def bollard_side(points, zero_n):
    """One thruster side from (pulse offset us > 0, force N) readings: T = K u^2, u = offset / 400.

    Readings at or below ``zero_n`` (the scale resolution) are inside the deadband: ``db_us`` is the
    midpoint between the largest such offset and the smallest offset that pulled (None without a
    zero reading). K is the force at full pulse, fitted over the pulling readings, with its standard
    error and the residual SD; with one pulling reading the error is None (0 dof), never 0.
    """
    points = sorted((float(o), float(f)) for o, f in points if o > 0 and math.isfinite(f))
    if not points:
        return None

    zeros = [o for o, f in points if f <= zero_n]
    pulling = [(o, f) for o, f in points if f > zero_n]
    if not pulling:
        return dict(K=0.0, K_se=None, residual_sd=None, n=len(points), dof=0, db_us=None)
    first = min(o for o, _ in pulling)
    below = [o for o in zeros if o < first]
    u = numpy.array([o / 400 for o, _ in pulling])
    f = numpy.array([f for _, f in pulling])
    basis = u * u
    k = float(basis @ f / (basis @ basis))
    dof = len(pulling) - 1
    residual = f - k * basis
    sd = float(numpy.sqrt(residual @ residual / dof)) if dof > 0 else None
    return dict(K=k, K_se=sd / float(numpy.sqrt(basis @ basis)) if sd is not None else None, residual_sd=sd,
                n=len(points), dof=dof, db_us=(max(below) + first) / 2 if below else None,
                residuals=[float(r) for r in residual])


def bollard_fit(readings, zero_n):
    """Per output: forward (pulse > 1500) and reverse sides, and the deadband centre offset o_us.

    ``readings`` are (output, pulse us, force N), ``zero_n`` the scale resolution in N. The sides
    are motor directions; which one pushes the boat forward is the record's ``reversed`` flag, so
    K_r / K_f is a measurement here, never assumed.
    """
    result = {}
    for output in sorted({r[0] for r in readings}):
        rows = [(p - 1500, f) for o, p, f in readings if o == output]
        forward = bollard_side([(d, f) for d, f in rows if d > 0], zero_n)
        reverse = bollard_side([(-d, f) for d, f in rows if d < 0], zero_n)
        offset = None
        if forward and reverse and forward['db_us'] is not None and reverse['db_us'] is not None:
            offset = (forward['db_us'] - reverse['db_us']) / 2
        ratio = reverse['K'] / forward['K'] if forward and reverse and forward['K'] > 0 else None
        result[int(output)] = dict(forward=forward, reverse=reverse, o_us=offset, reverse_ratio=ratio)
    return result
