"""Fits for the physics characterization steps (procedures/physics.py). ROS-free.

Every fit returns its numbers with the counts, residuals and covariance behind them, so the crane
gate can judge it; none of them drops a term or guesses an attribution (crane-physics-design 4).
"""
from __future__ import annotations

import math

G = 9.80665


def bifilar_iz(mass_kg, period_s, separation_m, length_m):
    """Yaw inertia from a bifilar swing: I = m g d^2 T^2 / (16 pi^2 L), d the string separation."""
    return mass_kg * G * separation_m ** 2 * period_s ** 2 / (16 * math.pi ** 2 * length_m)
