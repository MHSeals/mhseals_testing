"""Trust ceilings and drift trust under today's water, and the verdict (conditions spec 2).

ROS-free. The ceiling is the trust a perfect column would get: thrust's per-entry trust
``2*Phi(tau*|V|/sigma) - 1`` with the cross response ``y = 0``, multiplied over the gating entries
of a direction (psi for translations, the rotation centre for yaw rows; crab is shown elsewhere and
never gates). The gate value is the minimum over the *planned* directions only, so a one-direction
rerun is rated on its own direction. Thresholds are ``tolerance`` and ``verdict`` in
config/conditions.yaml and ``drift.practical`` (tau_d) from the estimator settings.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import erf, exp, hypot, sqrt
from pathlib import Path

import numpy as np
import yaml

from mhseals_testing.analysis import conditions, verdict as grading


@dataclass(frozen=True)
class Direction:
    """One signed direction at its main response ``speed`` (m/s, or rad/s for yaw), e.g. ``sway-``."""
    name: str
    axis: str           # surge, sway or yaw
    speed: float        # |V| at a_hi, from the speed curves or the plan


def load_tolerances(path=None) -> dict:
    """``{'psi': ..., 'ell': ...}`` from the YAML; ValueError naming the file for anything unusable."""
    path = Path(path or conditions._config_path())
    raw = (yaml.safe_load(path.read_text()) or {}).get('tolerance')
    try:
        out = {k: float(raw[k]) for k in ('psi', 'ell')}
    except (TypeError, KeyError, ValueError):
        raise ValueError(f'{path}: tolerance needs psi and ell') from None
    if min(out.values()) <= 0:
        raise ValueError(f'{path}: tolerance values must be positive')
    return out


def entry_trust(tolerance, speed, sigma) -> float:
    """``2*Phi(tau*|V|/sigma) - 1``: the chance one entry lands inside its band when the true value is zero."""
    if sigma <= 0:
        return 1.0
    return erf(tolerance * abs(speed) / sigma / sqrt(2))


def drift_sigma(est, now, floor=0.0) -> float | None:
    """``sigma_e = sqrt(se^2 + (beta*dt)^2)`` in m/s: pool error plus the trend since the pool's centroid."""
    if est.drift is None:
        return None
    centre = np.mean([w.t0 + w.duration / 2 for w in est.pool])
    beta = hypot(*est.trend) if est.trend is not None else 0.0
    return hypot(est.drift.se, beta * max(0.0, now - centre), floor)


def drift_trust(sigma_e, tau_d) -> float | None:
    """``T_d = 1 - exp(-tau_d^2 / (2 sigma_e^2))``, the Rayleigh form for an isotropic 2-D error."""
    if sigma_e is None:
        return None
    return 1.0 if sigma_e <= 0 else 1 - exp(-tau_d ** 2 / (2 * sigma_e ** 2))


def direction_ceiling(d: Direction, floor, sigma_e, tol, gyro_bias_se=0.0) -> float:
    """Product of the gating entries' trust for one direction.

    ``floor`` is ``wave_floor`` (yaw, surge, sway). A translation gates on psi, whose sigma is the gyro
    floor with the bias error added: immune to drift. A yaw row gates on the rotation centre, the surge
    and sway velocity per unit yaw rate: wave floor plus the drift error, tolerance ``ell*|r|``.
    """
    if d.axis in ('surge', 'sway'):
        return entry_trust(tol['psi'], d.speed, hypot(floor[0], gyro_bias_se))
    return float(np.prod([entry_trust(tol['ell'], d.speed, hypot(f, sigma_e or 0.0)) for f in floor[1:]]))


def ceilings(planned, floor, sigma_e, tol, gyro_bias_se=0.0) -> dict:
    """``{direction name: ceiling}`` for the planned directions only."""
    return {d.name: direction_ceiling(d, floor, sigma_e, tol, gyro_bias_se) for d in planned}


def rate(est, planned, now, tol=None, settings=None, thresholds=None, gyro_bias_se=0.0):
    """The conditions verdict as ``(verdict, reason, ceiling, drift_trust)``.

    ``ceiling`` is the minimum over ``planned``. Poor when the drift is materially trending or changed (a significant but
    sub-practical one is not), even with no drift estimate; unknown without one; else the graded
    ceiling and drift trust, the reason naming the limiting direction.
    """
    settings = settings or conditions.load_settings()
    tol = tol or load_tolerances()
    if est.material(settings):
        return 'poor', f'{est.state} {est.size:.3f} m/s', None, None
    floor = conditions.wave_floor(est.pool, settings) if est.drift is not None else None
    if floor is None or not planned:
        return 'unknown', 'not measured yet', None, None
    sigma_e = drift_sigma(est, now)
    by_direction = ceilings(planned, floor, sigma_e, tol, gyro_bias_se)
    worst = min(by_direction, key=by_direction.get)
    ceiling, trust = by_direction[worst], drift_trust(sigma_e, settings.drift_practical)
    grade, reason = grading.verdict(ceiling, trust, thresholds)
    return grade, reason.replace('ceiling', f'{worst} ceiling', 1), ceiling, trust
