"""Conditions verdict: good, marginal, poor or unknown, from the thrust ceiling and drift trust.

ROS-free. Thresholds are the ``verdict`` group of config/conditions.yaml, the single source;
health gates on the verdict this returns (conditions spec 2.5 and 3).
"""
from __future__ import annotations

from pathlib import Path

import yaml

from mhseals_testing.analysis.conditions import _config_path

MEASURES = ('ceiling', 'drift_trust')       # fractions in 0..1; None while not yet measured


def load_thresholds(path=None) -> dict:
    """``{'good': {...}, 'poor': {...}}`` from the YAML; ValueError naming the file for anything unusable."""
    path = Path(path or _config_path())
    raw = (yaml.safe_load(path.read_text()) or {}).get('verdict')
    try:
        out = {grade: {m: float(raw[grade][m]) for m in MEASURES} for grade in ('good', 'poor')}
    except (TypeError, KeyError, ValueError):
        raise ValueError(f'{path}: verdict needs good and poor, each with {" and ".join(MEASURES)}') from None
    if any(out['poor'][m] > out['good'][m] for m in MEASURES):
        raise ValueError(f'{path}: verdict poor thresholds must not exceed good')
    return out


def verdict(ceiling, drift_trust, thresholds=None) -> tuple[str, str]:
    """(verdict, reason). Unknown if either measure is missing; poor if either is under its poor
    threshold; good only if both reach their good threshold; otherwise marginal."""
    thresholds = thresholds or load_thresholds()
    values = dict(zip(MEASURES, (ceiling, drift_trust)))
    if any(v is None for v in values.values()):
        return 'unknown', 'not measured yet'
    for grade, below in (('poor', 'poor'), ('good', 'marginal')):
        low = [f'{m} {values[m]:.0%} < {thresholds[grade][m]:.0%}'
               for m in MEASURES if values[m] < thresholds[grade][m]]
        if low:
            return below, ', '.join(low)
    return 'good', ''
