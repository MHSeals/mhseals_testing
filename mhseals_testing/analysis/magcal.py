"""Compass calibration maths from handheld bearings (gps spec 4.3). ROS-free.

Headings are degrees clockwise from north. The handheld compass reads magnetic
north, so true heading = bearing + declination (COMPASS_DEC, east positive).
"""
from __future__ import annotations

import math

import numpy

PASS_DEG = 5.0  # every verify heading within this counts as a pass
MIN_SPREAD_DEG = 30.0  # headings closer than this give an ill-conditioned fit


def wrap(angle):
    """Wrap degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def true_heading(bearing, declination=0.0):
    """True heading in [0, 360) from a handheld magnetic bearing."""
    return (bearing + declination) % 360.0


def magcal_errors(ekf_yaws, bearings, declination=0.0):
    """Per-heading error ekf - true, wrapped, and whether all are within PASS_DEG."""
    errors = [wrap(y - true_heading(b, declination)) for y, b in zip(ekf_yaws, bearings)]
    return errors, bool(errors) and all(abs(e) <= PASS_DEG for e in errors)


def four_heading_fit(headings, fields):
    """Residual hard iron from the corrected horizontal field at known true headings.

    fields are (bx, by) in mG, body frame x forward y right. With the horizontal
    earth field h = (north, east) and offset o, b = R(heading)^T h + o; four
    unknowns, two equations per heading, so three well spread headings suffice.
    Returns the (dx, dy) to add to COMPASS_OFS_X/Y (-o), or None when the headings
    are too clustered or the fit is rank deficient.
    """
    if len(headings) != len(fields) or len(headings) < 3:
        return None
    if max(abs(wrap(a - b)) for a in headings for b in headings) < MIN_SPREAD_DEG:
        return None
    rows, rhs = [], []
    for psi, (bx, by) in zip(headings, fields):
        c, s = math.cos(math.radians(psi)), math.sin(math.radians(psi))
        rows += [[c, s, 1, 0], [-s, c, 0, 1]]
        rhs += [bx, by]
    a = numpy.array(rows)
    if numpy.linalg.matrix_rank(a) < 4:
        return None
    solution = numpy.linalg.lstsq(a, numpy.array(rhs, dtype=float), rcond=None)[0]
    return -float(solution[2]), -float(solution[3])
