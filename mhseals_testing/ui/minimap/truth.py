"""Check a map against simulator truth (map.md section 11.3); pure numpy, no ROS.

The map frame (navsat ENU) and the simulator world frame differ by a rigid 2-D
transform. `umeyama` fits it from the boat's truth odometry against the trail
crumbs of the first seconds; the checks then compare in map cells, the same raster
the canvas uses, so "within 1 cell" means what the operator sees.
"""

import json
import math
from pathlib import Path

import numpy as np

ALIGN_SECONDS = 10.0  # window of the first samples used to fit the transform
MIN_SCORE = 0.5  # tracks below this are not held to the truth
TRAIL_SHARE = 0.95  # share of trail dots that must lie near the truth track
CELL_TOLERANCE = 1  # cells, Chebyshev


def read_jsonl(path):
    """Rows of a JSON-lines file; torn or blank lines are skipped."""
    rows = []
    for line in Path(path).read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows


def umeyama(src, dst):
    """Rigid transform ``(R, t)`` with ``dst ~ R @ src + t`` for (n, 2) point arrays (no scale)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if len(src) < 2 or len(src) != len(dst):
        raise ValueError('need at least two paired points')
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    u, _, vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    flip = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    rot = u @ flip @ vt
    return rot, mu_d - rot @ mu_s


def align(odom, crumbs, seconds=ALIGN_SECONDS):
    """World -> map transform from odometry rows against crumbs, paired by nearest time.

    Rows are dicts with ``t``, ``x``, ``y``; only samples in the first ``seconds`` of
    the crumbs are used.
    """
    t0 = crumbs[0]['t']
    early = [c for c in crumbs if c['t'] - t0 <= seconds]
    times = np.array([o['t'] for o in odom])
    pairs = [(odom[int(np.abs(times - c['t']).argmin())], c) for c in early]
    return umeyama([[o['x'], o['y']] for o, _ in pairs], [[c['x'], c['y']] for _, c in pairs])


def apply(transform, points):
    rot, shift = transform
    return np.asarray(points, float) @ rot.T + shift


def cell_of(point, centre, half_width, cols, rows):
    """Canvas cell (col, row) of a map point, row 0 at the top; dots are square, cells 2 x 4 dots."""
    dot = half_width / cols  # m per dot: 2 * half_width over 2 * cols dots
    return (math.floor((cols + (point[0] - centre[0]) / dot) / 2),
            math.floor((rows * 2 - (point[1] - centre[1]) / dot) / 4))


def near(a, b, tolerance=CELL_TOLERANCE):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= tolerance


def object_misses(tracks, truth, cell, min_score=MIN_SCORE):
    """Tracks at ``min_score`` or more with no same-label truth element within the cell tolerance.

    ``cell`` maps a map point to a cell; ``truth`` is a list of (label, point) already
    in the map frame. An empty result passes.
    """
    return [t for t in tracks if t['score'] >= min_score and not any(
        label == t['label'] and near(cell((t['x'], t['y'])), cell(p)) for label, p in truth)]


def trail_share(crumbs, track, cell):
    """Share of crumbs whose cell is within tolerance of a cell of the truth track."""
    cells = {cell(p) for p in track}
    if not crumbs:
        return 1.0
    hits = sum(any(near(cell((c['x'], c['y'])), k) for k in cells) for c in crumbs)
    return hits / len(crumbs)
