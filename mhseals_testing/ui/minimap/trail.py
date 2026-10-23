"""Replay trail (minimap_layers section 3): crumbs from ``minimap/trail.jsonl`` cut at the playhead.

Pure and ROS-free. A crumb is the dict the live owner appends
(``t_ns, x, y, epoch, fix, h_acc_m, speed_mps, xte_m``, schema ``mhseals.trail/1``).
"""

from bisect import bisect_right
import math
from pathlib import Path

from mhseals_testing import data

SCHEMA = 'mhseals.trail/1'
FILE = ('minimap', 'trail.jsonl')  # relative to the session directory

def load(session_dir):
    """Time-ordered crumbs of a session; torn lines, other schemas and non-finite fields are dropped."""
    crumbs = []
    for rec in data.read_jsonl(Path(session_dir, *FILE)):
        if rec.get('schema', SCHEMA) != SCHEMA:
            continue
        t, x, y = rec.get('t_ns'), rec.get('x'), rec.get('y')
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (t, x, y)):
            crumbs.append(rec)
    crumbs.sort(key=lambda c: c['t_ns'])
    return crumbs

def upto(crumbs, t_ns):
    """The crumbs at or before the playhead ``t_ns``: the trail never runs ahead of it."""
    return crumbs[:bisect_right([c['t_ns'] for c in crumbs], t_ns)]
