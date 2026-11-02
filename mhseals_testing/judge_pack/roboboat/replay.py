"""Reset-closed-loop replay of a higher tier's goal segments through the fast tier's plant.

A segment starts at a ``task.goal`` event of the higher-tier run (Unity or the boat) and ends at
the next goal, a stop, or 30 s (crane fastsim.fidelity windows). From the higher tier's own state at
the segment start, the fast tier's goal tracker and calibrated plant (the fast session's
``fastsim.json``: limits and plant gain/delay/speed_scale) drive to that same goal; the judge
compares the result with what the higher tier did. Same goal, same start: what remains is
dynamics and control, not decisions. Open-loop replay needs the higher tier's commands, which the
Unity pair sessions do not keep (their bags are unreadable lean captures), so that term is absent.
"""

from dataclasses import fields
import json
from pathlib import Path

import numpy as np

from mhseals_testing.judge_pack.roboboat._core import core

core()

from judge.insight.divergence import Replayer  # noqa: E402

HORIZON_S = 30.0


def fast_spec(fast_session):
    """(Limits, vehicle spec) from a fast session's fastsim.json, or None."""
    p = Path(fast_session) / 'fastsim.json'
    if not p.exists():
        return None
    from fastsim.plant import Limits
    doc = json.loads(p.read_text())
    lim = doc.get('limits') or {}
    names = {f.name for f in fields(Limits)}
    limits = Limits.from_dict({k: v for k, v in lim.items() if k in names and k != 'extra'})
    cfg = doc.get('config') or {}
    spec = {'gain': (cfg.get('plant') or {}).get('gain', cfg.get('gain', 1.0)),
            **{k: v for k, v in (cfg.get('plant') or {}).items() if k != 'gain'}}
    return limits, spec


def goal_segments(rec, horizon_s=HORIZON_S, min_s=2.0):
    goals = rec.events_of('task.goal')
    stops = [e.t for e in rec.events_of('authority', 'mission.stop')]
    t_end = rec.header.window[1]
    out = []
    for k, g in enumerate(goals):
        t0 = g.t
        nxt = [goals[k + 1].t] if k + 1 < len(goals) else []
        t1 = min([t0 + horizon_s, t_end] + [s for s in stops if s > t0] + nxt)
        if t1 - t0 >= min_s:
            out.append((t0, t1, g.payload.get('why') or 'goal'))
    return out


class FastPlantReplayer(Replayer):
    horizon_s = None

    def __init__(self, limits, spec, dt=0.05):
        self.limits, self.spec, self.dt = limits, spec, dt

    def reset_closed_loop(self, high, t0, t1):
        from fastsim.plant import Goal, vehicle
        g = [e for e in high.events_of('task.goal') if abs(e.t - t0) < 1e-6 or e.t <= t0]
        if not g:
            return None
        goal_ev = g[-1]
        p = goal_ev.payload
        if 'wx' not in p:
            return None
        pose = high.channel('pose.se2').at([t0])[0]
        tw = high.channel('twist.body').at([t0])[0]
        tracker, plant = vehicle(self.spec, self.limits, tuple(pose), tuple(tw))
        goal = Goal([(p['wx'], p['wy'], p['wyaw'])], bool(p.get('precise')))
        ts, xy, t = [], [], t0
        while t <= t1 + 1e-9:
            ts.append(t)
            xy.append(plant.pose[:2])
            plant.step(tracker.command(goal, plant.pose, t), self.dt)
            t += self.dt
        return np.array(ts), np.array(xy)
