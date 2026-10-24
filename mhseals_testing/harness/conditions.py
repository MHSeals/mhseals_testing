"""Live water conditions: a ring of neutral samples rated by the estimator (conditions spec 3).

ROS-free. The ingest path calls ``feed`` per sample on one clock and ``cut`` when a non-zero command
ends; the health snapshot reads ``facts`` and a procedure reads ``evaluate``. Nothing here moves
the boat, and callers use the verdict only to decide whether to *start* a procedure.

``ceiling(estimate, windows)`` is the planned-direction trust ceiling (analysis/readiness). Without
one the ceiling is not limiting and ``conditions.ceiling`` reads None, so the verdict rests on drift.
``planned()`` (the Directions about to be run) builds that ceiling live: the minimum over them (spec 2.2). Drift trust
uses sigma_e^2 = se^2 + (beta*dt)^2 (spec 2.1), so a trending drift lowers it.
"""
from __future__ import annotations

from collections import deque
import math
import time

import numpy as np

from mhseals_testing.analysis import conditions as est, readiness
from mhseals_testing.analysis.verdict import load_thresholds, verdict
from mhseals_testing.health.model import Fact

REFRESH_S = 1.0         # the estimate is recomputed at most this often
MAX_HZ = 8.0            # cap above the 5 Hz GPS rate (jittered real samples are never dropped); only a faster feed (sim odometry) is thinned
NEEDED_WINDOWS = 3      # the estimator's minimum, for the "measuring" text


FLAGS = {'sparse': 'gap', 'spin': 'turn', 'coast': 'coast'}     # estimator flag -> spec 5.1 name


def _detail(estimate, windows, t_end):
    """The popup's figures from one estimate; usable windows outside its pool are ``retired``."""
    pool = {id(w) for w in estimate.pool}
    rows = []
    for w in sorted(windows, key=lambda w: w.t0):
        flag = FLAGS.get(w.flag, w.flag) or ('' if id(w) in pool or not estimate.pool else 'retired')
        rows.append(dict(ago=float(t_end - w.t0), T=float(w.duration), v_n=float(w.mean[1]), v_e=float(w.mean[0]),
                         psi=math.degrees(w.heading) % 360, r=math.degrees(float(w.mean[2])), flag=flag))
    drift = estimate.drift
    return dict(windows=rows, state=estimate.state, pool=len(estimate.pool),
                dispersion=None if drift is None else float(drift.dispersion),
                trend=None if estimate.trend is None else [float(x) for x in estimate.trend],
                trend_z=float(estimate.drift_trend_z))


class Conditions:
    def __init__(self, settings=None, thresholds=None, clock=time.monotonic, ceiling=None, planned=None, tol=None):
        self.s = settings or est.load_settings()
        self.thresholds = thresholds or load_thresholds()
        self.clock, self.ceiling = clock, ceiling
        self.planned, self.tol = planned, tol
        self.samples = deque()              # (t, E, N, heading, rate, ok, vel_acc)
        self.t_coast = -math.inf
        self._cache, self._at = None, -math.inf
        self._detail = None                 # window table and estimator figures behind the last rating

    def feed(self, t, vel, heading, rate, ok, vel_acc):
        """One sample: world velocity (E, N) m/s, compass rad, yaw rate rad/s, ``ok`` = neutral and afloat.

        Samples closer than 1/MAX_HZ (0.125 s, well under the 0.2 s GPS period) to the last kept one are
        dropped unless ``ok`` changed, which bounds the ring (about 4800 samples) and so the cost of a
        re-rate whatever the feed rate.
        """
        last = self.samples[-1] if self.samples else None
        if last is not None and 0 <= t - last[0] < 1 / MAX_HZ - 1e-9 and last[5] == bool(ok):
            return
        self.samples.append((t, vel[0], vel[1], heading, rate, bool(ok), vel_acc))
        while self.samples[0][0] < t - self.s.drift_stale_s:
            self.samples.popleft()

    def cut(self, t, t95=0.0):
        """The last non-zero command ended at ``t``; samples count again after the coast."""
        self.t_coast = est.coast_time(t, t95, self.s)

    def evaluate(self, now=None):
        """The current rating as a dict; cached for REFRESH_S so a per-tick caller stays cheap."""
        now = self.clock() if now is None else now
        if self._cache is None or not 0 <= now - self._at < REFRESH_S:
            self._cache, self._at = self._rate(), now
        return self._cache

    def _rate(self):
        s = self.s
        out = dict(verdict='unknown', reason='', ceiling=None, drift_trust=None, neutral_s=0.0,
                   drift=None, state='unknown', se=None)
        self._detail = None
        if len(self.samples) < 2:
            out['reason'] = 'measuring 0 s'
            return out
        t, e, n, heading, rate, ok, acc = (np.array(c, float) for c in zip(*self.samples))
        mask = est.neutral_mask(t, ok.astype(bool), acc, self.t_coast, s)
        windows = est.build_windows(t, np.column_stack([e, n]), heading, rate, mask, s)
        estimate = est.estimate(windows, s)
        self._detail = _detail(estimate, windows, t[-1])
        out.update(state=estimate.state, neutral_s=sum(w.duration for w in windows if not w.flag))
        if estimate.drift is None:
            out['reason'] = f'measuring {out["neutral_s"]:.0f}/{NEEDED_WINDOWS * s.min_window_s:.0f} s'
            return out
        sigma_e = readiness.drift_sigma(estimate, t[-1])
        ceiling, limiting = self._ceiling(estimate, windows, sigma_e)
        out.update(drift=[float(x) for x in estimate.drift.vector], se=float(estimate.drift.se),
                   drift_trust=readiness.drift_trust(sigma_e, s.drift_practical), ceiling=ceiling)
        if estimate.material(s):
            out.update(verdict='poor', reason=f'{estimate.state} {estimate.size:.3f} m/s')
        else:
            out['verdict'], out['reason'] = verdict(
                1.0 if ceiling is None else ceiling, out['drift_trust'], self.thresholds)
            if limiting:
                out['reason'] = out['reason'].replace('ceiling', f'{limiting} ceiling', 1)
        return out

    def _ceiling(self, estimate, windows, sigma_e):
        """``(ceiling, limiting direction)``: the injected ceiling, else the min over the planned directions."""
        if self.ceiling:
            return self.ceiling(estimate, windows), None
        planned = self.planned() if self.planned else None
        floor = est.wave_floor(windows, self.s) if planned else None
        if not planned or floor is None:
            return None, None
        self.tol = self.tol or readiness.load_tolerances()
        by_direction = readiness.ceilings(planned, floor, sigma_e, self.tol)
        worst = min(by_direction, key=by_direction.get)
        return by_direction[worst], worst

    def detail(self, now=None):
        """Estimator detail for the conditions popup, or None before any window exists.

        ``windows``: per window ``ago`` (s before the newest sample), ``T`` (s), ``v_n``, ``v_e`` (m/s),
        ``psi`` (compass deg), ``r`` (deg/s) and ``flag`` (coast, gap, turn or retired; '' = pooled).
        Also ``dispersion`` (D), ``trend`` (m/s per s, E and N), ``trend_z``, ``state`` and ``pool`` size.
        """
        self.evaluate(now)
        return self._detail

    def facts(self, now=None):
        """Health facts for the snapshot probe; the verdict carries its reason in ``data``."""
        r = self.evaluate(now)
        time_ = self.clock() if now is None else now
        return {'conditions.verdict': Fact(r['verdict'], data={'reason': r['reason']}, time=time_),
                'conditions.ceiling': Fact(r['ceiling'], time=time_),
                'conditions.drift_trust': Fact(r['drift_trust'], time=time_),
                'conditions.neutral_s': Fact(r['neutral_s'], time=time_)}

    def summary(self, now=None):
        """The rating for session.json and run records: plain JSON values."""
        return dict(self.evaluate(now))
