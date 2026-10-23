"""Live water conditions: a ring of neutral samples rated by the estimator (conditions spec 3).

ROS-free. The ingest path calls ``feed`` per sample on one clock and ``cut`` when a non-zero command
ends; the health snapshot reads ``facts`` and a procedure reads ``evaluate``. Nothing here moves
the boat, and callers use the verdict only to decide whether to *start* a procedure.

``ceiling(estimate, windows)`` is the planned-direction trust ceiling (analysis/readiness). Without
one the ceiling is not limiting and ``conditions.ceiling`` reads None, so the verdict rests on drift.
"""
from __future__ import annotations

from collections import deque
import math
import time

import numpy as np

from mhseals_testing.analysis import conditions as est
from mhseals_testing.analysis.verdict import load_thresholds, verdict
from mhseals_testing.health.model import Fact

REFRESH_S = 1.0         # the estimate is recomputed at most this often
NEEDED_WINDOWS = 3      # the estimator's minimum, for the "measuring" text


class Conditions:
    def __init__(self, settings=None, thresholds=None, clock=time.monotonic, ceiling=None):
        self.s = settings or est.load_settings()
        self.thresholds = thresholds or load_thresholds()
        self.clock, self.ceiling = clock, ceiling
        self.samples = deque()              # (t, E, N, heading, rate, ok, vel_acc)
        self.t_coast = -math.inf
        self._cache, self._at = None, -math.inf

    def feed(self, t, vel, heading, rate, ok, vel_acc):
        """One sample: world velocity (E, N) m/s, compass rad, yaw rate rad/s, ``ok`` = neutral and afloat."""
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
        if len(self.samples) < 2:
            out['reason'] = 'measuring 0 s'
            return out
        t, e, n, heading, rate, ok, acc = (np.array(c, float) for c in zip(*self.samples))
        mask = est.neutral_mask(t, ok.astype(bool), acc, self.t_coast, s)
        windows = est.build_windows(t, np.column_stack([e, n]), heading, rate, mask, s)
        estimate = est.estimate(windows, s)
        out.update(state=estimate.state, neutral_s=sum(w.duration for w in windows if not w.flag))
        if estimate.drift is None:
            out['reason'] = f'measuring {out["neutral_s"]:.0f}/{NEEDED_WINDOWS * s.min_window_s:.0f} s'
            return out
        se = estimate.drift.se
        out.update(drift=[float(x) for x in estimate.drift.vector], se=float(se),
                   drift_trust=1 - math.exp(-s.drift_practical ** 2 / (2 * se ** 2)),
                   ceiling=self.ceiling(estimate, windows) if self.ceiling else None)
        if estimate.material(s):
            out.update(verdict='poor', reason=f'{estimate.state} {estimate.size:.3f} m/s')
        else:
            out['verdict'], out['reason'] = verdict(
                1.0 if out['ceiling'] is None else out['ceiling'], out['drift_trust'], self.thresholds)
        return out

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
