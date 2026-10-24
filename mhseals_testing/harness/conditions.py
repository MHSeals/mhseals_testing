"""Pure water conditions: admitted sample segments rated by the estimator (conditions spec 3).

ROS-free and unwired to live ingestion. Offline callers feed real sample receipts; the explicit
``advance`` seam ages and bounds admission on monotonic polling time, even without messages.
Nothing here moves the boat. Callers use the verdict only to decide whether to start a procedure.

``ceiling(estimate, windows)`` is the planned-direction trust ceiling (analysis/readiness). Without
one the ceiling is not limiting and ``conditions.ceiling`` reads None, so the verdict rests on drift.
``planned()`` (the Directions about to be run) builds that ceiling live: the minimum over them (spec 2.2). Drift trust
uses sigma_e^2 = se^2 + (beta*dt)^2 (spec 2.1), so a trending drift lowers it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

import numpy as np

from mhseals_testing.analysis import conditions as est, readiness
from mhseals_testing.analysis.verdict import load_thresholds, verdict
from mhseals_testing.harness.conditions_evidence import EvidenceState, finite
from mhseals_testing.health.model import Fact

REFRESH_S = 1.0         # the estimate is recomputed at most this often
MAX_HZ = 8.0            # cap above the 5 Hz GPS rate (jittered real samples are never dropped); only a faster feed (sim odometry) is thinned
STILL_SPEED, STILL_RATE, STILL_FRESH_S = 0.15, 0.1, 2.0   # m/s, rad/s, s: 'still' for the resume gate
NEEDED_WINDOWS = 3      # the estimator's minimum, for the "measuring" text


def stillness(e, n, rate):
    """Live measured motion only; missing or nonfinite values never establish stillness."""
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)
               for v in (e, n, rate)):
        return ['odometry is missing or nonfinite']
    reasons = []
    if math.hypot(e, n) > STILL_SPEED:
        reasons.append(f'moving {math.hypot(e, n):.2f} m/s')
    if abs(rate) > STILL_RATE:
        reasons.append(f'turning {abs(rate):.2f} rad/s')
    return reasons


_OMITTED = object()


@dataclass
class _Segment:
    start: float
    coast: float


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
        self.by_direction = None
        self._members = deque()             # segment ID per public sample; None is ineligible
        self._segments = {}
        self._serial = 0
        self._active = None
        self._eligible = _OMITTED           # omitted: legacy offline feed owns admission
        self._epoch = _OMITTED
        self._now = None
        self._coast_event = None
        self._pristine = True

    def _invalidate(self):
        self._cache, self._at = None, -math.inf
        self._detail = self.by_direction = None

    def reset(self):
        """Clear history, admission, clocks and derived state without inventing a sample."""
        self.samples.clear()
        self._members.clear()
        self._segments.clear()
        self._active = None
        self._serial = 0
        self._eligible = self._epoch = _OMITTED
        self._now = self._coast_event = None
        self.t_coast = -math.inf
        self._pristine = True
        self._invalidate()

    def _prune(self, now):
        expired = False
        while self.samples and self.samples[0][0] < now - self.s.drift_stale_s:
            self.samples.popleft()
            self._members.popleft()
            expired = True
        if expired:
            if not self.samples:
                self._close()
            retained = set(self._members) | {self._active}
            self._segments = {key: value for key, value in self._segments.items()
                              if key in retained}
            self._invalidate()

    def _close(self):
        if self._active is not None:
            if self._active not in self._members:
                self._segments.pop(self._active, None)
            self._active = None
            self._invalidate()

    def _open(self, now, coast):
        self._pristine = False
        self._serial += 1
        self._active = self._serial
        self._segments[self._active] = _Segment(now, coast)
        self.t_coast = coast
        self._invalidate()

    def advance(self, now, *, epoch=_OMITTED, eligibility=_OMITTED, t95=0.0):
        """
        Age on monotonic polling time, independently of real sample receipts.

        Unwired caller contract: poll the source even on duplicates, pass its epoch
        even if no sample was returned, and combine source status with affirmative
        EvidenceGate eligibility (e.g. replace(state, eligible=status.valid and
        state.eligible)). EvidenceState.coast_started identifies one coast event;
        repeated cut flags never allocate segments. A bool/None can also close or
        reopen admission. Omitted arguments preserve legacy offline callers.
        Return False for an invalid clock after clearing all historical evidence.
        """
        if epoch is not _OMITTED:
            try:
                hash(epoch)
                valid_epoch = bool(epoch == epoch)
            except (TypeError, ValueError):
                valid_epoch = False
            if not valid_epoch:
                raise ValueError('epoch must be a stable, reflexive hashable identity')
        if not finite(t95) or t95 < 0:
            raise ValueError('t95 must be finite and nonnegative')
        if not finite(now):
            self.reset()
            return False
        if ((epoch is not _OMITTED and self._epoch is not _OMITTED and epoch != self._epoch)
                or (self._now is not None and now < self._now)):
            self.reset()
        if epoch is not _OMITTED:
            self._epoch = epoch
        self._now = now
        self._prune(now)
        if eligibility is not _OMITTED:
            state = eligibility if isinstance(eligibility, EvidenceState) else None
            eligible = state.eligible is True if state else eligibility is True
            event = state.coast_started if state else None
            # Reject malformed event time conservatively; it cannot establish coast.
            if event is not None and (not finite(event) or event > now):
                eligible = False
            new_event = event is not None and event != self._coast_event
            if not eligible:
                self._close()
            elif self._active is None or new_event:
                self._close()
                self._open(now, est.coast_time(now, t95, self.s))
            self._eligible = eligible
            self._coast_event = event
        return True

    def feed(self, t, vel, heading, rate, ok, vel_acc):
        """One sample: world velocity (E, N) m/s, compass rad, yaw rate rad/s, ``ok`` = neutral and afloat.

        The caller must advance on polling time even when no source message arrives.
        Samples closer than 1/MAX_HZ (0.125 s, well under the 0.2 s GPS period) to the last kept one are
        dropped unless ``ok`` or the admission segment changed. This bounds the ring
        (about 4800 samples) and so the cost of a re-rate whatever the feed rate.
        """
        if not finite(t):
            return
        last = self.samples[-1] if self.samples else None
        if ((last is not None and t <= last[0])
                or (self._now is not None and t < self._now - self.s.drift_stale_s)):
            return                          # reordered receipts never reset an epoch
        admitted = bool(ok) and self._eligible is not False
        if not admitted:
            self._pristine = False
            self._close()
        elif self._active is None:
            # Initial legacy offline streams have no preceding command boundary.
            coast = (-math.inf if self._pristine and self.t_coast == -math.inf
                     and self._eligible is _OMITTED else est.coast_time(t, 0.0, self.s))
            self._open(t, max(coast, self.t_coast))
        if (last is not None and 0 <= t - last[0] < 1 / MAX_HZ - 1e-9 and last[5] == bool(ok)
                and self._members[-1] == self._active):
            return
        self.samples.append((t, vel[0], vel[1], heading, rate, bool(ok), vel_acc))
        segment = self._segments.get(self._active)
        self._members.append(self._active if admitted and segment and t >= segment.start else None)
        self._prune(max(t, self._now) if self._now is not None else t)

    def moving(self, now=None):
        """Reasons fresh measured motion does not establish stillness."""
        now = self.clock() if now is None else now
        if not self.samples:
            return ['odometry is missing']
        t, e, n, _, rate, _, _ = self.samples[-1]
        if not math.isfinite(now - t) or not 0 <= now - t <= STILL_FRESH_S:
            return ['odometry is stale']
        return stillness(e, n, rate)

    def cut(self, t, t95=0.0):
        """The last non-zero command ended at ``t``; samples count again after the coast."""
        if not finite(t) or not finite(t95) or t95 < 0:
            raise ValueError('cut and t95 must be finite; t95 must be nonnegative')
        self._close()
        self._open(t, est.coast_time(t, t95, self.s))

    def evaluate(self, now=None):
        """The current rating as a dict; cached for REFRESH_S so a per-tick caller stays cheap."""
        now = self.clock() if now is None else now
        if not self.advance(now):
            return self._unknown('invalid clock')
        if self._cache is None or not 0 <= now - self._at < REFRESH_S:
            self._cache, self._at = self._rate(now), now
        return self._cache

    @staticmethod
    def _unknown(reason=''):
        return {'verdict': 'unknown', 'reason': reason, 'ceiling': None, 'drift_trust': None,
                'neutral_s': 0.0, 'drift': None, 'state': 'unknown', 'se': None, 'floor': None,
                'sea': None, 'ceilings': None}

    def _rate(self, now):
        s = self.s
        out = self._unknown()
        self._detail = None
        if len(self.samples) < 2:
            out['reason'] = 'measuring 0 s'
            return out
        t, e, n, heading, rate, ok, acc = (np.array(c, float) for c in zip(*self.samples))
        mask = np.zeros(len(t), dtype=bool)
        windows = []
        members = np.array(self._members, dtype=object)
        for key, segment in self._segments.items():
            idx = np.nonzero(members == key)[0]
            if not len(idx):
                continue
            local = est.neutral_mask(t[idx], ok[idx].astype(bool), acc[idx], segment.coast, s)
            mask[idx] = local
            windows.extend(est.build_windows(t[idx], np.column_stack([e[idx], n[idx]]),
                                             heading[idx], rate[idx], local, s))
        windows.sort(key=lambda w: w.t0)
        estimate = est.estimate(windows, s)
        self._detail = _detail(estimate, windows, now)
        out.update(state=estimate.state, neutral_s=sum(w.duration for w in windows if not w.flag))
        if estimate.drift is None:
            out['reason'] = f'measuring {out["neutral_s"]:.0f}/{NEEDED_WINDOWS * s.min_window_s:.0f} s'
            return out
        out.update(self._sea(t, e, n, rate, mask, windows))
        sigma_e = readiness.drift_sigma(estimate, now)
        ceiling, limiting = self._ceiling(estimate, windows, sigma_e)
        out['ceilings'] = self.by_direction
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
        self.by_direction = None
        if self.ceiling:
            return self.ceiling(estimate, windows), None
        planned = self.planned() if self.planned else None
        floor = est.wave_floor(windows, self.s) if planned else None
        if not planned or floor is None:
            return None, None
        self.tol = self.tol or readiness.load_tolerances()
        by_direction = readiness.ceilings(planned, floor, sigma_e, self.tol)
        worst = min(by_direction, key=by_direction.get)
        self.by_direction = {d: float(c) for d, c in by_direction.items()}   # per-direction ceilings, for the Correction tab
        return by_direction[worst], worst

    def detail(self, now=None):
        """Estimator detail for the conditions popup, or None before any window exists.

        ``windows``: per window ``ago`` (s before evaluation time), ``T`` (s), ``v_n``, ``v_e`` (m/s),
        ``psi`` (compass deg), ``r`` (deg/s) and ``flag`` (coast, gap, turn or retired; '' = pooled).
        Also ``dispersion`` (D), ``trend`` (m/s per s, E and N), ``trend_z``, ``state`` and ``pool`` size.
        """
        now = self.clock() if now is None else now
        self.evaluate(now)
        if self._detail is None:
            return None
        return dict(self._detail, windows=[dict(row, ago=row['ago'] + now - self._at)
                                           for row in self._detail['windows']])

    def _sea(self, t, e, n, rate, mask, windows):
        """Wave level for the Conditions panel: ``floor`` the yaw trust floor (rad/s), ``sea`` the yaw-rate
        scatter of the neutral samples (rad/s). There is no heave (a_z) source yet, so no up-down figure."""
        floor = est.wave_floor(windows, self.s)
        out = dict(floor=None if floor is None else float(floor[0]))
        scatter = rate[mask & np.isfinite(rate)]    # a bad sample must not turn the figure into NaN
        if len(scatter) > 2:
            out['sea'] = float(np.std(scatter))
        return out

    def facts(self, now=None):
        """Health facts for the snapshot probe; the verdict carries its reason in ``data``."""
        time_ = self.clock() if now is None else now
        r = self.evaluate(time_)
        source = self.samples[-1][0] if self.samples else None
        diagnostic = dict(reason=r['reason'], source_last_time=source,
                          source_age=None if source is None else time_ - source)
        time_ = time_ if finite(time_) else None
        return {'conditions.verdict': Fact(r['verdict'], data=diagnostic, time=time_),
                'conditions.ceiling': Fact(r['ceiling'], time=time_),
                'conditions.drift_trust': Fact(r['drift_trust'], time=time_),
                'conditions.neutral_s': Fact(r['neutral_s'], time=time_)}

    def summary(self, now=None):
        """The rating for session.json and run records: plain JSON values."""
        return dict(self.evaluate(now))
