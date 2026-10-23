"""Tracking feed: the forward model beside the boat, and its per-axis score (thrust addendum).

Each tick the new commanded-effort samples are run through ``analysis.predictor.Predictor`` with the
hardware config's speed curves and t95 lags, plus the world drift from Conditions rotated into the
body frame. The predicted series is the Motion ghost; ``analysis.tracking.tracking_score`` over the
last ``live.window_s`` seconds is the header's ``track NN%`` (None while gated, drawn as a dash).
Only effort commands are modelled; with velocity units the feed contributes nothing.

``add`` takes each gated window score for the session summary; ``close`` appends one summary per
axis to ``provenance.tracking`` in calibration/current.json via ``tracking.record_session``, which
also re-judges staleness for the health info check.
"""

import numpy as np

from mhseals_testing import data
from mhseals_testing.analysis import tracking
from mhseals_testing.analysis.predictor import AXES, Predictor
from mhseals_testing.analysis.tracking import load_settings, score_colour, tracking_score

MEAS_KEYS = ('vx', 'vy', 'wz')          # plots['meas'] key of each axis in AXES order
FLOOR_INDEX = (1, 2, 0)                 # wave_floor is (yaw, surge, sway); AXES is (surge, sway, yaw)
POOR = 'poor'                           # conditions verdict that drops the window (a bad day, not a bad model)


class TrackingFeed:
    """Predicted series and latest scores per axis for Motion (``view()``), plus the session sums for close."""

    def __init__(self, settings=None, path=None):
        self.settings = settings or load_settings()
        self.path = path                                # calibration/current.json unless given
        self.version = None
        self.axes = {}                                  # session sums: axis -> [score sum, windows, trust sum]
        self.window = self.settings['live']['window_s']
        self.key = self.predictor = None
        self.fitted = (False, False, False)             # which axes have a speed curve (else the command ghost stays)
        self.last = None                                # time of the newest command sample consumed
        self.pred, self.cmd = ([], [], []), ([], [], [])   # per axis [(t, v)]
        self.scores = (None, None, None)

    def _model(self, characterization):
        """(Re)build the predictor when the curves or lags changed; drop the old ghost."""
        curves = characterization.get('speed_curves') or {}
        t95 = characterization.get('t95') or {}
        key = repr((sorted(curves.items()), sorted(t95.items())))
        if key != self.key:
            self.key, self.predictor, self.last = key, Predictor(curves, t95), None
            self.fitted = tuple(any(curves.get(a + s) for s in '+-') for a in AXES)
            self.pred, self.cmd = ([], [], []), ([], [], [])
        return self.predictor

    def tick(self, app, snapshot, now):
        plots = snapshot.get('plots')
        if not plots or app.profile.command_units != 'effort':
            self.clear()
            return
        characterization = app.args.hardware_config.get('characterization', {})
        predictor = self._model(characterization)
        if not any(self.fitted):                        # nothing characterized: no model, no score
            self.clear()
            return
        conditions = app.conditions.evaluate()
        drift = conditions.get('drift') or (0.0, 0.0)
        odometry = snapshot['sensors'].get('odometry', {}).get('values', {})
        yaw = odometry.get('yaw') or 0.0
        self.advance(predictor, plots['cmd'], drift, yaw)
        floor = characterization.get('wave_floor') or (0.0, 0.0, 0.0)
        self.scores = self.score(plots['meas'], plots['now'], [floor[i] for i in FLOOR_INDEX],
                                 ignore=conditions.get('verdict') == POOR)

    def clear(self):
        """Drop the ghost, the commands behind it and the scores (and re-seed the next advance)."""
        self.pred, self.cmd = ([], [], []), ([], [], [])
        self.scores = (None, None, None)
        self.last = None

    def advance(self, predictor, commands, drift, yaw):
        """Step the model over the command samples newer than the last one seen."""
        for t, command in commands:
            if self.last is not None and t <= self.last:
                continue
            if self.last is not None:
                for i, v in enumerate(predictor.step(command, t - self.last, drift, yaw)):
                    if not self.fitted[i]:
                        continue
                    self.pred[i].append((t, v))
                    self.cmd[i].append((t, command[i]))
            self.last = t
        if self.last is not None:
            for series in (*self.pred, *self.cmd):
                while series and series[0][0] < self.last - self.window:
                    series.pop(0)

    def score(self, meas, now, sigma, ignore=False):
        """Per-axis score over the last window against the measured series (None when gated)."""
        out = []
        for i, key in enumerate(MEAS_KEYS):
            samples = [s for s in meas.get(key, []) if s[0] >= now - self.window]
            if not samples or not self.pred[i] or not self.fitted[i]:
                out.append(None)
                continue
            times = [t for t, _ in self.pred[i]]
            measured = np.interp(times, [t for t, _ in samples], [v for _, v in samples])
            out.append(tracking_score(measured, [v for _, v in self.pred[i]],
                                      [c for _, c in self.cmd[i]], sigma[i], self.settings, ignore))
        return tuple(out)

    def view(self):
        if not any(self.fitted):
            return {}
        return {'tracking': dict(pred=[list(p) for p in self.pred], score=self.scores,
                                 colour=[score_colour(s, self.settings) for s in self.scores])}

    def add(self, axis, score, trust, version):
        """One window's score (None or NaN is ignored) and the conditions trust it ran under."""
        if score is None or score != score:
            return
        self.version = version
        row = self.axes.setdefault(axis, [0.0, 0, 0.0])
        row[0] += score
        row[1] += 1
        row[2] += trust

    def close(self):
        """Record every axis that scored; returns {axis: staleness verdict}."""
        path = self.path or data.path('calibration', 'current.json')
        verdicts = {axis: tracking.record_session(
            path, axis, dict(mean=total / n, n=n, trust=trust / n, version=self.version), self.settings)
            for axis, (total, n, trust) in self.axes.items()}
        self.axes = {}
        return verdicts
