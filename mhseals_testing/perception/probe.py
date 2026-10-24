"""Rolling perception statistics from canonical detection frames (perception.md section 4).

``Probe.observe`` takes one frame (capture stamp, receive time, the detections
in it, possibly none); ``Probe.facts`` returns the ``perc.<source>.*`` health
facts. Time comes from an injected clock so tests need no sleeping.
"""

import math
from collections import deque, namedtuple

from mhseals_testing.health.model import Fact

Detection = namedtuple('Detection', 'label score')   # score 0..1
Frame = namedtuple('Frame', 'recv latency detections')   # latency None for a bad stamp
MIN_SPAN = 1.0      # seconds; rates over less than this are noise, so the span never shrinks below it


def percentile(values, q):
    """The q-quantile (0..1) of ``values`` by linear interpolation; None when empty."""
    ordered = sorted(values)
    if not ordered:
        return None
    pos = q * (len(ordered) - 1)
    low = math.floor(pos)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def histogram(values, bins=10):
    """Counts of 0..1 values in ``bins`` equal bins; 1.0 lands in the last."""
    counts = [0] * bins
    for v in values:
        counts[min(bins - 1, max(0, int(v * bins)))] += 1
    return counts


def _latencies(frames):
    """Latency samples of ``frames``, leaving out those with a bad stamp."""
    return [f.latency for f in frames if f.latency is not None]


class Probe:
    """Rates, gaps, latency, confidence, class presence and stamp errors of one source.

    ``clock`` must use the same timebase as the header stamps (ROS or wall time, not a
    monotonic clock), because latency = clock() - stamp - offset.
    """

    def __init__(self, source, clock, rate_window=10.0, window=60.0, history=120,
                 thresholds=None, bins=10):
        self.source, self.clock = source, clock
        self.rate_window, self.window, self.history, self.bins = rate_window, window, history, bins
        self.thresholds = thresholds or {}       # class label -> minimum score
        self.frames = deque()                    # kept for the longest window
        self.errors = deque()                    # receive times of bad stamps
        self.first = None
        self.last_stamp = None
        self.last_seen = {}                      # label -> receive time

    def observe(self, stamp, detections=(), frame_id=None, offset=0.0):
        """Record a frame; ``offset`` is the sender-minus-receiver clock offset in seconds.

        ``frame_id`` is checked only when given (an empty one is a stamp error). Frames
        with a bad stamp still count for rates but give no latency sample.
        """
        now = self.clock()
        self._reset_if_rewound(now)
        if self.first is None:
            self.first = now
        bad = stamp <= 0 or (frame_id is not None and not frame_id) or (
            self.last_stamp is not None and stamp <= self.last_stamp)
        if bad:
            self.errors.append(now)
        self.last_stamp = stamp
        dets = tuple(detections)
        self.frames.append(Frame(now, None if bad else now - stamp - offset, dets))
        for d in dets:
            self.last_seen[d.label] = now
        self._trim(now)

    def _reset_if_rewound(self, now):
        """Forget everything when the clock went back (a sim episode reset rewinds ROS time)."""
        if self.frames and now < self.frames[-1].recv:
            self.frames.clear()
            self.errors.clear()
            self.last_seen.clear()
            self.first = self.last_stamp = None

    def _trim(self, now):
        keep = max(self.window, self.history)
        while self.frames and now - self.frames[0].recv > keep:
            self.frames.popleft()
        while self.errors and now - self.errors[0] > 60.0:
            self.errors.popleft()

    def _within(self, now, span):
        return [f for f in self.frames if now - f.recv <= span]

    def _span(self, now, span):
        """Seconds the window really covers, so the first seconds do not read low."""
        return max(MIN_SPAN, min(span, now - self.first)) if self.first is not None else span

    def _buckets(self, now, value):
        """``history`` one-second buckets, oldest first; ``value(frames)`` or None if empty."""
        groups = [[] for _ in range(self.history)]
        for f in self.frames:
            age = int(now - f.recv)
            if 0 <= age < self.history:
                groups[self.history - 1 - age].append(f)
        return [value(g) if g else None for g in groups]

    def facts(self):
        """``perc.<source>.*`` facts; empty until the first frame."""
        now = self.clock()
        self._reset_if_rewound(now)
        if not self.frames:
            return {}
        self._trim(now)
        s = f'perc.{self.source}.'
        recent = self._within(now, self.rate_window)
        wide = self._within(now, self.window)
        out = {}

        def put(name, value, **data):
            out[s + name] = Fact(value, self.source, data, now)

        put('hz', len(recent) / self._span(now, self.rate_window))
        stamps = [f.recv for f in recent]
        if len(recent) < len(self.frames):   # silence reaching back past the window still counts
            stamps.insert(0, max(self.frames[-len(recent) - 1].recv, now - self.rate_window))
        edges = stamps + [now]               # the trailing gap is silence since the last frame
        put('max_gap', max((b - a for a, b in zip(edges, edges[1:])), default=0.0))

        lat = _latencies(recent)
        put('latency', percentile(lat, 0.5) if lat else None, p95=percentile(lat, 0.95),
            history=self._buckets(now, lambda g: percentile(_latencies(g), 0.5)))
        put('empty_ratio', sum(not f.detections for f in recent) / len(recent) if recent else None)
        put('stamp_errors', len(self.errors))

        scores = [d.score for f in wide for d in f.detections]
        put('confidence', percentile(scores, 0.5), hist=histogram(scores, self.bins),
            history=self._buckets(now, lambda g: (
                sum(d.score for f in g for d in f.detections) / n
                if (n := sum(len(f.detections) for f in g)) else None)))

        classes = {}
        for label in {d.label for f in wide for d in f.detections}:
            mine = [d.score for f in wide for d in f.detections if d.label == label]
            quick = [d for f in recent for d in f.detections if d.label == label]
            floor = self.thresholds.get(label, 0.0)
            classes[label] = {
                'rate': len(quick) / self._span(now, self.rate_window),
                'presence': sum(any(d.label == label for d in f.detections)
                                for f in recent) / len(recent) if recent else 0.0,
                'hist': histogram(mine, self.bins), 'p50': percentile(mine, 0.5),
                'last_age': now - self.last_seen[label],
                'below_threshold': sum(v < floor for v in mine)}
        put('classes', len(classes), **classes)
        return out
