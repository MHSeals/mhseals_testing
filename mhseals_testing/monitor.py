"""Thread-safe store of sensor rates, TF edges, advisories, events and results.

ROS callbacks write from the executor thread; the UI reads ``snapshot()``.
Every buffer is bounded so a long session cannot grow memory without limit.
The monitor stores data only: scoring lives in ``analysis``.
"""

from collections import deque
from copy import deepcopy
import math
import threading
import time

HISTORY_SAMPLES = 120000  # 40 minutes at 50 Hz, covering a full 30-minute mission

ADVISORY_LEVELS = ('OK', 'WARN', 'FAIL', 'N/A')


class BoatMonitor:
    def __init__(self, stale_after=2.0, clock=time.monotonic, history_sensors=('odometry',)):
        self.stale_after = stale_after
        self.clock = clock
        self.lock = threading.RLock()
        self.sensors = {}
        self.edges = {}
        self.advisories = {}
        # Full-rate samples for trial analysis, per configured sensor.
        self.histories = {name: deque(maxlen=HISTORY_SAMPLES) for name in history_sensors}
        self.events = deque(maxlen=12)
        self.results = deque(maxlen=100)
        self.drift = None  # (x, y) the velocity controller estimates, odom frame, with time

    def observe(self, name, values=None, issue='', stamp_age=None):
        """Record one received message (already validated) for ``name``."""
        now = self.clock()
        values = dict(values or {})
        for key, value in values.items():
            if isinstance(value, float) and not math.isfinite(value):
                values[key] = None  # keeps snapshots JSON-exportable
                issue = issue or 'non-finite measurement'
        with self.lock:
            entry = self.sensors.setdefault(name, {'times': deque(maxlen=300), 'count': 0})
            entry['times'].append(now)
            entry['count'] += 1
            entry.update(values=values, issue=issue, stamp_age=stamp_age)
            if name in self.histories:
                self.histories[name].append((now, deepcopy(values), issue))

    def transform(self, parent, child, stamp_age=0.0, static=False, authority=''):
        """Record a TF edge; ``authority`` identifies the publisher to spot conflicts."""
        with self.lock:
            self.edges[(parent, child, authority)] = (self.clock(), stamp_age, static)
            # Retain static transforms but bound stale dynamic discovery churn.
            if len(self.edges) > 300:
                oldest = min(self.edges, key=lambda key: self.edges[key][0])
                del self.edges[oldest]

    def estimate_drift(self, x, y):
        """Record the velocity controller's live world-frame drift estimate."""
        with self.lock:
            self.drift = (x, y, self.clock())

    def advise(self, name, level, detail=''):
        """Record a non-blocking diagnostic (power, links, compute, ...)."""
        if level not in ADVISORY_LEVELS:
            raise ValueError(f'advisory level must be one of {ADVISORY_LEVELS}')
        with self.lock:
            self.advisories[name] = (self.clock(), level, str(detail))

    def event(self, message):
        with self.lock:
            self.events.append((self.clock(), str(message)))

    def record_result(self, result):
        with self.lock:
            self.results.append(deepcopy(result))

    def history(self, name, since=float('-inf'), until=float('inf'), include_issues=False):
        """Samples of ``name`` as (time, values) within [since, until]."""
        with self.lock:
            samples = list(self.histories.get(name, ()))
        return [(t, values) for t, values, issue in samples
                if since <= t <= until and (include_issues or not issue)]

    def _sensor_rows(self, now):
        sensors = {}
        for name, entry in self.sensors.items():
            times = [t for t in entry['times'] if now - t <= 5]
            age = now - entry['times'][-1]
            stamp_age, issue = entry['stamp_age'], entry['issue']
            if stamp_age is not None:
                stamp_age += age  # the message has aged since it arrived
                if stamp_age < -0.25:
                    issue = issue or 'timestamp is in the future'
                elif stamp_age > self.stale_after:
                    issue = issue or 'stale message timestamp'
            status = 'STALE' if age > self.stale_after else ('WARN' if issue else 'LIVE')
            hz = ((len(times) - 1) / (times[-1] - times[0])
                  if len(times) > 1 and times[-1] > times[0] else 0)
            sensors[name] = dict(age=age, status=status, issue=issue, stamp_age=stamp_age,
                                 count=entry['count'], hz=hz, values=deepcopy(entry['values']))
        return sensors

    def _tf_rows(self, now, required_tf):
        # For each required chain (e.g. map -> odom), build a graph of the
        # fresh edges and search it for a directed path. A frame with more
        # than one parent, or a cycle, is a broken TF tree even when a path
        # exists.
        rows = []
        for source, target in required_tf:
            graph, parents = {}, {}
            for (parent, child, authority), (received, age, static) in self.edges.items():
                if static or (now - received <= self.stale_after and
                              -.25 <= age + now - received <= self.stale_after):
                    graph.setdefault(parent, []).append(child)
                    parents.setdefault(child, set()).add((parent, authority))
            pending, found = [(source, ())], None
            while pending:
                frame, path = pending.pop()
                if frame in path:
                    continue
                if frame == target:
                    found = path + (frame,)
                    break
                pending.extend((child, path + (frame,)) for child in graph.get(frame, []))
            conflicts = [child for child in (found or ()) if len(parents.get(child, ())) > 1]
            visited = set()

            def cyclic(frame, path):
                if frame in path:
                    return True
                if frame in visited:
                    return False
                visited.add(frame)
                return any(cyclic(child, path | {frame}) for child in graph.get(frame, []))

            cycle = cyclic(source, set())
            status = 'MISSING' if found is None else ('WARN' if conflicts or cycle else 'LIVE')
            if cycle:
                detail = 'cycle detected'
            elif conflicts:
                detail = 'multiple parents/publishers: ' + ', '.join(conflicts)
            else:
                detail = ' -> '.join(found) if found else 'no fresh directed chain'
            rows.append(dict(source=source, target=target, status=status, detail=detail))
        return rows

    def snapshot(self, required_tf=(('map', 'odom'), ('odom', 'base_link')), trend_sensor='odometry'):
        now = self.clock()
        with self.lock:
            sensors = self._sensor_rows(now)
            tf = self._tf_rows(now, required_tf)
            advisories = {name: dict(level=level, detail=detail, age=now - received)
                          for name, (received, level, detail) in self.advisories.items()}
            recent = []
            for t, v, issue in reversed(self.histories.get(trend_sensor, ())):
                if now - t > 10:
                    break
                if not issue:
                    recent.append((t, v))
            recent.reverse()
            step = max(1, len(recent) // 32)
            trends = {key: [v[key] for _, v in recent[::step] if v.get(key) is not None][-32:]
                      for key in ('vx', 'vy', 'wz')}
            drift = (self.drift[:2] if self.drift and now - self.drift[2] <= self.stale_after
                     else None)
            return dict(sensors=sensors, tf=tf, advisories=advisories, events=list(self.events),
                        trends=trends, results=deepcopy(list(self.results)), drift=drift)
