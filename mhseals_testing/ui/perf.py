"""The TUI's own speed: frame and key-latency percentiles, bytes written, and where its CPU goes.

config/perf.yaml holds the knobs (``tune``) and the budgets (``budget``). A ``Meter`` takes one sample per
frame (``frame``) and per key (``key``), and the CPU split: the UI thread's CPU spent on frames, the ROS
executor thread's (``ros_cpu``, read from that thread), and the rest of the process. ``report()`` is the
summary perf.json and ``boat_agent status`` carry; ``over()`` names any p99 over its budget.
"""

import os
import threading
import time
from collections import deque

from mhseals_testing.tunables import tune

_lock = threading.Lock()
EWMA = 0.1                 # weight of the newest frame in Meter.typical


def settings():
    """config/perf.yaml ``tune``."""
    return dict(tune('perf'))


def budget():
    """config/perf.yaml ``budget``."""
    from mhseals_testing import strict_yaml
    from mhseals_testing.health.config import config_path
    return dict(strict_yaml.load(config_path('config', 'perf.yaml'))['budget'])


def pct(values, q):
    """The q-quantile (0..1) of ``values`` (nearest rank), None when empty."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


class Meter:
    """Frame, key and CPU samples over the budget window."""

    def __init__(self, limits=None, clock=time.monotonic, cpu=time.process_time, knobs=None):
        self.limits = limits or budget()
        self.settings = knobs or settings()
        self.last = None           # the newest report()
        self.clock, self.cpu = clock, cpu
        self.frames = deque()      # (t, seconds, bytes, ui_cpu)
        self.keys = deque()        # (t, seconds)
        self.skipped = 0           # frames laid out and found unchanged (nothing written)
        self.start, self.cpu0 = clock(), cpu()
        self.ros = None            # (thread_time at start, latest) of the ROS executor thread
        self.pending_key = None    # when the newest unanswered key was read
        self.typical = None        # s; a moving average of frame time (period())

    def frame(self, seconds, sent, ui_cpu=0.0):
        now = self.clock()
        self.frames.append((now, seconds, sent, ui_cpu))   # sent None: a Live that does not say (counted sent)
        self.typical = seconds if self.typical is None else self.typical + EWMA * (seconds - self.typical)
        if sent == 0:
            self.skipped += 1
        if self.pending_key is not None and sent != 0:
            self.keys.append((now, now - self.pending_key))
            self.pending_key = None
        self._trim(now)

    def period(self):
        """Seconds between continuous frames (motion, manual, a popup): 1/``fps``, stretched so laying out frames
        takes at most ``render_share`` of the UI thread, leaving the rest to keys and the tick."""
        base = 1.0 / self.settings['fps']
        if self.typical is None:
            return base
        return min(1.0 / self.settings['fps_min'], max(base, self.typical / self.settings['render_share']))

    def key(self, at=None):
        """A key was read (its frame closes the sample)."""
        if self.pending_key is None:
            self.pending_key = self.clock() if at is None else at

    def ros_cpu(self, seconds):
        """The ROS executor thread's own CPU time so far (time.thread_time on that thread)."""
        with _lock:
            self.ros = (seconds, seconds) if self.ros is None else (self.ros[0], seconds)

    def _trim(self, now):
        horizon = now - self.limits['window']
        for q in (self.frames, self.keys):
            while q and q[0][0] < horizon:
                q.popleft()

    def report(self):
        now = self.clock()
        self._trim(now)
        span = max(1e-6, now - self.frames[0][0]) if len(self.frames) > 1 else None
        times = [f[1] for f in self.frames]
        keys = [k[1] for k in self.keys]
        wall = max(1e-6, now - self.start)
        total = (self.cpu() - self.cpu0) / wall
        with _lock:
            ros = (self.ros[1] - self.ros[0]) / wall if self.ros else None
        frames_cpu = sum(f[3] for f in self.frames) / span if span else None
        out = {
            'frames': len(self.frames), 'fps': round(len(self.frames) / span, 1) if span else None,
            'frame_p50': pct(times, .5), 'frame_p99': pct(times, .99), 'frame_max': max(times, default=None),
            'key_p50': pct(keys, .5), 'key_p99': pct(keys, .99), 'keys': len(keys),
            'bytes_per_s': round(sum(f[2] or 0 for f in self.frames) / span) if span and all(
                f[2] is not None for f in self.frames) else None,
            'unchanged': sum(1 for f in self.frames if f[2] == 0),
            'cpu': round(100 * total, 1),                                   # % of one core, whole process
            'cpu_frames': round(100 * frames_cpu, 1) if frames_cpu is not None else None,
            'cpu_ros': round(100 * ros, 1) if ros is not None else None,
            'cores': os.cpu_count(), 'host_cpu': self.host_cpu(), 'load': _load(),
            'window': self.limits['window'],
        }
        for k in ('frame_p50', 'frame_p99', 'frame_max', 'key_p50', 'key_p99'):
            if out[k] is not None:
                out[k] = round(out[k], 4)
        out['over'] = self.over(out)
        return out

    def host_cpu(self):
        """% busy of all cores together since the last report (from /proc/stat), None where it can't be read."""
        now = _proc_stat()
        last, self._stat = getattr(self, '_stat', None), now
        if now is None or last is None or now[1] <= last[1]:
            return None
        return round(100 * (now[0] - last[0]) / (now[1] - last[1]), 1)

    def over(self, report=None):
        """The budgets the window's p99 passes: ``['frame 0.061 > 0.040', ...]``."""
        report = report or self.report()
        bad = []
        for name, key in (('frame', 'frame_p99'), ('key', 'key_p99')):
            value = report.get(key)
            if value is not None and value > self.limits[name]:
                bad.append(f'{name} {value:.3f} > {self.limits[name]:.3f}')
        return bad


def _proc_stat():
    """(busy, total) jiffies of all cores, or None off Linux."""
    try:
        with open('/proc/stat') as f:
            fields = [int(v) for v in f.readline().split()[1:]]
    except (OSError, ValueError):
        return None
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return sum(fields) - idle, sum(fields)


def _load():
    try:
        return [round(v, 2) for v in os.getloadavg()]
    except OSError:
        return None


SCHEMA = 'mhseals.perf/1'


def start(app):
    """Before the session's screen opens: DiffLive in place of Rich's Live (config ``diff``; a Live the caller
    chose stays), and the Meter as ``app.perf``."""
    from rich.live import Live
    from mhseals_testing.ui import difflive
    if getattr(app, 'live_factory', None) is Live:
        app.live_factory = difflive.live_class()
    if getattr(app, 'perf', None) is None:
        app.perf = Meter()


def period(app, default):
    """The continuous frame period: the Meter's (``Meter.period``), else ``default``."""
    meter = getattr(app, 'perf', None)
    return default if meter is None else meter.period()


def draw(app, force=True):
    """``app.refresh(force)``, timed into the Meter when it drew a frame (``last_frame`` moved); a continuous
    (unforced) frame lets held panels reuse their last layout."""
    meter, before = getattr(app, 'perf', None), getattr(app, 'last_frame', None)
    start, cpu = time.monotonic(), time.thread_time()
    if force:
        app.refresh()                       # the calls the loop made before
    else:
        from mhseals_testing.ui import held
        with held.continuous():             # slow panels may reuse their layout (ui/held.py)
            app.refresh(force=False)
    if meter is not None and getattr(app, 'last_frame', None) != before:
        meter.frame(time.monotonic() - start, getattr(app.live, 'sent', None), time.thread_time() - cpu)


def tick(app, now=None):
    """Once per loop tick: read the ROS thread's CPU and, every ``report_every``, write perf.json in the
    session directory. Advisory: a failed write tries again next period."""
    meter = getattr(app, 'perf', None)
    if meter is None:
        return
    spin = getattr(getattr(app, 'ros', None), 'spin_cpu', None)
    if spin:
        meter.ros_cpu(spin)
    every = meter.settings['report_every']
    now = meter.clock() if now is None else now
    if not every or now - getattr(meter, 'reported', float('-inf')) < every:
        return
    meter.reported = now
    meter.last = meter.report()
    try:
        from mhseals_testing import data
        data.write_json(app.paths.directory / 'perf.json', meter.last, SCHEMA)
    except (OSError, ValueError, TypeError, AttributeError):
        pass
