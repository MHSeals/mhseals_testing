"""``perc.<source>.*`` health facts from /perception/status stages and the ZED /diagnostics.

``Probe.status`` takes one DiagnosticStatus of /perception/status (``name`` is
``perception/<source>/<stage>``, stage ``backend``, ``tracker`` or ``heading``;
perception-v2.md section 7.2), ``Probe.diagnostic`` one of /diagnostics. Missing keys
leave their fact out. A source that stays silent longer than ``silence_s`` reads 0 Hz
instead of vanishing, so a dead or unreachable camera host turns the rate row red. The
timings are the ``probes: perception`` settings of config/health/perception.yaml. Time
comes from an injected monotonic clock so tests need no sleeping.
"""

import statistics
from collections import deque

from mhseals_testing.health.model import Fact
from mhseals_testing.perception import zed_diag

HISTORY = 30                # samples kept for sparks and the clock offset
DROPS = ('drop_stale', 'drop_reorder', 'drop_malformed')
HEADING = {0: 'ok', 1: 'warn', 2: 'error', 3: 'stale'}
ZED = 'zed_native'          # the source the ZED wrapper diagnostics belong to


def number(values, key):
    """The float behind a numeric string key, or None when absent or not a number."""
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if value == value and abs(value) != float('inf') else None


class Source:
    """What one source's stages last said, and the short histories the facts need."""

    def __init__(self):
        self.stages = {}                       # stage -> (receive time, level, message, values)
        self.latency = deque(maxlen=HISTORY)   # backend p50 in s, one per sample
        self.funnel = deque()                  # (time, hz, total drops) inside silence_s
        self.offsets = deque(maxlen=HISTORY)   # clock offsets in s
        self.classes = {}                      # class -> deque of mean confidence


class Probe:
    """Maps the stage keys of every source to facts; ``source`` is the one the profile expects."""

    def __init__(self, source, clock, settings):
        self.source, self.clock = source, clock
        self.rate_window = float(settings['silence_s'])     # silence before 0 Hz; facts drop
        self.grace = float(settings['status_grace_s'])      # quiet tracker before the grid ages
        self.started = clock()
        self.sources = {source: Source()}
        self.zed = None                        # (receive time, zed_diag.parse result)

    def status(self, status, offset=None):
        """Record one /perception/status entry; ``offset`` is its clock offset in seconds."""
        parts = str(status.name).split('/')
        if len(parts) != 3 or parts[0] != 'perception':
            return
        _, name, stage = parts
        now = self.clock()
        src = self.sources.setdefault(name, Source())
        values = {str(item.key): str(item.value) for item in status.values}
        src.stages[stage] = (now, int.from_bytes(status.level, 'little')
                             if isinstance(status.level, bytes) else int(status.level),
                             str(status.message), values)
        if offset is not None:
            src.offsets.append(offset)
        if stage == 'backend':
            latency = number(values, 'latency_ms')
            if latency is not None:
                src.latency.append(latency / 1000)
            hz = number(values, 'hz')
            drops = [number(values, k) for k in DROPS if k in values]
            if hz is not None and drops and None not in drops:
                if src.funnel and sum(drops) < src.funnel[-1][2]:
                    src.funnel.clear()         # the backend restarted; counters began again
                src.funnel.append((now, hz, sum(drops)))
        elif stage == 'tracker':
            for key in values:
                if key.startswith('conf_') and number(values, key) is not None:
                    src.classes.setdefault(key[5:], deque(maxlen=HISTORY)).append(number(values, key))

    def diagnostic(self, status):
        """Record one /diagnostics entry; only the ZED wrapper's count."""
        if zed_diag.is_zed(status):
            self.zed = (self.clock(), zed_diag.parse(status))

    def _live(self, src, stage, now):
        entry = src.stages.get(stage)
        return entry if entry and now - entry[0] <= self.rate_window else None

    def facts(self):
        """Every source's ``perc.<source>.*`` facts."""
        now = self.clock()
        out = {}
        for name, src in self.sources.items():
            out.update(self._source(name, src, now))
        return out

    def _source(self, name, src, now):
        s, out = f'perc.{name}.', {}

        def put(key, value, label='', **data):
            out[s + key] = Fact(value, label or name, data, now)

        silent = now - self.started > self.rate_window and (src.stages or name == self.source)
        backend = self._live(src, 'backend', now)
        stages = {}
        if backend:
            _, _, _, v = backend
            hz = number(v, 'hz')
            if hz is not None:
                put('hz', hz)
                stages['backend'] = {'hz': hz}
            latency = number(v, 'latency_ms')
            if latency is not None:
                p95 = number(v, 'latency_ms_p95')
                put('latency', latency / 1000, history=list(src.latency),
                    **({} if p95 is None else {'p95': p95 / 1000}),
                    **({} if number(v, 'proc_ms') is None else {'proc_ms': number(v, 'proc_ms')}))
            loss = self._funnel(src, now)
            if loss is not None:
                put('funnel_loss', loss, drops={k[5:]: number(v, k) for k in DROPS if k in v})
        elif silent:
            put('hz', 0.0)
        tracker = self._live(src, 'tracker', now)
        if tracker:
            self._tracker(put, tracker, stages, src)
        elif silent and 'tracker' in src.stages:
            put('tracker_hz', 0.0)
        if 'tracker' in src.stages:            # no news from the tracker: its grid only ages
            seen, _, _, v = src.stages['tracker']
            age = number(v, 'grid_age_s')
            if age is not None:
                put('costmap_age', age + max(0.0, now - seen - self.grace))
        if backend and 'ids' in backend[3]:
            share = number(tracker[3], 'id_assoc_share') if tracker else None
            put('ids', backend[3]['ids'], **({} if share is None else {'id_assoc_share': share}))
        heading = self._live(src, 'heading', now)
        if heading:
            _, level, message, v = heading
            put('heading', HEADING.get(level, 'stale'), reason=message,
                **{k: v[k] for k in ('lag_s', 'rho', 'agree', 'n', 'tracks', 'yaw_rate_std', 'excited')
                   if k in v})
        if src.offsets and backend:
            put('clock_offset', statistics.median(src.offsets))
        if name == ZED and self.zed and now - self.zed[0] <= self.rate_window:
            parsed = self.zed[1]
            if parsed['inference_ms'] is not None:
                put('inference_ms', parsed['inference_ms'])
            stages = {**parsed['stages'], **stages}
        if stages:
            put('stages', stages)
        return out

    def _tracker(self, put, tracker, stages, src):
        _, _, _, v = tracker
        hz = number(v, 'hz')
        if hz is not None:
            put('tracker_hz', hz)
            stages['tracker'] = {'hz': hz}
        if number(v, 'latency_ms') is not None:
            put('tracker_latency', number(v, 'latency_ms') / 1000)
        split = {k: number(v, f'tracks_{k}') for k in ('static', 'moving', 'unknown', 'prior')}
        if None not in split.values():
            put('split', ' '.join(f'{k[0].upper()}{n:.0f}' for k, n in split.items()), **split)
        classes = {c: {'conf': number(v, f'conf_{c}'), 'rate': number(v, f'n_{c}') or 0.0,
                       'history': list(src.classes.get(c, ()))}
                   for c in sorted(k[5:] for k in v if k.startswith('conf_'))
                   if number(v, f'conf_{c}') is not None}
        seen = {c: d for c, d in classes.items() if d['rate'] > 0}
        if classes:
            put('classes', len(seen), **classes)
        if seen:                               # the weakest class seen, for the warn row
            low = min(seen, key=lambda c: seen[c]['conf'])
            put('class_conf', seen[low]['conf'], low)

    def _funnel(self, src, now):
        """Share of frames the backend dropped over ``rate_window``: Δdrops / (Δframes + Δdrops)."""
        while src.funnel and now - src.funnel[0][0] > self.rate_window:
            src.funnel.popleft()
        if len(src.funnel) < 2:
            return None
        samples = list(src.funnel)
        frames = sum(b[1] * (b[0] - a[0]) for a, b in zip(samples, samples[1:]))
        drops = samples[-1][2] - samples[0][2]
        return drops / (frames + drops) if frames + drops > 0 else None
