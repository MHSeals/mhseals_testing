"""Phase D gate for the Activity timeline (timeline.md section 10).

Not collected by the default run (name does not match test_*.py); run it on the
target with `pytest test/bench_timeline.py -s`. Budgets are the spec's blastoise
numbers: cold build < 15 ms, cached frame < 2 ms. MHS_BENCH_SCALE multiplies
them (a slow CI host); the default 1 is the real gate.
"""

import io
import os
import time

from rich.console import Console

from mhseals_testing.ui.strip import Point as SPoint
from mhseals_testing.ui.strip import Span as SSpan
from mhseals_testing.ui.strip import rows
from mhseals_testing.ui.timeaxis import Axis
from mhseals_testing.ui.timeline import Timeline

COLD_MS, CACHED_MS = 15.0, 2.0
SCALE = float(os.environ.get('MHS_BENCH_SCALE', '1'))
WIDTHS = (48, 118)
LINES = 2000
T0 = 50400.0
RUNS = ('magcal', 'characterize', 'square', 'station')


def journal(n=LINES):
    """A 4 h session of about n journal lines, in time order."""
    out = [{'t_ns': int(T0 * 1e9), 'phase': 'session_start'},
           {'t_ns': int((T0 + 1) * 1e9), 'phase': 'armed'}]
    step = 4 * 3600 / n
    t, i = T0 + 2, 0
    while len(out) < n:
        name = RUNS[i % len(RUNS)]
        out.append({'t_ns': int(t * 1e9), 'phase': name + '_start'})
        out.append({'t_ns': int((t + step) * 1e9), 'phase': name + '_stop'})
        out.append({'t_ns': int((t + 2 * step) * 1e9), 'phase': 'marker', 'id': i, 'note': 'm'})
        out.append({'t_ns': int((t + 2.5 * step) * 1e9), 'phase': 'config_changed',
                    'applied': 'confirmed', 'summary': f'c{i:04d}'})
        t += 3 * step
        i += 1
    return out[:n]


def build(timeline, width):
    """Fold result to width-exact strip rows: the cold path."""
    spans, points = timeline.spans(), timeline.points()
    end = max([s.t1 or s.t0 for s in spans] + [p.t for p in points])
    axis = Axis(T0, end, width, timeline.busy())
    armed = [(s.t0, s.t1 or end) for s in spans if s.kind == 'armed']
    procs = [SSpan(s.t0, s.t1 or end, s.key or s.label[:1], s.kind, running=s.t1 is None)
             for s in spans if s.kind in ('proc', 'manual')]
    pts = [SPoint(p.t, p.kind) for p in points]
    return rows(axis, procs, pts, armed, ascii_mode=False, color=True)


class Cache:
    """The ts098 raster cache in miniature: rebuild only when the key changes."""

    def __init__(self, timeline):
        self.timeline, self.key, self.rows = timeline, None, None

    def get(self, width):
        key = (self.timeline.version, width)
        if key != self.key:
            self.key, self.rows = key, build(self.timeline, width)
        return self.rows


def timed(fn, repeat=20):
    """Best of `repeat` runs, in ms (the best run is the least noisy)."""
    best = float('inf')
    for _ in range(repeat):
        t = time.perf_counter()
        fn()
        best = min(best, (time.perf_counter() - t) * 1e3)
    return best


def cold(width):
    lines = journal()  # fixture built once, outside the timed region

    def run():
        tl = Timeline()
        for line in lines:
            tl.add(line)
        build(tl, width)
    return timed(run, 5)


def cached(width):
    tl = Timeline()
    for line in journal():
        tl.add(line)
    cache = Cache(tl)
    cache.get(width)
    console = Console(file=io.StringIO(), width=width, force_terminal=True)

    def frame():
        for row in cache.get(width):
            console.print(row)
    return timed(frame)


def test_journal_is_the_spec_size():
    lines = journal()
    assert len(lines) == LINES
    assert [x['t_ns'] for x in lines] == sorted(x['t_ns'] for x in lines)


def test_cold_build_budget():
    for width in WIDTHS:
        ms = cold(width)
        print(f'cold {width} cols: {ms:.1f} ms')
        assert ms < COLD_MS * SCALE, (width, ms)


def test_cached_frame_budget():
    for width in WIDTHS:
        ms = cached(width)
        print(f'cached {width} cols: {ms:.2f} ms')
        assert ms < CACHED_MS * SCALE, (width, ms)


def test_cache_rebuilds_only_when_a_line_lands():
    tl = Timeline()
    for line in journal(200):
        tl.add(line)
    cache = Cache(tl)
    first = cache.get(48)
    assert cache.get(48) is first
    tl.add({'t_ns': int((T0 + 5 * 3600) * 1e9), 'phase': 'marker', 'id': 9999})
    assert cache.get(48) is not first
