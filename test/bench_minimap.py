"""Print the cached map frame cost (map.md 9): ``PYTHONPATH=. python test/bench_minimap.py``.

Run it on the boat computer too; above `cache.BUDGET_MS` the Drive panel falls back to 2 Hz.
"""
import time

from mhseals_testing.ui.minimap import cache
from mhseals_testing.ui.minimap.canvas import Canvas


def _layer(n):
    c = Canvas(56, 30)
    for i in range(n):
        c.dot(i % 112, (i * 7) % 120, 'costmap')
    return c


def main(runs=200):
    lc = cache.LayerCache()
    layers = [('cost', 1, lambda: _layer(1500)), ('ref', 1, lambda: _layer(300))]
    lc.frame(56, 30, layers)
    times = []
    for _ in range(runs):
        t = time.perf_counter()
        f = lc.frame(56, 30, layers)
        f.dot(10, 10, 'boat')
        f.cells()
        times.append((time.perf_counter() - t) * 1000)
    times.sort()
    print(f'cached frame: median {times[runs // 2]:.2f} ms, p95 {times[int(runs * .95)]:.2f} ms '
          f'(budget {cache.BUDGET_MS} ms)')


if __name__ == '__main__':
    main()
