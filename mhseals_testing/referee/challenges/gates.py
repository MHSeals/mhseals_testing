"""Ordered red/green (or black/black) gates, optionally out and back through one pair."""

import math

from mhseals_testing.referee import geometry
from mhseals_testing.referee.core import achieved, kept

PARAMS = dict(red='red_buoy', green='green_buoy', max_width=4.0, order='any', out_and_back=False)


def judge(items, window, params, final, course):
    segments = geometry.gates(items, params['red'], params['green'], params['max_width'])
    if params['order'] == 'sequence' and window.track:  # nearest the start first
        start = window.track[0][1:3]
        segments.sort(key=lambda s: math.dist(start, ((s[0][0] + s[1][0]) / 2, (s[0][1] + s[1][1]) / 2)))
    passes = [geometry.crossings(window.track, s) for s in segments]
    if params['out_and_back']:
        ok = any(len(run) >= 2 and run[0][1] != run[-1][1] for run in passes)
        splits = {}
        for run in passes:
            if len(run) >= 2:
                splits = dict(out=run[0][0], back=run[-1][0])
                break
        return [achieved('out and back through the same pair', ok, final)], splits, {}
    missed = [str(i + 1) for i, run in enumerate(passes) if not run]
    checks = [achieved('every gate crossed', bool(segments) and not missed, final,
                       'missed gate ' + ', '.join(missed) if missed else '')]
    if params['order'] == 'sequence':
        firsts = [run[0][0] for run in passes if run]
        checks.append(kept('gates in order', firsts == sorted(firsts), final))
    splits = {f'gate{i + 1}': run[0][0] for i, run in enumerate(passes) if run}
    return checks, splits, dict(gates=f'{len(segments) - len(missed)}/{len(segments)}')
