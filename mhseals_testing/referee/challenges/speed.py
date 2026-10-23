"""Speed challenge: out and back through a gate, round a mark, timed from the first to the last crossing."""

from mhseals_testing.referee import geometry
from mhseals_testing.referee.core import achieved

PARAMS = dict(gate=dict(red='red_buoy', green='green_buoy', max_width=4.0), mark='blue_buoy',
              circle_deg=300.0)


def judge(items, window, params, final, course):
    gate = params['gate']
    [segment] = geometry.gates(items, gate['red'], gate['green'], gate['max_width'])[:1] or [None]
    passes = geometry.crossings(window.track, segment) if segment else []
    marks = geometry.points(items, params['mark'])
    swept = round(geometry.circled(window.track, marks[0][:2]), 1) if marks else 0.0
    out_back = len(passes) >= 2 and passes[0][1] != passes[-1][1]
    checks = [achieved('out and back through the gate', out_back, final),
              achieved('circled the mark', swept >= params['circle_deg'], final, f'{swept:.0f} deg')]
    splits, details = {}, dict(around_mark_deg=swept)
    if passes:
        splits['gate_out'] = passes[0][0]
    if len(passes) >= 2:
        splits['gate_back'] = passes[-1][0]
        details['gate_to_gate_s'] = round(passes[-1][0] - passes[0][0], 2)
    return checks, splits, details
