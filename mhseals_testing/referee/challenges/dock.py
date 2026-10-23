"""Docking: held in an empty bay under the banner of the day, never in another bay."""

from mhseals_testing.referee import geometry
from mhseals_testing.referee.core import achieved, kept

PARAMS = dict(banner='', hold_s=5.0, wrong_s=1.0, facing_deg=30.0, margin=0.3)


def judge(items, window, params, final, course):
    banner = course.day.get('docking', {}).get('banner') or params['banner']
    bays = [i for i in items if 'yaw' in i]
    right = [b for b in bays if b['banner'] == banner and not b['occupied']]

    def held(bay):
        return geometry.held_in_bay(window.track, bay, params['hold_s'], params['facing_deg'], params['margin'])

    best = max((held(b) for b in right), default=0.0)
    wrong = max((held(b) for b in bays if b not in right), default=0.0)
    checks = [achieved('docked in an empty bay of the banner', best >= params['hold_s'], final),
              kept('never in a wrong bay', wrong < params['wrong_s'], final)]
    return checks, {}, dict(held_s=round(best, 1))
