"""Circle test: round a mark, then back to where the window began."""

import math

from mhseals_testing.referee import geometry
from mhseals_testing.referee.core import achieved

PARAMS = dict(mark='yellow_buoy', circle_deg=345.0, return_m=0.75)


def judge(items, window, params, final, course):
    swept = max((geometry.circled(window.track, m[:2])
                 for m in geometry.points(items, params['mark'])), default=0.0)
    home = bool(window.track) and math.dist(window.track[0][1:3], window.track[-1][1:3]) <= params['return_m']
    lapped = swept >= params['circle_deg']
    checks = [achieved('circled a yellow buoy', lapped, final, f'{swept:.0f} deg'),
              achieved('returned to start', home and (lapped or final), final)]
    return checks, {}, dict(around_mark_deg=round(swept, 1))
