"""Inline anchors: the worst status among the checks that colour one place on screen.

Healthy is plain text. Colours are ANSI names: yellow warn, red fail, dim unknown.
"""

from mhseals_testing.health.model import STATUSES

COLOURS = {'warn': 'yellow', 'fail': 'red', 'unknown': 'dim'}


def anchor_status(specs, results, anchor):
    """Worst status of the checks whose ``show`` names ``anchor``; 'ok' when none have a result."""
    seen = [results[s.id].status for s in specs if anchor in s.show and s.id in results]
    return max(seen, key=STATUSES.index, default='ok')


def anchor_colour(specs, results, anchor):
    """The colour name for ``anchor``, or '' when it should stay plain."""
    return COLOURS.get(anchor_status(specs, results, anchor), '')
