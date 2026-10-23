"""RTK detail pane for the Odometry popup: the fix timeline and its totals; plain text lines.

``r`` on the pane opens the base controls (``KEYS``); the host maps the action name.
"""

from mhseals_testing.analysis.fixlog import STATES, Fixlog

KEYS = {'r': 'base_controls'}
_BLOCKS = {'none': '▁', '3D': '▃', 'DGPS': '▄', 'FLOAT': '▆', 'FIXED': '█', None: '·'}
_ASCII = {'none': '_', '3D': '-', 'DGPS': '=', 'FLOAT': '+', 'FIXED': '#', None: '.'}


def clock(seconds):
    seconds = int(seconds)
    return f'{seconds // 60}:{seconds % 60:02d}'


def strip(log, width, ascii=False):
    glyphs = _ASCII if ascii else _BLOCKS
    return ''.join(glyphs[s] for s in log.bins(width))


def rtk_pane(events, now, width=40, ascii=False, start=None):
    """Lines: the timeline strip, then one row per state, then time to first FIXED."""
    log = Fixlog(events, now, start)
    if log.current is None:
        return ['no fix events yet', 'r  base controls']
    lines = [strip(log, width, ascii), f'now {log.current}']
    for state in STATES:
        lines.append(f'{state:<6}{clock(log.seconds[state]):>7}  x{log.entries[state]}')
    first = 'never' if log.first_fixed is None else clock(log.first_fixed)
    lines += [f'first FIXED {first}', 'r  base controls']
    return [line[:width] for line in lines]


def key_action(key):
    """The action for a key pressed on the pane, or None."""
    return KEYS.get(key)
