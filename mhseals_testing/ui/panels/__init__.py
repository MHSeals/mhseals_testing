"""The dashboard's panels: one module each, plus what they share.

Every ``<name>_panel(...)`` returns a Rich renderable from the monitor snapshot and the
app's view state; none reads ROS. ``ui.layout`` places them on the page.
"""

PANELS = ('status', 'motion', 'thrusters', 'procedures', 'activity')
TITLES = {'status': 'Status', 'motion': 'Motion', 'thrusters': 'Thrusters',
          'procedures': 'Procedures', 'activity': 'Activity'}
QUIET = {'LIVE', 'OK', 'PASS', 'READY'}      # rows that may hide inside a folded group
FOLD_ORDER = ('System', 'Frames', 'Sensors')  # quietest first, as U13 folds Session, then System


def window(items, cursor, size):
    """The slice of ``items`` that keeps ``cursor`` visible in ``size`` rows."""
    if size <= 0 or len(items) <= size:
        return 0, items
    start = min(max(0, cursor - size // 2), len(items) - size)
    return start, items[start:start + size]


def fold_groups(groups, budget, cursor_group=None):
    """Groups to collapse so the Status rows fit in ``budget`` lines.

    ``groups`` maps a group name to its row states in display order. A group folds only when
    every row is quiet (never warn, fail or busy) and it does not hold the cursor; groups fold in
    ``FOLD_ORDER`` and only until the rows fit.
    """
    def lines(folded):
        return sum(1 if name in folded else 1 + len(states) for name, states in groups.items())

    folded = set()
    for name in FOLD_ORDER:
        if lines(folded) <= budget:
            break
        states = groups.get(name)
        if states and name != cursor_group and all(str(s).upper() in QUIET for s in states):
            folded.add(name)
    return folded
