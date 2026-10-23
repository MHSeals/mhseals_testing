"""The dashboard's panels: one module each, plus what they share.

Every ``<name>_panel(...)`` returns a Rich renderable from the monitor snapshot and the
app's view state; none reads ROS. ``ui.layout`` places them on the page.
"""



PANELS = ('status', 'motion', 'thrusters', 'procedures', 'activity')
TITLES = {'status': 'Status', 'motion': 'Motion', 'thrusters': 'Thrusters',
          'procedures': 'Procedures', 'activity': 'Activity'}


def window(items, cursor, size):
    """The slice of ``items`` that keeps ``cursor`` visible in ``size`` rows."""
    if size <= 0 or len(items) <= size:
        return 0, items
    start = min(max(0, cursor - size // 2), len(items) - size)
    return start, items[start:start + size]
