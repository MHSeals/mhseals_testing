"""The tab bodies: BODIES maps a tab name to the function that lays out its panels."""

import math

from rich.layout import Layout
from rich.text import Text

from mhseals_testing.ui import held, theme
from mhseals_testing.ui.panels import (bags as bags_panel, correction, fusion as fusion_panel,
                                       labels as labels_panel, mission as mission_panel)
from mhseals_testing.ui.panels.activity import activity_panel
from mhseals_testing.ui.panels.conditions import conditions_panel
from mhseals_testing.ui.panels.minimap import WIDTH as MINIMAP_WIDTH, minimap_panel
from mhseals_testing.ui.panels.motion import latest as motion_latest, motion_panel
from mhseals_testing.ui.panels.procedures import procedures_panel
from mhseals_testing.ui.panels.status import status_panel
from mhseals_testing.ui.panels.thrusters import thrusters_panel
from mhseals_testing.ui.layout.density import COMPACT_ROWS, COND_ROWS, FOLD, FULL_CONDITIONS, MINIMAP, MOTION_COMPACT, RIGHT, ROW, SIDEBAR, THRUSTER_ROWS, TOP_ROW, motion_shown, status_width


def placeholder_body(snapshot, view, width, height):
    """The body of a tab nobody has built yet."""
    body = Text(f"{view['tab']} is not built yet", style=theme.MUTED)
    return theme.block(body, view['tab'], True)


def _cells(slots, sizes=None):
    """Layouts for the slots that hold a panel, in order; an empty slot (None) takes no space."""
    sizes = sizes or {}
    return [Layout(panel, size=sizes.get(name)) for name, panel in slots if panel is not None]


def _drive_dims(width, height):
    """``(wide, lower, half)``: the row tier, the rows under the top rows, and half of them (min 4)."""
    lower = height - top_rows(width) - (MOTION_COMPACT if width < ROW and motion_shown(width, height) else 0)
    return width >= ROW, lower, max(4, lower // 2)


def top_rows(width):
    """Rows of the Conditions + Thrusters stack at ``width``."""
    return (COND_ROWS if width >= FULL_CONDITIONS else COMPACT_ROWS) + THRUSTER_ROWS


def ground_speed(snapshot):
    """Speed over ground (m/s) from the latest fresh odometry surge and sway, or None while odometry is quiet."""
    plots = snapshot.get('plots') or {}
    surge, sway = (motion_latest(plots.get('meas', {}).get(key, []), plots.get('now', 0.0)) for key in ('vx', 'vy'))
    return None if surge is None or sway is None else math.hypot(surge, sway)


def drive_slots(snapshot, view, width, height):
    """The Drive panels by named slot for the tier of ``width`` (ui.md 4.5); None leaves a slot empty.

    ``height`` is the body's. Below ROW columns Motion is the compact stacked panel, absent in a short body.
    """
    wide, lower, half = _drive_dims(width, height)
    below = lower if wide else lower - half   # the Activity rows
    # over Activity while the mission status is fresh, and only when Activity keeps three rows
    mission = mission_panel.strip(view) if below - mission_panel.STRIP_ROWS >= 3 else None
    strip = mission_panel.STRIP_ROWS if mission is not None else 0
    motion = None
    if wide:
        motion = motion_panel(snapshot, view, TOP_ROW)
    elif motion_shown(width, height):
        motion = motion_panel(snapshot, view, compact=True)
    # slow panels lay out at their perf.yaml hold rate between keys (ui/held.py); a key lays them out at once
    return {
        'status': held.hold('status', lambda: status_panel(snapshot, view, height)) if width >= FOLD else None,
        'motion': motion,
        'conditions': held.hold('conditions', lambda: conditions_panel(
            {**view, 'sog': ground_speed(snapshot)}, width < FULL_CONDITIONS)),
        'thrusters': held.hold('thrusters', lambda: thrusters_panel(view, width < 120) if wide
                               else thrusters_panel(view, True, pins=False)),
        'procedures': held.hold('procedures', lambda: procedures_panel(view, lower if wide else half,
                                                                       snapshot['results'])),
        'mission': mission,
        'activity': held.hold('activity', lambda: activity_panel(snapshot, view, below - strip)),
        'minimap': minimap_panel(view, TOP_ROW) if width >= MINIMAP else None,
    }


def drive_body(snapshot, view, width, height):
    """Status column beside the main area: Motion and the right column over Procedures and Activity."""
    slots = drive_slots(snapshot, view, width, height)
    wide, _, half = _drive_dims(width, height)
    main = Layout()
    if wide:
        top = Layout(size=TOP_ROW)
        right = Layout()   # Conditions over Thrusters; an empty slot takes no rows
        right.split_column(*_cells([('conditions', slots['conditions']), ('thrusters', slots['thrusters'])],
                                   {'conditions': COND_ROWS}))
        top.split_row(*_cells([('motion', slots['motion']), ('right', right), ('minimap', slots['minimap'])],
                              {'minimap': MINIMAP_WIDTH, 'right': RIGHT if width < 110 else None}))
        activity = Layout()   # the mission strip, when shown, over Activity
        activity.split_column(*_cells([('mission', slots['mission']), ('activity', slots['activity'])],
                                      {'mission': mission_panel.STRIP_ROWS}))
        bottom = Layout()
        bottom.split_row(*_cells([('procedures', slots['procedures']), ('activity', activity)],
                                 {'procedures': SIDEBAR}))
        main.split_column(top, bottom)
    else:
        main.split_column(*_cells([('motion', slots['motion']), ('conditions', slots['conditions']),
                                   ('thrusters', slots['thrusters']), ('procedures', slots['procedures']),
                                   ('mission', slots['mission']), ('activity', slots['activity'])],
                                  {'motion': MOTION_COMPACT, 'conditions': top_rows(width) - THRUSTER_ROWS,
                                   'thrusters': THRUSTER_ROWS, 'procedures': half,
                                   'mission': mission_panel.STRIP_ROWS}))
    page = Layout()
    page.split_row(*_cells([('status', slots['status'])], {'status': status_width(width) or 30}), main)
    return page


# Tab -> body function (snapshot, view, width, height) -> renderable. A tab not listed shows the
# placeholder; building a tab is registering its body here.
def correction_body(snapshot, view, width, height):
    """The Correction tab: the table (full width below 110 columns) and its side column."""
    model = correction.tab_view(view.get('hardware') or {}, view.get('correction'), view.get('running'))
    return correction.render(model, width, height)


LAYERS_ROWS = 7   # the Layers box: five interior rows and its border
LAYERS_WIDTH = 32
DECISIONS = 100   # from this width the Decisions log joins Layers and Mission


def map_body(snapshot, view, width, height):
    """The Map tab: the minimap over the full body, with the Layers box under it when there is room,
    and beside Layers the Mission box and (from 100 columns) the Decisions log when a mission feed exists."""
    box = view.get('layerbox')
    rows = LAYERS_ROWS if box is not None and height > LAYERS_ROWS + 8 else 0
    page = Layout()
    cells = [Layout(minimap_panel(view, height - rows, width, full=True))]
    if rows and 'mission' in view:
        under = Layout(size=rows)
        under.split_row(Layout(box.render(view['focus'] == 'map'), size=LAYERS_WIDTH),
                        Layout(mission_panel.pane(view)),
                        *([Layout(mission_panel.decisions(view, rows))] if width >= DECISIONS else []))
        cells.append(under)
    elif rows:
        cells.append(Layout(box.render(view['focus'] == 'map'), size=rows))
    page.split_column(*cells)
    return page


def bags_body(snapshot, view, width, height):
    """The Bags tab: the session list over the selected session's summary."""
    bags = view.get('bags') or []
    cursor = min(view['cursors'].get('list', 0), max(0, len(bags) - 1))
    page = Layout()
    page.split_column(
        Layout(bags_panel.render(bags, view.get('bag_usage') or (0, 0, 0), cursor, root=view.get('bag_root', ''),
                                 focused=view['focus'] == 'list')),
        Layout(bags_panel.summary(bags[cursor] if bags else None), size=5))
    return page


def labels_body(snapshot, view, width, height):
    """The Labels tab: conditions, the session timeline and the review list (panels/labels.py)."""
    return labels_panel.render(view['labels'], width, height, view['focus'] == 'labels')


def fusion_body(snapshot, view, width, height):
    """The Fusion tab: camera-lidar alignment (panels/fusion.py)."""
    return fusion_panel.body(view, width, height)


BODIES = {'Drive': drive_body, 'Correction': correction_body, 'Map': map_body, 'Bags': bags_body,
          'Labels': labels_body, 'Fusion': fusion_body}
