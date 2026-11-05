"""Page layout: header, footer, popups and the arrangement of the panels.

``render(snapshot, view, width, height)`` draws the monitor snapshot and the app's view
state. Layout, after cx:

  header   name, profile, host; health strip of status icons
  body     Status (sensors, frames, system) | Motion, Thrusters | Procedures, Activity
  footer   armed state, focus, notice; key hints for the focused panel

The pure decisions (ui.md section 4.5, U13) need no Rich: ``tier`` maps a terminal width to a
density tier, ``fold_groups`` (from ui.panels) picks which quiet Status groups collapse when the
column is short, and ``TabMemory`` keeps the focused panel and every panel's cursor per tab so
switching tabs returns to where the operator was.

The package: ``density`` (sizes, tiers, TabMemory), ``header`` (header and footer), ``overlay``
(popups, toasts, the modal overlay) and ``bodies`` (BODIES, one per tab); every name is re-exported here.
"""

# Every name layout.py imported stays importable as layout.X (unused here: noqa F401 throughout).
import io
import math

from rich.console import Console, Group
from rich.layout import Layout
from rich.segment import Segment
from rich.style import Style
from rich.table import Table
from rich.text import Text

from mhseals_testing.harness import recwatch
from mhseals_testing.ui import chrome, theme, widgets  # noqa: F401  (layout.chrome, layout.theme)
from mhseals_testing.ui.rtk import row as rtk_row
from mhseals_testing.ui.panels import scrubber
from mhseals_testing.ui.panels import TITLES, fold_groups  # noqa: F401  (layout.fold_groups)
from mhseals_testing.ui.panels.activity import activity_panel
from mhseals_testing.ui.panels import correction
from mhseals_testing.ui.panels import bags as bags_panel
from mhseals_testing.ui.panels import checklist as checklist_panel
from mhseals_testing.ui.panels import fusion as fusion_panel
from mhseals_testing.ui.panels import labels as labels_panel
from mhseals_testing.ui.panels.conditions import conditions_panel
from mhseals_testing.ui.panels.minimap import WIDTH as MINIMAP_WIDTH, minimap_panel
from mhseals_testing.ui.panels import mission as mission_panel
from mhseals_testing.ui.panels.motion import latest as motion_latest, motion_panel
from mhseals_testing.ui.panels.procedures import procedures_panel
from mhseals_testing.ui.panels.status import reconcile, status_panel, status_rows
from mhseals_testing.ui.panels.thrusters import thrusters_panel
from mhseals_testing.ui.layout.density import (  # noqa: F401
    CHECKLIST_SIDE, COMPACT_ROWS, COND_ROWS, DEFAULT_FOCUS, FOLD, FOOTER_ROWS, FULL_CONDITIONS, HEADER_ROWS,
    MINIMAP, MIN_SIZE, MOTION_COMPACT, MOTION_MIN_BODY, PINNED_HINTS, REVIEW_ROWS, RIGHT, ROW, SIDEBAR,
    STRIP_FULL, STRIP_SHORT, TABS, THRUSTER_ROWS, TIERS, TOP_ROW, TabMemory, WIDE, WIDER, body_rows,
    motion_shown, status_width, tier)
from mhseals_testing.ui.layout.header import (  # noqa: F401
    KEY_TOKENS, fit_hints, fit_strip, footer, header, health_strip, health_total, issue_chip, key_style,
    playing, rec_chip, strip_item)
from mhseals_testing.ui.layout.overlay import (  # noqa: F401
    FADE, Overlay, TOAST_LINES, TOAST_MAX, TOAST_MIN, TOAST_STYLE, Window, _height, _with_modal, popup,
    popup_rows, toast_box, toast_stack)
from mhseals_testing.ui.layout.bodies import (  # noqa: F401
    BODIES, DECISIONS, LAYERS_ROWS, LAYERS_WIDTH, _cells, _drive_dims, bags_body, correction_body, drive_body,
    drive_slots, fusion_body, ground_speed, labels_body, map_body, placeholder_body, top_rows)


def too_small(view):
    """What replaces the page in a terminal too small for it; Space still stops.

    The first line keeps the one fact the operator needs at any size: armed or not, and the trial.
    """
    state = 'ARMED' if view.get('armed') else 'disarmed'
    if view.get('running'):
        state += f"{theme.SEPARATOR}{view['running']}"
    sep = theme.SEPARATOR
    return Text(f'boat_test{sep}{state}\nenlarge terminal{sep}Space stop{sep}q quit')


def render(snapshot, view, width=120, height=34):
    if width < MIN_SIZE[0] or height < MIN_SIZE[1]:
        return too_small(view)
    extra = REVIEW_ROWS if playing(view) else 0   # strip footer rows below the focus line
    body_height = body_rows(height, bool(extra))
    layout = Layout()
    layout.split_column(Layout(header(snapshot, view, width), size=2), Layout(name='body'),
                        Layout(footer(view, width), size=3 + extra))
    body = layout['body']
    build = BODIES.get(view.get('tab', TABS[0]), placeholder_body)
    checklist = view.get('checklist')
    if checklist is None or not (width >= CHECKLIST_SIDE or checklist.active):
        body.update(build(snapshot, view, width, body_height))
    elif width >= CHECKLIST_SIDE:      # boat_tui preview: the checklist beside every tab
        body.split_row(Layout(build(snapshot, view, width - checklist_panel.WIDTH, body_height)),
                       Layout(checklist.panel(body_height), size=checklist_panel.WIDTH))
    else:                              # narrow: over the body while checking
        body.update(checklist.panel(body_height))
    return _with_modal(layout, view, width, height)

