"""Layout sizes, density tiers and per-tab memory: the pure decisions (ui.md section 4.5, U13), no Rich."""

PINNED_HINTS = ('Space', '?', 'q', 'x / Esc')  # footer keys that are never cut on narrow terminals
WIDE = 106          # footer: four hint columns from here
WIDER = 120         # and five from here, so a 120-column page keeps q Quit on the first row
ROW = 90            # from here Motion sits beside the right column; below it Motion is a compact panel in the stack
RIGHT = 42          # the right column's width below 110 columns: Conditions needs 38 columns inside its border
MOTION_COMPACT = 5  # the stacked Motion: three one-row braille axes
MOTION_MIN_BODY = 24   # the stack drops Motion when the body is shorter than this
SIDEBAR = 36
MINIMAP = 150       # from this width Drive gains the minimap column (ui U6)
FOLD = 60           # below this the Status sidebar folds away so Thrusters keeps its width
COND_ROWS, THRUSTER_ROWS = 13, 7   # the right column: Conditions over Thrusters (ui.md 4.5)
FULL_CONDITIONS = 90               # below this Conditions is the two-line Cond/Cmd form
COMPACT_ROWS = 4
TOP_ROW = COND_ROWS + THRUSTER_ROWS   # Motion height, and the right column's
CHECKLIST_SIDE = 130  # from this width the preview checklist sits beside the body; below it only while checking
STRIP_FULL, STRIP_SHORT = 100, 5   # below 100 columns the strip shows its first five items

HEADER_ROWS = 2
FOOTER_ROWS = 3
REVIEW_ROWS = 3   # events, skyline and axis join the status line
MIN_SIZE = (36, 10)  # below this no panel is legible; only the stop hint is

def body_rows(height, review=False):
    """Rows the body gets in a terminal ``height`` rows tall."""
    return max(5, height - 5 - (REVIEW_ROWS if review else 0))


def motion_shown(width, body):
    """Whether Drive shows Motion: always in the row tier, in the stack only if the body has the rows."""
    return width >= ROW or body >= MOTION_MIN_BODY

# --- density tiers and per-tab memory (no Rich) -----------------------------------

TABS = ('Drive', 'Correction', 'Map', 'Bags', 'Labels', 'Fusion')
DEFAULT_FOCUS = {'Drive': 'status', 'Correction': 'table', 'Map': 'map', 'Bags': 'list', 'Labels': 'labels',
                 'Fusion': 'fusion'}

# (minimum width, tier name, Status column width on Drive); Status is hidden in 'single'.
TIERS = ((150, 'wide', 34), (110, 'standard', 34), (90, 'compact', 28),
         (70, 'narrow', 30), (0, 'single', 0))


def tier(width):
    """Name of the density tier for a terminal ``width``."""
    return next(name for floor, name, _ in TIERS if width >= floor)


def status_width(width):
    """Columns the Status panel takes on Drive at ``width`` (0 when it folds away)."""
    return next(columns for floor, _, columns in TIERS if width >= floor)


class TabMemory:
    """Focus and cursors remembered for each tab."""

    def __init__(self, tabs=TABS):
        self.tabs = tuple(tabs)
        self.current = self.tabs[0]
        self._saved = {}

    def switch(self, step, focus, cursors):
        """Save ``focus`` and ``cursors`` for the current tab, move ``step`` tabs on.

        Returns the new tab's remembered ``(focus, cursors)``; a tab seen for the first time gets
        its default focus and zero cursors.
        """
        self._saved[self.current] = (focus, dict(cursors))
        self.current = self.tabs[(self.tabs.index(self.current) + step) % len(self.tabs)]
        focus, cursors = self._saved.get(self.current, (DEFAULT_FOCUS[self.current], {}))
        return focus, dict(cursors)
