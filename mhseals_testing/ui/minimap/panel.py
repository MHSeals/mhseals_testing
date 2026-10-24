"""The Drive minimap column (ui U6): a boat-centred view of the same ``MapState`` at 2 Hz.

``MiniMap`` is callable as ``minimap(cols, rows)`` and returns the cached lines of the view;
a new frame is drawn only when the 2 Hz scheduler (``cache.Redraw``) says one is due or the
size changed. It draws the same layers as the Map tab (trail, plan, objects, boat) through
``emit`` at the palette tier, so colours and ASCII mode follow the theme.
"""

import math
import time

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui import palette, theme
from mhseals_testing.ui.minimap import view
from mhseals_testing.ui.minimap.cache import Redraw
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.minimap.emit import emit
from mhseals_testing.ui.minimap.layers import Crumb, Frame, draw_plan, draw_trail
from mhseals_testing.ui.minimap.marks import Projection, draw_objects
from mhseals_testing.ui.minimap.structure import draw_boat

TOKENS = {'plan': 'accent', 'local_plan': 'muted', 'goal': 'accent', 'home': 'accent', 'cursor': 'accent',
          'contact': 'fail', 'wall': 'fg', 'ring': 'muted', 'label': 'muted', 'trail': 'fg'}   # layer -> theme token
HALF = 20.0   # m, the default half-width: the boat sits in the middle of +-20 m


def style(layer, tier=None):
    """Rich style for a canvas layer name: ``layer:token@fade``, ``trail.<mode>.<band>`` or a plain layer."""
    base, _, token = layer.partition(':')
    token, _, fade = token.partition('@')
    if not token:
        parts = base.split('.')
        token = TOKENS.get(parts[0], 'muted')
        if parts[0] == 'trail' and len(parts) == 3:
            token = {'gps': 'gps', 'speed': 'spd', 'track': 'xte'}.get(parts[1], 'fg') + '.' + parts[2]
    try:
        out = theme.PALETTE.style(token, tier=tier)
    except KeyError:
        out = theme.MUTED
    return f'{out} dim' if fade and int(fade) < 100 else out


def crumbs(state):
    """The state's trail crumbs (dicts) as the ``Crumb`` tuples the trail painter takes."""
    return [Crumb(c['x'], c['y'], c['fix'], c['xte_m'], c['speed_mps'], c['epoch']) for c in state.crumbs]


def draw(state, cols, rows, half, table=None, ascii_=False):
    """A canvas of the trail, plan, objects and boat centred in the view; blank without a pose."""
    canvas = Canvas(cols, rows)
    if state is None or state.pose is None:
        return canvas
    _, x, y, yaw = state.pose
    frame, proj = Frame(canvas, (x, y), half), Projection(canvas, (x, y), half)
    draw_trail(frame, crumbs(state), epoch=state.epoch)
    draw_plan(frame, state.plan)
    if table is not None:
        draw_objects(proj, state.objects, table, boat=(x, y), ascii_=ascii_)
    draw_boat(proj, (x, y), math.degrees(yaw), ascii_=ascii_)
    return canvas


class MiniMap:
    """Cached minimap lines for a ``MapState`` (or a callable returning one, or None)."""

    def __init__(self, state, clock=time.monotonic, half=HALF):
        self.state, self.clock, self.half = state, clock, half
        self.redraw = Redraw()
        self.table = load_classes()
        self.tier = palette.detect_tier()
        self._lines, self._key = None, None

    def zoom(self, direction):
        """``z`` (direction 1) or ``Z`` (-1): the next preset half-width."""
        self.half = view.zoom_step(self.half, direction)
        self.redraw.mark()

    def reset(self):
        """``0``: back to the +-20 m default."""
        self.half = HALF
        self.redraw.mark()

    @property
    def subtitle(self):
        return f'{theme.pick("±", "+-")}{self.half:g} m'

    def __call__(self, cols, rows):
        key = (cols, rows, self.half)
        if key != self._key:
            self._key = key
            self.redraw.mark()
        self.redraw.mark()   # the state changes under us; the scheduler caps the rate
        now = self.clock()
        if self._lines is None or self.redraw.due(now, on_map_tab=False):
            began = time.perf_counter()
            state = self.state() if callable(self.state) else self.state
            try:
                canvas = draw(state, cols, rows, self.half, self.table, theme.ASCII)
                self._lines = emit(canvas, lambda layer: style(layer, self.tier),
                                   self.tier, ascii=theme.ASCII)
            except Exception:  # one bad frame must not end the session; keep the last lines
                if self._lines is None:
                    self._lines = []
            self.redraw.done(now, (time.perf_counter() - began) * 1000)
        return self._lines
