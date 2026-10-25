"""The Drive minimap column (ui U6): a boat-centred view of the same ``MapState`` at 2 Hz.

``MiniMap`` is callable as ``minimap(cols, rows)`` and returns the cached lines of the view;
a new frame is drawn only when the 2 Hz scheduler (``cache.Redraw``) says one is due or the
size changed. It draws the same layers as the Map tab (trail, plan, objects, boat) through
``emit`` at the palette tier, so colours and ASCII mode follow the theme.
"""

import math
import time

import numpy as np

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui import palette, theme
from mhseals_testing.ui.minimap import tasks, view
from mhseals_testing.ui.minimap.cache import LayerCache, Redraw
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.minimap.emit import emit
from mhseals_testing.ui.minimap.frame import draw_accuracy, draw_chrome, draw_rings, ring_step
from mhseals_testing.ui.minimap.layers import Crumb, Frame, draw_costmap, draw_hits, draw_plan, draw_trail
from mhseals_testing.ui.minimap.marks import Projection, draw_fov, draw_objects
from mhseals_testing.ui.minimap.structure import VMAX, draw_boat

TOKENS = {'plan': 'accent', 'local_plan': 'muted', 'goal': 'accent', 'home': 'accent', 'cursor': 'accent',
          'contact': 'fail', 'wall': 'fg', 'ring': 'muted', 'label': 'muted', 'trail': 'fg', 'chrome': 'muted',
          'hits': 'fg', 'hits.near': 'fail', 'costmap': 'muted', 'task': 'task.confirmed'}   # layer -> theme token
HALF = 20.0   # m, the default half-width: the boat sits in the middle of +-20 m
YAW_Q = 3.0   # deg: heading-up rotation of the cached costmap layer


def _token(base):
    """The palette token of a bare layer name: listed, a palette token itself, or a trail band."""
    if base in TOKENS:
        return TOKENS[base]
    if base in theme.PALETTE.tokens:
        return base
    parts = base.split('.')
    if parts[0] == 'trail' and len(parts) == 3:
        return {'gps': 'gps', 'speed': 'spd', 'track': 'xte'}.get(parts[1], 'fg') + '.' + parts[2]
    return TOKENS.get(parts[0], 'muted')


def style(layer, tier=None):
    """Rich style for a canvas layer or tint: ``layer:token@fade``, a bare ``token@fade``,
    ``trail.<mode>.<band>`` or a plain layer. A full-strength object is bold, a faded one dim."""
    base, _, token = layer.partition(':')
    token, _, fade = (token or base).partition('@')
    if ':' not in layer:
        token = _token(token)
    try:
        out = theme.PALETTE.style(token, tier=tier)
    except KeyError:
        out = theme.MUTED
    if fade and int(fade) < 100:
        return f'{out} dim'
    return f'{out} bold' if base == 'object' else out


def crumbs(state):
    """The trail painter's ``Crumb`` tuples: the smoothed trail, else the raw crumbs (dicts)."""
    if getattr(state, 'smooth', None):
        return list(state.smooth)
    return [Crumb(c['x'], c['y'], c['fix'], c['xte_m'], c['speed_mps'], c['epoch'], c.get('t'))
            for c in state.crumbs]


def grid_of(message):
    """(cells int8 HxW, resolution, origin xy) of a nav_msgs/OccupancyGrid (duck-typed)."""
    info = message.info
    cells = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
    return cells, info.resolution, (info.origin.position.x, info.origin.position.y)


def _costmap_layer(cols, rows, half, centre, up, message, box, density):
    def build():
        canvas = Canvas(cols, rows)
        frame = Frame(canvas, centre, half, up)
        cells, resolution, origin = grid_of(message)
        if box.costmap:
            draw_costmap(frame, cells, resolution, origin, density)
        if box.hits:
            draw_hits(frame, cells, resolution, origin, centre)
        return canvas
    return build


def draw(state, cols, rows, half, table=None, ascii_=False, box=None, full=False, tier=None, cache=None,
         slow=False):
    """A canvas of the map layers centred on the boat; blank without a pose.

    ``box`` (a LayerBox) switches the layers and heading-up; without one the trail, plan, objects
    and boat draw as on the Drive column. ``full`` is the Map tab: rings with labels and the
    camera cone. Costmap and rings come from ``cache`` (a LayerCache) when given. ``slow`` (the
    frame is over budget) drops the wake and ring labels first.
    """
    canvas = Canvas(cols, rows)
    if state is None or state.pose is None:
        return canvas
    t, x, y, yaw = state.pose
    up = math.degrees(yaw) if box is not None and box.up == 'H' and math.isfinite(yaw) else None
    frame = Frame(canvas, (x, y), half, up)
    layers = []
    message = state.latest.get('costmap')
    if box is not None and (box.costmap or box.hits) and message is not None:
        cell = 4 * frame.metres_per_dot   # snap the cached layer's centre to a cell
        centre = (round(x / cell) * cell, round(y / cell) * cell)
        up_q = None if up is None else round(up / YAW_Q) * YAW_Q
        density = tier in ('16', 'mono') or ascii_
        layers.append(('costmap', (id(message), cols, rows, half, centre, up_q, box.costmap, box.hits, density),
                       _costmap_layer(cols, rows, half, centre, up_q, message, box, density)))
    scale = canvas.dots_wide / (2 * half) if half > 0 else 0.0
    rings = full and (box is None or box.rings) and scale > 0
    count = 3 if rows >= 30 else 2
    if rings:
        layers.append(('rings', (cols, rows, half, count, not slow),
                       lambda: _rings(cols, rows, scale, count, not slow)))
    if layers:
        cache = cache or LayerCache()
        canvas = cache.frame(cols, rows, layers)
        frame.canvas = canvas
    proj = Projection(canvas, (x, y), half, up)
    trail = 'plain' if box is None else box.trail
    if trail != 'off':
        draw_trail(frame, crumbs(state), mode=trail, epoch=state.epoch, vmax=VMAX,
                   colourless=tier in ('16', 'mono'), now=t, **({'columns': cache.trail} if cache else {}))
    draw_plan(frame, state.plan)
    draw_plan(frame, state.latest.get('local_plan') or [], local=True)
    accuracy = state.accuracy() if hasattr(state, 'accuracy') else None
    draw_accuracy(proj, (x, y), accuracy)
    if full and state.fov and (box is None or box.fov is not False) and (camera := state.camera()):
        draw_fov(proj, camera[:2], camera[2], state.fov['hfov_deg'], state.fov['range_m'],
                 table.fov['colour'] if table is not None else 'fov')
    found, goal = getattr(state, 'tasks', ()), getattr(state, 'mission_goal', None)
    if (box is None or box.tasks) and (found or goal):
        tasks.draw(proj, found, goal, ascii_, labels=full)
    if table is not None and (box is None or box.objects != 'off'):
        draw_objects(proj, state.objects, table, boat=(x, y), ascii_=ascii_)
    draw_boat(proj, (x, y), math.degrees(yaw), ascii_=ascii_, speed=getattr(state, 'speed', None),
              cog=state.course() if hasattr(state, 'course') else None,
              wake=(box is None or box.wake) and not slow)
    if scale > 0:
        draw_chrome(canvas, ring_step(canvas.dots_high, scale), scale, up, ascii_,
                    accuracy[0] if accuracy else None)
    return canvas


def _rings(cols, rows, scale, count, labels):
    canvas = Canvas(cols, rows)
    draw_rings(canvas, scale, count, labels)
    return canvas


class MiniMap:
    """Cached minimap lines for a ``MapState`` (or a callable returning one, or None)."""

    def __init__(self, state, clock=time.monotonic, half=HALF):
        self.state, self.clock, self.half = state, clock, half
        self.redraw = Redraw()
        self.table = load_classes()
        self.tier = palette.detect_tier()
        self.cache = LayerCache()
        self._lines, self._key, self._facts, self._step = None, None, [], None

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
        """``±20 m``; on the Map tab also ``rings 5 m · fix · 0.8 m/s 069°`` (course as a bearing)."""
        return theme.SEPARATOR.join([f'{theme.pick("±", "+-")}{self.half:g} m', *self._facts])

    def _title_facts(self, state, full):
        if not full or state is None or state.pose is None:
            return []
        facts = []
        if self._step:
            facts.append(f'rings {self._step:g} m')
        accuracy = state.accuracy()
        if accuracy:
            facts.append(accuracy[1])
        speed, cog = state.speed, state.course()
        if speed is not None and math.isfinite(speed):
            facts.append(f'{speed:.1f} m/s' + (f' {(90 - cog) % 360:03.0f}°' if cog is not None else ''))
        return facts

    def __call__(self, cols, rows, box=None, full=False):
        """The emitted lines of a ``cols`` x ``rows`` view; ``box`` is the Layers box, ``full`` the Map tab."""
        key = (cols, rows, self.half, full)
        if key != self._key:
            self._key = key
            self.redraw.mark()
        self.redraw.mark()   # the state changes under us; the scheduler caps the rate
        now = self.clock()
        if self._lines is None or self.redraw.due(now, on_map_tab=full):
            began = time.perf_counter()
            state = self.state() if callable(self.state) else self.state
            try:
                if box is not None and box.fov is None:
                    box.fov = bool(self.table.fov['show'])
                canvas = draw(state, cols, rows, self.half, self.table, theme.ASCII, box, full, self.tier,
                              self.cache, self.redraw.slow)
                rings = full and self.half > 0 and (box is None or box.rings)
                self._step = ring_step(rows * 4, cols / self.half) if rings else None
                self._facts = self._title_facts(state, full)
                self._lines = emit(canvas, lambda layer: style(layer, self.tier),
                                   self.tier, ascii=theme.ASCII)
            except Exception:  # one bad frame must not end the session; keep the last lines
                if self._lines is None:
                    self._lines = []
            self.redraw.done(now, (time.perf_counter() - began) * 1000)
        return self._lines
