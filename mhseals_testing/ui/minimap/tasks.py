"""Mission task hypotheses and the mission goal on the map (the ``task`` layer, ``T``); pure, no ROS.

``parse`` reads one ``mhseals.task_hypotheses/1`` message into ``Task`` tuples. ``draw`` paints each
task's extent polygon: confirmed solid, a hypothesis dashed (its ``?NN`` label gives the confidence),
done only its corners in the muted ``task.done``, failed solid with a ✗ by its label; at CLOSE zoom
and in, a task is its corners and a hypothesis its label alone. The label (Map tab only) always carries the state as text or a
glyph, so colour is never the only cue. The goal is a ``goal``-rank glyph with its ``why`` beside it.
Layers are ``task:task.<state>``.
"""

import math
from collections import namedtuple

from mhseals_testing.ui.minimap.marks import line

SCHEMA = 'mhseals.task_hypotheses/1'
STATES = ('hypothesis', 'confirmed', 'done', 'failed')
NAMES = {'nav_channel': 'channel', 'follow_path': 'path', 'speed_gate': 'speed', 'speed_mark': 'mark',
         'dock': 'dock', 'home_gate': 'home'}
PERIOD = {'hypothesis': 3, 'confirmed': 1, 'done': 0, 'failed': 1}   # every Nth outline dot; 0 corners only
SUFFIX = {'done': ('✓', '+'), 'failed': ('✗', 'x')}
WHY = 28            # cells of the goal's why drawn beside it
FAR = 4             # a segment longer than this many canvas widths is off the view: skipped
CLOSE = 10.0        # m half-width: at this zoom and in, outlines are corners only

# extent: closed polygon [(x, y)...]; support: [(track_id, label, x, y)...]; map frame, metres.
Task = namedtuple('Task', 'id type state confidence pose extent support')


REACH = 1e5        # m: a map coordinate beyond this is garbage, not a far task


def _finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def placed(*values):
    """Finite map coordinates within REACH."""
    return _finite(*values) and all(abs(v) <= REACH for v in values)


def parse(message):
    """``Task`` per well-formed hypothesis of a message; another schema or frame gives none. An entry
    with a confidence outside 0..1 (or not finite) or a pose off the map is dropped, as are such extent points."""
    if not isinstance(message, dict) or message.get('schema', SCHEMA) != SCHEMA \
            or message.get('frame', 'map') != 'map' or not isinstance(message.get('hypotheses'), (list, type(None))):
        return []
    out = []
    for h in message.get('hypotheses') or []:
        try:
            x, y, yaw = (list(h['pose']) + [0.0])[:3]
            extent = tuple((float(p[0]), float(p[1])) for p in h.get('extent') or ()
                           if isinstance(p, (list, tuple)) and len(p) >= 2 and placed(*p[:2]))
            support = tuple(tuple(s[:4]) for s in h.get('support') or ())
            confidence, state = h.get('confidence', 0.0), h.get('state', 'hypothesis')
            if not (_finite(confidence) and 0.0 <= confidence <= 1.0 and placed(x, y) and _finite(yaw)):
                continue
            task = Task(str(h.get('id', '')), str(h.get('type', '')), state if state in STATES else 'hypothesis',
                        float(confidence), (float(x), float(y), float(yaw)), extent, support)
        except (KeyError, IndexError, TypeError, ValueError, AttributeError):
            continue
        out.append(task)
    return out


def label(task, ascii_=False):
    """``speed ?45`` for a hypothesis, ``dock ✗`` failed, ``channel ✓`` done, the bare name confirmed."""
    name = NAMES.get(task.type, task.type.replace('_', ' '))
    if task.state == 'hypothesis':
        return f'{name} ?{round(task.confidence * 100)}'
    suffix = SUFFIX.get(task.state)
    return f'{name} {suffix[ascii_]}' if suffix else name


def _text(canvas, col, row, text, layer):
    for i, char in enumerate(text[:max(0, canvas.cols - col)]):
        canvas.glyph(col + i, row, char, layer)


def _layer(task):
    return f'task:task.{task.state}'   # no confidence fade: the ?NN label carries it


class _Dots:
    """The outline raster: collects ``dot`` calls once, replayed with ``Canvas.dots`` each frame."""

    def __init__(self):
        self.xs, self.ys, self.layers = [], [], []

    def dot(self, x, y, layer):
        self.xs.append(x)
        self.ys.append(y)
        self.layers.append(layer)


_memo = [None, None, None]   # (task list, projection key, _Dots): one hypothesis set per view


def _outlines(proj, tasks):
    """The outline dots of ``tasks``, rasterized again only when the set or the view changes. A done
    task is history and draws its corners alone; so does every task at CLOSE zoom and in, where an
    outline crosses many rows and each crossing costs SGR bytes of the frame's SSH budget."""
    canvas = proj.canvas
    key = (canvas.dots_wide, canvas.dots_high, proj.centre, proj.half, proj.yaw)
    if _memo[0] is tasks and _memo[1] == key:
        return _memo[2]
    out, far = _Dots(), FAR * (canvas.dots_wide + canvas.dots_high)
    for task in tasks:
        close = proj.half <= CLOSE
        if close and task.state == 'hypothesis':
            continue   # close in, a hypothesis is its ?NN label alone: several boxes overrun the bytes
        layer, period = _layer(task), 0 if close else PERIOD[task.state]
        dots = [proj.dot(p) for p in task.extent]
        for a, b in zip(dots, dots[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) < far:
                out.dot(*a, layer)
                if period:
                    line(out, a, b, layer, period)
    _memo[:] = tasks, key, out
    return out


def draw(proj, tasks, goal=None, ascii_=False, labels=True):
    """Paint ``tasks`` and the mission ``goal`` (x, y, yaw, why) through a ``marks.Projection``."""
    canvas = proj.canvas
    out = _outlines(proj, tasks)
    canvas.dots(out.xs, out.ys, out.layers)
    for task in tasks:
        if labels or not task.extent:
            col, row = proj.cell(task.pose[:2])
            _text(canvas, col + (1 if labels else 0), row, label(task, ascii_) if labels else '·', _layer(task))
    if goal is not None and _finite(goal[0], goal[1]):
        col, row = proj.cell(goal[:2])
        canvas.glyph(col, row, '@' if ascii_ else '◎', 'goal:task.goal')
        why = str(goal[3] or '')
        if labels and why:
            why = why if len(why) <= WHY else why[:WHY - 1] + ('~' if ascii_ else '…')
            if col + 2 + len(why) <= canvas.cols:
                col += 2
            else:   # no room on its right: the row under it (over it at the bottom), against the edge
                col, row = max(0, canvas.cols - len(why)), row + 1 if row + 1 < canvas.rows else row - 1
            _text(canvas, col, row, why.encode('ascii', 'replace').decode() if ascii_ else why, 'task:task.goal')
