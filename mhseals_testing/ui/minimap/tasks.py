"""Mission task hypotheses and the mission goal on the map (the ``task`` layer, ``T``); pure, no ROS.

``parse`` reads one ``mhseals.task_hypotheses/1`` message into ``Task`` tuples. ``draw`` paints each
task's extent polygon: confirmed solid, a hypothesis dashed and faded by its confidence, done
every 2nd dot in the muted ``task.done``, failed solid with a ✗ by its label. The label (Map tab
only) always carries the state as text or a glyph, so colour is never the only cue. The goal
is a ``goal``-rank glyph with its ``why`` beside it. Layers are ``task:task.<state>@<fade>``.
"""

import math
from collections import namedtuple

from mhseals_testing.ui.minimap.marks import line, strength

SCHEMA = 'mhseals.task_hypotheses/1'
STATES = ('hypothesis', 'confirmed', 'done', 'failed')
NAMES = {'nav_channel': 'channel', 'follow_path': 'path', 'speed_gate': 'speed', 'speed_mark': 'mark',
         'dock': 'dock', 'home_gate': 'home'}
PERIOD = {'hypothesis': 3, 'confirmed': 1, 'done': 2, 'failed': 1}   # every Nth outline dot
SUFFIX = {'done': ('✓', '+'), 'failed': ('✗', 'x')}
WHY = 28            # cells of the goal's why drawn beside it
FAR = 4             # a segment longer than this many canvas widths is off the view: skipped

# extent: closed polygon [(x, y)...]; support: [(track_id, label, x, y)...]; map frame, metres.
Task = namedtuple('Task', 'id type state confidence pose extent support')


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def parse(message):
    """``Task`` per well-formed hypothesis of a message; another schema or frame gives none."""
    if not isinstance(message, dict) or message.get('schema', SCHEMA) != SCHEMA \
            or message.get('frame', 'map') != 'map':
        return []
    out = []
    for h in message.get('hypotheses') or []:
        try:
            x, y, yaw = (list(h['pose']) + [0.0])[:3]
            extent = tuple((float(p[0]), float(p[1])) for p in h.get('extent') or () if _finite(*p[:2]))
            support = tuple(tuple(s[:4]) for s in h.get('support') or ())
            task = Task(str(h.get('id', '')), str(h.get('type', '')), h.get('state', 'hypothesis'),
                        float(h.get('confidence', 0.0)), (float(x), float(y), float(yaw)), extent, support)
        except (KeyError, TypeError, ValueError):
            continue
        if _finite(*task.pose[:2]):
            out.append(task._replace(state=task.state if task.state in STATES else 'hypothesis'))
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


def draw(proj, tasks, goal=None, ascii_=False, labels=True):
    """Paint ``tasks`` and the mission ``goal`` (x, y, yaw, why) through a ``marks.Projection``."""
    canvas = proj.canvas
    far = FAR * (canvas.dots_wide + canvas.dots_high)
    for task in tasks:
        token = 'task.' + task.state
        fade = strength(task.confidence) if task.state == 'hypothesis' else 100
        layer = f'task:{token}' + (f'@{fade}' if fade < 100 else '')
        dots = [proj.dot(p) for p in task.extent]
        for a, b in zip(dots, dots[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) < far:
                canvas.dot(*a, layer)
                line(canvas, a, b, layer, PERIOD[task.state])
        if labels or not dots:
            col, row = proj.cell(task.pose[:2])
            _text(canvas, col + (1 if labels else 0), row, label(task, ascii_) if labels else '·', layer)
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
