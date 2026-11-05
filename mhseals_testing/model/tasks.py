"""Mission task hypotheses as plain tuples: ``parse`` reads one ``mhseals.task_hypotheses/1`` message into
``Task`` tuples; pure, stdlib only. ui/minimap/tasks.py draws them and re-exports these names.
"""

import math
from collections import namedtuple

SCHEMA = 'mhseals.task_hypotheses/1'
STATES = ('hypothesis', 'confirmed', 'done', 'failed')

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
