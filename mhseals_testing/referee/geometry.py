"""Pure track-against-course geometry for the referee (docs/sections/mission.tex, "The referee").

A track is a list of (time, x, y, yaw) in the course frame; a course element is a
dict with ``x``, ``y``, optional ``radius`` and ``label``. The hull is a rectangle
of the boat's measured length and beam about the track point.
"""

import math

HULL = (1.063, 0.895)  # m, length and beam


def reported(events, name):
    """The count the tree reported as ``name``, or None."""
    return next((e.get('count') for e in reversed(events) if e.get('event') == name), None)


def elements(truth, task):
    return [e for e in truth['elements'] if e.get('task') == task]


def points(items, *labels):
    return [(e['x'], e['y'], e.get('radius', 0.2)) for e in items if e['label'] in labels]


def gates(items, red, green, max_width):
    """Gate segments from the truth, nearest pairs first; one colour pairs with itself."""
    reds, greens = points(items, red), points(items, green)
    same = red == green
    pairs = sorted((math.dist(r[:2], g[:2]), i, j) for i, r in enumerate(reds)
                   for j, g in enumerate(greens)
                   if 0.5 < math.dist(r[:2], g[:2]) <= max_width and not (same and j <= i))
    used, segments = set(), []
    for _, i, j in pairs:
        a, b = ('r', i), (('r' if same else 'g'), j)
        if a in used or b in used:
            continue
        used |= {a, b}
        segments.append((reds[i][:2], greens[j][:2]))
    return segments


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def crossings(track, segment):
    """Times the boat's centre crossed the segment between the two buoys, with direction.

    Direction is +1 or -1 by which side it crossed from (the sign of the 2-D
    cross product), so out-and-back through one gate shows as two opposite signs.
    """
    a, b = segment
    found = []
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(track, track[1:]):
        p, q = (x0, y0), (x1, y1)
        d1, d2 = _cross(a, b, p), _cross(a, b, q)
        d3, d4 = _cross(p, q, a), _cross(p, q, b)
        # A sample exactly on the line counts once: for the step that reaches it.
        if d1 != 0 and d1 * d2 <= 0 and d3 * d4 <= 0:
            found.append((t1, 1 if d1 < 0 else -1))
    return found


def hull_clearance(pose, centre, radius, hull=HULL):
    """Distance from a round object's edge to the hull rectangle (negative is contact)."""
    x, y, yaw = pose
    dx, dy = centre[0] - x, centre[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    along, across = c * dx + s * dy, -s * dx + c * dy
    outside = math.hypot(max(abs(along) - hull[0] / 2, 0), max(abs(across) - hull[1] / 2, 0))
    return outside - radius


def contacts(track, items):
    """Labels of the course objects the hull touched, each once."""
    touched = []
    for index, item in enumerate(items):
        if any(hull_clearance(pose[1:], (item['x'], item['y']), item.get('radius', 0.2)) < 0
               for pose in track):
            touched.append(f"{item.get('label') or item.get('template') or 'object'}#{index}")
    return touched


def circled(track, centre, radius=8.0):
    """Degrees the boat went round ``centre`` while within ``radius`` of it."""
    swept, previous = 0.0, None
    for _, x, y, _ in track:
        if not math.isfinite(x + y) or math.dist((x, y), centre) > radius:
            previous = None
            continue
        angle = math.atan2(y - centre[1], x - centre[0])
        if previous is not None:
            swept += math.remainder(angle - previous, 2 * math.pi)
        previous = angle
    return abs(math.degrees(swept))


def held_in_bay(track, bay, seconds):
    """Longest stay (s) with the boat's centre inside the bay, facing into it."""
    c, s = math.cos(bay['yaw']), math.sin(bay['yaw'])
    best, since = 0.0, None
    for t, x, y, yaw in track:
        dx, dy = x - bay['x'], y - bay['y']
        along, across = c * dx + s * dy, -s * dx + c * dy
        facing = math.cos(yaw - (bay['yaw'] + math.pi)) > math.cos(math.radians(30))
        inside = abs(along) <= bay['depth'] / 2 + 0.3 and abs(across) <= bay['width'] / 2
        if inside and facing:
            since = t if since is None else since
            best = max(best, t - since)
        else:
            since = None
    return best
