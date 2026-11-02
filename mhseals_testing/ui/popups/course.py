"""Course area (owner decision 2026-11-01): set the day's lat/lon course area corner by corner, live.

Each corner comes from the boat's GPS fix (drive to it: ``g`` adds one there, ``G`` moves the selected one there) or
is nudged in place by metres (``n`` ``s`` ``e`` ``w``, ``+``/``-`` change the step); ``x`` removes, ``J``/``K``
reorder, the arrows select. The minimap previews the corners as they are edited (MapState ``fence_preview``).
``y`` applies them to fence_watchdog (course.corners through SetParameters), checks the readback and keeps them in
~/.config/mhseals/fence.yaml; ``o`` toggles on_exit (alert or return). Esc leaves; nothing applies unless ``y``.
"""

from mhseals_testing.harness import course_area
from mhseals_testing.ui import agent_acts, theme

STEPS = (1.0, 5.0, 10.0, 25.0)
NUDGE = {'n': (1, 0), 's': (-1, 0), 'e': (0, 1), 'w': (0, -1)}
HINT = theme.SEPARATOR.join(('g corner at boat', 'G move here', 'nsew nudge', '+- step', 'x remove', 'JK order',
                             'o on_exit', 'y apply', 'Esc leave'))


def rows(edit, step, status):
    out = [(f"{theme.FOCUS if i == edit.at else ' '} {i + 1}", f'{lat:.7f}, {lon:.7f}')
           for i, (lat, lon) in enumerate(edit.items)]
    why = course_area.problem(edit.items) if edit.items else 'no corners yet: drive to one and press g'
    out += [('step', f'{step:g} m'), ('on exit', (status or {}).get('on_exit') or '?'),
            ('check', why or 'a valid course area')]
    return out


def apply(app, client, corners):
    """SetParameters, readback, then the fence file; the notice says what happened."""
    why = course_area.problem(corners)
    if why:
        return f'Course area not applied: {why}'
    title = f'Course area: applying {len(corners)} corners'
    try:
        ok, why = agent_acts.in_background(app, title, lambda: client.set_corners(corners), message=title)
    except (TimeoutError, ValueError, RuntimeError) as error:     # fence_watchdog down or not declaring them
        ok, why = False, str(error)
    if not ok:
        return f'Course area refused: {why}'
    path = course_area.save(corners)
    app.emit('course_area', corners=course_area.flatten(corners), file=str(path))
    return f'Course area applied ({len(corners)} corners), read back and kept in {path}'


def toggle_on_exit(app, client, status):
    mode = 'return' if (status or {}).get('on_exit', 'alert') == 'alert' else 'alert'
    try:
        ok, why = agent_acts.in_background(app, f'on_exit {mode}', lambda: client.set_on_exit(mode))
    except (TimeoutError, ValueError, RuntimeError) as error:
        ok, why = False, str(error)
    app.emit('fence_on_exit', mode=mode, ok=ok)
    return f'On exit: {mode}' if ok else f'On exit not changed: {why}'


def open_course(app):
    """The Course area popup; returns the notice."""
    from mhseals_testing.ui import popups
    state, client = app.map_state(), app.fence_client()
    if state is None or client is None:
        return 'Course area: live session owners only'
    status = state.fence_status or {}
    edit = course_area.Corners(course_area.pairs(status.get('corners')) or course_area.load())
    box = {'step': 1, 'notice': ''}
    modal = dict(kind='message', title='Course area', message='Corners of the day\'s course area (lat, lon)',
                 rows=None, width=72, hint=HINT)

    def refresh():
        modal['rows'] = rows(edit, STEPS[box['step']], state.fence_status)
        boat = state.pose[1:3] if state.pose else None
        state.fence_preview = course_area.preview(edit.items, state.fix_ll, boat)

    def handle(key):
        fix = state.fix_ll
        if key == 'escape':
            return 'left'
        if key in ('g', 'G'):
            if fix is None:
                box['notice'] = 'no GPS fix yet'
            else:
                (edit.add if key == 'g' else edit.set)(fix)
        elif key in NUDGE:
            north, east = NUDGE[key]
            edit.nudge(north * STEPS[box['step']], east * STEPS[box['step']])
        elif key in ('+', '='):
            box['step'] = min(box['step'] + 1, len(STEPS) - 1)
        elif key in ('-', '_'):
            box['step'] = max(box['step'] - 1, 0)
        elif key == 'x':
            edit.remove()
        elif key in ('up', 'down'):
            edit.select(-1 if key == 'up' else 1)
        elif key in ('K', 'J'):
            edit.move(-1 if key == 'K' else 1)
        elif key == 'o':
            box['notice'] = toggle_on_exit(app, client, state.fence_status)
        elif key == 'y':
            box['notice'] = apply(app, client, list(edit.items))
            if box['notice'].startswith('Course area applied'):
                return 'applied'
        refresh()
        modal['message'] = box['notice'] or modal['message']
        return None
    refresh()
    try:
        popups.popup(app, modal, handle, refresh)
    finally:
        state.fence_preview = []
    return box['notice'] or 'Course area unchanged'
