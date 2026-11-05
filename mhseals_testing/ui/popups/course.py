"""Course area (owner decision 2026-11-01): set the day's lat/lon course area corner by corner, live.

Each corner comes from the boat's GPS fix (drive to it: ``g`` adds one there, ``G`` moves the selected one there) or
is nudged in place by metres (``n`` ``s`` ``e`` ``w``, ``+``/``-`` change the step); ``x`` removes, ``J``/``K``
reorder, the arrows select. The minimap previews the corners as they are edited (MapState ``fence_preview``).
``y`` applies them to fence_watchdog (course.corners through SetParameters), checks the readback and keeps them in
~/.config/mhseals/fence.yaml; ``o`` toggles on_exit (alert or return), kept there too.
``boat_agent fence`` takes the same path (ui/feeds/agent_ops.py). Esc leaves; nothing applies unless ``y``.
"""

import yaml

from mhseals_testing.harness import course_area
from mhseals_testing.ui import agent_acts, keymap, theme

STEPS = (1.0, 5.0, 10.0, 25.0)
NUDGE = {'nudge_north': (1, 0), 'nudge_south': (-1, 0), 'nudge_east': (0, 1), 'nudge_west': (0, -1)}   # popup.course
HINT = theme.SEPARATOR.join(('g corner at boat', 'G move here', 'nsew nudge', '+- step', 'x remove', 'JK order',
                             'o on_exit', 'y apply', 'Esc leave'))


def rows(edit, step, status):
    out = [(f"{theme.FOCUS if i == edit.at else ' '} {i + 1}", f'{lat:.7f}, {lon:.7f}')
           for i, (lat, lon) in enumerate(edit.items)]
    why = course_area.problem(edit.items) if edit.items else 'no corners yet: drive to one and press g'
    out += [('step', f'{step:g} m'), ('on exit', (status or {}).get('on_exit') or '?'),
            ('check', why or 'a valid course area')]
    return out


def set_corners(app, client, corners, by=None):
    """SetParameters, readback, then the fence file: ``(ok, why, path)``. ``by`` ({who, why}) names an agent."""
    why = course_area.problem(corners)
    if why:
        return False, why, None
    title = f'Course area: applying {len(corners)} corners'
    try:
        ok, why = agent_acts.in_background(app, title, lambda: client.set_corners(corners), message=title)
    except (TimeoutError, ValueError, RuntimeError) as error:     # fence_watchdog down or not declaring them
        ok, why = False, str(error)
    if not ok:
        return False, why, None
    try:
        path = course_area.save(corners)
    except (ValueError, yaml.YAMLError) as error:
        return False, f'live, but the fence file is not kept: {error}', None
    app.emit('course_area', corners=course_area.flatten(corners), file=str(path), **(by or {}))
    return True, '', path


def set_on_exit(app, client, mode, by=None):
    """on_exit through SetParameters, read back and kept in the fence file: ``(ok, why)``."""
    try:
        ok, why = agent_acts.in_background(app, f'on_exit {mode}', lambda: client.set_on_exit(mode))
    except (TimeoutError, ValueError, RuntimeError) as error:
        ok, why = False, str(error)
    try:
        if ok:
            course_area.save(on_exit=mode)
    except (ValueError, yaml.YAMLError) as error:
        ok, why = False, f'live, but the fence file is not kept: {error}'
    app.emit('fence_on_exit', mode=mode, ok=ok, **(by or {}))
    return ok, why


def apply(app, client, corners):
    """The popup's ``y``; the notice says what happened."""
    ok, why, path = set_corners(app, client, corners)
    if not ok:
        return f'Course area {"not applied" if course_area.problem(corners) else "refused"}: {why}'
    return f'Course area applied ({len(corners)} corners), read back and kept in {path}'


def toggle_on_exit(app, client, status):
    mode = 'return' if (status or {}).get('on_exit', 'alert') == 'alert' else 'alert'
    ok, why = set_on_exit(app, client, mode)
    return f'On exit: {mode}' if ok else f'On exit not changed: {why}'


def open_course(app):
    """The Course area popup; returns the notice."""
    from mhseals_testing.ui import popups
    state, client = app.map_state(), app.fence_client()
    if state is None or client is None:
        return 'Course area: live session owners only'
    status = state.fence_status or {}
    box = {'step': 1, 'notice': ''}
    corners = course_area.pairs(status.get('corners'))
    try:
        corners = corners or course_area.load()
    except (ValueError, yaml.YAMLError) as error:
        box['notice'] = f'fence file: {error}'
    edit = course_area.Corners(corners)
    modal = dict(kind='message', title='Course area',
                 message=box['notice'] or 'Corners of the day\'s course area (lat, lon)',
                 rows=None, width=72, hint=HINT)

    def refresh():
        modal['rows'] = rows(edit, STEPS[box['step']], state.fence_status)
        boat = state.pose[1:3] if state.pose else None
        state.fence_preview = course_area.preview(edit.items, state.fix_ll, boat)

    def handle(key):
        fix = state.fix_ll
        action = keymap.dispatch('popup.course', key)
        if action == 'leave':
            return 'left'
        if action in ('corner_add', 'corner_here'):
            if fix is None:
                box['notice'] = 'no GPS fix yet'
            else:
                (edit.add if action == 'corner_add' else edit.set)(fix)
        elif action in NUDGE:
            north, east = NUDGE[action]
            edit.nudge(north * STEPS[box['step']], east * STEPS[box['step']])
        elif action == 'step_up':
            box['step'] = min(box['step'] + 1, len(STEPS) - 1)
        elif action == 'step_down':
            box['step'] = max(box['step'] - 1, 0)
        elif action == 'remove':
            edit.remove()
        elif action in ('up', 'down'):
            edit.select(-1 if action == 'up' else 1)
        elif action in ('move_up', 'move_down'):
            edit.move(-1 if action == 'move_up' else 1)
        elif action == 'on_exit':
            box['notice'] = toggle_on_exit(app, client, state.fence_status)
        elif action == 'apply':
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
