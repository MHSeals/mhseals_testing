"""The device parts as Status rows: host and state, and Enter to start or stop one (with a confirm).

Each row is one Stack component (harness/components.py): ``title  state [on host]``. A part on
another computer starts and stops as a unit there (harness/remote.py); unreachable (ssh failed)
reads apart from off, and a part a device profile disables says so. Every start and stop asks a
confirm naming the computer; that confirm is the operator's go-ahead a commander part (Nav2, the
mission runner) needs to start remotely. Stop is never refused, armed or not.
"""

from mhseals_hardware.menu import Cancelled

from mhseals_testing.harness.components import ComponentError
from mhseals_testing.ui import theme

LEVELS = {'ours': 'OK', 'external': 'OK', 'starting': 'CHECKING', 'exited': 'FAIL', 'off': 'OFF',
          'unreachable': 'WARN', 'unknown': 'CHECKING', 'disabled': 'N/A'}


def rows(stack, nodes):
    """One Components row per part (``source='part'``)."""
    out = []
    for item in stack.items.values():
        status, detail = stack.state(item.key, nodes)
        where = item.remote or 'here'
        out.append(dict(group='Components', source='part', key=item.key, state=LEVELS[status],
                        name=item.title, value=status if item.remote is None else f'{status} on {where}',
                        detail=theme.SEPARATOR.join(filter(None, [f'{item.part} ({item.profile}) on {where}',
                                                                  detail])),
                        required=False))
    return out


def feed(app):
    """Rows for the snapshot; the units' results become the notice and the journal."""
    stack = getattr(app, 'stack', None)
    if stack is None or not stack.items:
        return []
    for line in stack.units.take_notices() if stack.units is not None else ():
        app.notice = line
        app.ros.emit('unit', detail=line)
    return rows(stack, app.ros.nodes())


def toggle(app, key):
    """Enter on a part row: confirm, then start or stop it where it runs."""
    stack = app.stack
    item = stack.items[key]
    nodes = app.ros.nodes()
    status, detail = stack.state(key, nodes)
    if status in ('disabled', 'external'):
        app.notice = f'{item.title}: {detail or status}'
        return
    # Unreachable or not read yet may still be ours: Stop goes out anyway (harmless if absent).
    stopping = status in ('ours', 'starting') or (status in ('unreachable', 'unknown')
                                                  and stack.running(key))
    verb = 'Stop' if stopping else 'Start'
    where = item.remote or 'this computer'
    info = [('Computer', where), ('Part', f'{item.part} ({item.profile}, {item.kind})')]
    if item.kind == 'commander' and not stopping:
        info.append(('Motion', 'can command the boat once armed with the mux in auto'))
    if item.kind == 'commander' and stopping and app.armed:
        info.append(('Armed', 'stopping it leaves the mux holding'))
    # Cancel is focused, so Enter alone never starts or stops a part.
    try:
        choice = app.choose(f'{item.title} on {where}: {verb}?', [verb, 'Cancel'], {'y': 0, 'n': 1},
                            rows=info, danger=item.kind == 'commander' and not stopping, default=1)
    except Cancelled:
        return
    if choice != verb:
        return
    try:
        if stopping:
            done = stack.stop(key, nodes)
            app.notice = f'Stopping {", ".join(stack.items[name].title for name in done)}'
        else:
            stack.start(key, nodes, commander=True)
            app.notice = f'Starting {item.title} on {where}'
        app.ros.emit('component', action=verb.lower(), component=key, host=where)
    except ComponentError as error:
        app.notice = str(error)


def quit_rows(app):
    """Quit popup rows for the units this TUI keeps alive on other computers, or []."""
    stack = getattr(app, 'stack', None)
    if stack is None or stack.units is None:
        return []
    return [(f'On {host}', f'{", ".join(titles)}: Quit stops them')
            for host, titles in stack.units.held_by_host().items()]
