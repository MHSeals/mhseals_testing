"""The device parts as Status rows: host and state, and Enter to start or stop one (with a confirm).

Each row is one Stack component (harness/components.py): ``title  state [on host]``, the state as
the part's own computer reports it: running whoever started it (who did is in the detail and the
popup), off, starting, exited, or unknown before the first report. A part on another computer
starts and stops as a unit there (harness/remote.py). A problem past its grace (ui/feeds/issues.py:
the host not answering, the nodes not discovered here, an exit) adds the issue glyph to the row,
never a different state. Every start and stop asks a confirm naming the computer; that confirm is
the operator's go-ahead a commander part (Nav2, the mission runner) needs to start remotely. Stop
is never refused, armed or not, except for a process no TUI started.

``start`` and ``stop`` are that path after the confirm; the agent desk (ui/feeds/agent.py, boat_agent
start/stop COMPONENT) calls them too, with the agent's id, so an agent-started part's row says
``by agent W`` until it stops, and the notice and the journal name the agent.
"""

from mhseals_hardware.menu import Cancelled

from mhseals_testing.harness.components import ComponentError
from mhseals_testing.ui import theme

LEVELS = {'running': 'OK', 'starting': 'CHECKING', 'exited': 'FAIL', 'off': 'OFF',
          'unknown': 'CHECKING', 'disabled': 'N/A'}
UP = {'running', 'starting'}
# Why a part matters while it is not running, beside its own detail.
WHY_DOWN = {'operator_heartbeat': 'link watchdog holds autonomy until started'}


def rows(stack, nodes, issues=None):
    """One Components row per part (``source='part'``); ``issues`` (feeds.issues.Issues) marks rows."""
    out = []
    for item in stack.items.values():
        status, detail = stack.state(item.key, nodes)
        if status not in UP and item.part in WHY_DOWN:
            detail = theme.SEPARATOR.join(filter(None, [WHY_DOWN[item.part], detail]))
        where = item.remote or 'here'
        value = status if item.remote is None else f'{status} on {where}'
        by = getattr(stack, 'by', {}).get(item.key)
        if by:
            value += f' by agent {by}'
        found = issues.on_row(item) if issues is not None else []
        if found:
            value += ' ' + theme.PALETTE.glyph('issue')
        out.append(dict(group='Components', source='part', key=item.key, state=LEVELS[status],
                        name=item.title, value=value,
                        detail=theme.SEPARATOR.join(filter(None, [f'{item.part} ({item.profile}) on {where}',
                                                                  f'started by agent {by}' if by else '',
                                                                  detail, *found])),
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
    return rows(stack, app.ros.nodes(), getattr(app, 'issues', None))


def toggle(app, key):
    """Enter on a part row: confirm, then start or stop it where it runs."""
    stack = app.stack
    item = stack.items[key]
    nodes = app.ros.nodes()
    status, detail = stack.state(key, nodes)
    if status == 'disabled':
        app.notice = f'{item.title}: {detail or status}'
        return
    # Not read yet may still be a unit we hold: Stop goes out anyway (harmless if absent).
    stopping = status in UP or (status == 'unknown' and stack.running(key))
    verb = 'Stop' if stopping else 'Start'
    where = item.remote or 'this computer'
    info = [('Computer', where), ('Part', f'{item.part} ({item.profile}, {item.kind})')]
    if status in UP and detail:
        info.append(('Owner', detail))          # ownership is a detail, never the state
    if getattr(stack, 'by', {}).get(key):
        info.append(('Started by', f'agent {stack.by[key]}'))
    issues = getattr(app, 'issues', None)
    for text in issues.on_row(item) if issues is not None else ():
        info.append(('Issue', text))
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
        stop(app, key) if stopping else start(app, key)
    except ComponentError as error:
        app.notice = str(error)


def start(app, key, who=None):
    """Start ``key`` where it runs (the operator's confirmed Enter, or agent ``who``); ComponentError refuses.

    The operator's confirm is the go-ahead a remote commander part needs; an agent never starts one
    (the desk refuses commanders before this)."""
    stack, item = app.stack, app.stack.items[key]
    where = item.remote or 'this computer'
    stack.start(key, app.ros.nodes(), commander=who is None, by=who)
    app.notice = f"{f'Agent {who}: s' if who else 'S'}tarting {item.title} on {where}"
    app.ros.emit('component', action='start', component=key, host=where,
                 **({'by': 'agent', 'who': who} if who else {}))
    return [key]


def stop(app, key, who=None):
    """Stop ``key`` and its running dependants (the operator's, or agent ``who``); returns the keys stopped."""
    stack, item = app.stack, app.stack.items[key]
    where = item.remote or 'this computer'
    done = stack.stop(key, app.ros.nodes())
    app.notice = (f"{f'Agent {who}: s' if who else 'S'}topping "
                  f"{', '.join(stack.items[name].title for name in done)}")
    app.ros.emit('component', action='stop', component=key, host=where,
                 **({'by': 'agent', 'who': who} if who else {}))
    return done


def quit_rows(app):
    """Quit popup rows for the units this TUI keeps alive on other computers, or [].

    Quit stops the ones it started; the ones it adopted (another TUI may have started them)
    keep running unless the operator picks the option that stops them too.
    """
    stack = getattr(app, 'stack', None)
    if stack is None or stack.units is None:
        return []
    return [*((f'On {host}', f'{", ".join(titles)}: Quit stops them')
              for host, titles in stack.units.held_by_host(adopted=False).items()),
            *((f'On {host}', f'{", ".join(titles)}: adopted, Quit keeps them running')
              for host, titles in stack.units.held_by_host(adopted=True).items())]
