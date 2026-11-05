"""The device parts as Status rows: host and state, and Enter to start or stop one (with a confirm).

Each row is one Stack component (harness/components.py): its title and state glyph, the state as
the part's own computer reports it (``state on host`` is the detail line and Enter's popup): running whoever started it (who did is in the detail and the
popup), off, starting, exited, or unknown before the first report. A part on another computer
starts and stops as a unit there (harness/remote.py). A problem past its grace (ui/feeds/issues.py:
the host not answering, the nodes not discovered here, an exit) adds the issue glyph to the row,
never a different state. Every start and stop asks a confirm naming the computer; that confirm is
the operator's go-ahead a commander part (Nav2, the mission runner) needs to start remotely. Stop
is never refused, armed or not, except for a process no TUI started.

The Stack row (the profile's ``stack: true`` parts) starts or stops them together after one confirm,
each through that same ``start``/``stop`` in needs order; their own rows show the progress.

``start`` and ``stop`` are that path after the confirm; the agent desk (ui/feeds/agent_desk/, boat_agent
start/stop COMPONENT) calls them too, with the agent's id, so an agent-started part's row says
``by agent W`` until it stops, and the notice and the journal name the agent.
"""

from mhseals_testing.model import Cancelled

from mhseals_testing.harness.components import ComponentError, moves
from mhseals_testing.ui import theme

LEVELS = {'running': 'OK', 'starting': 'CHECKING', 'exited': 'FAIL', 'off': 'OFF',
          'unknown': 'CHECKING', 'disabled': 'N/A'}
UP = {'running', 'starting'}
# Why a part matters while it is not running, beside its own detail.
WHY_DOWN = {'operator_heartbeat': 'link watchdog holds autonomy until started'}


def stack_keys(stack):
    """The Stack row's parts in start order (needs first); never one that can command motion."""
    return [key for key in stack.start_order('stack') if not moves(stack.items[key])]


def stack_row(stack, nodes):
    """The Stack row (``source='stack'``): how many of its parts are up; None without any."""
    keys = stack_keys(stack)
    if not keys:
        return None
    states = [stack.state(key, nodes)[0] for key in keys]
    up = sum(status in UP for status in states)
    level = ('OK' if all(status == 'running' for status in states) else
             'CHECKING' if any(status in ('starting', 'unknown') for status in states) else
             'OFF' if up == 0 else 'WARN')
    titles = [stack.items[key].title for key in keys]
    carried = [item.title for item in stack.items.values()
               if item.carried_by in titles]
    return dict(group='Components', source='stack', key='stack', state=level, name='Stack',
                value=f'{up}/{len(keys)} up', required=False,
                detail=theme.SEPARATOR.join(filter(None, [
                    'Enter starts or stops ' + ', '.join(titles),
                    f'{", ".join(carried)} with them' if carried else ''])))


def rows(stack, nodes, issues=None):
    """One Components row per part (``source='part'``), after the Stack row; ``issues``
    (feeds.issues.Issues) marks rows."""
    out = list(filter(None, [stack_row(stack, nodes)]))
    for item in stack.items.values():
        status, detail = stack.state(item.key, nodes)
        if status not in UP and item.part in WHY_DOWN:
            detail = theme.SEPARATOR.join(filter(None, [WHY_DOWN[item.part], detail]))
        # The row is the name, the state glyph and at most an agent's id and the issue glyph: the
        # state and its computer are the detail line and Enter's popup, so no host name crowds the column.
        by = getattr(stack, 'by', {}).get(item.key)
        found = issues.on_row(item) if issues is not None else []
        value = ' '.join(filter(None, [f'by agent {by}' if by else '', theme.PALETTE.glyph('issue') if found else '']))
        out.append(dict(group='Components', source='part', key=item.key, state=LEVELS[status],
                        name=item.title, value=value,
                        unreached=item.remote is not None and status == 'unknown', issue=bool(found),
                        detail=theme.SEPARATOR.join(filter(None, [f'{status} on {item.remote or "this computer"}',
                                                                  f'{item.part} ({item.profile})',
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
        app.say(line)
        app.ros.emit('unit', detail=line)
    for line in stack.advance(app.ros.nodes()):     # the starts waiting for their needs
        app.say(line)
        app.ros.emit('component', action='start_failed', detail=line)
    return rows(stack, app.ros.nodes(), getattr(app, 'issues', None))


def toggle(app, key):
    """Enter on a part row: confirm, then start or stop it where it runs."""
    stack = app.stack
    item = stack.items[key]
    nodes = app.ros.nodes()
    status, detail = stack.state(key, nodes)
    if status == 'disabled':
        app.say(f'{item.title}: {detail or status}')
        return
    if item.carried_by:
        return toggle_carried(app, key, status)
    # Not read yet may still be a unit we hold: Stop goes out anyway (harmless if absent).
    stopping = status in UP or (status == 'unknown' and stack.running(key))
    verb = 'Stop' if stopping else 'Start'
    where = item.remote or 'this computer'
    info = [('State', status), ('Computer', where), ('Part', f'{item.part} ({item.profile}, {item.kind})')]
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
    title = f'{item.title} on {where}: {verb}?'
    if not stopping:      # the parts it needs that are not up start first, each once the one before runs
        try:
            needs = [stack.items[name].title for name in stack.plan(key, nodes)[:-1]]
        except ComponentError as error:
            app.say(str(error))
            return
        if needs:
            title = f"{item.title} needs {', '.join(needs)}: start all {len(needs) + 1}?"
            info.append(('Also starts', ', '.join(needs)))
    # Cancel is focused, so Enter alone never starts or stops a part.
    try:
        choice = app.choose(title, [verb, 'Cancel'], {'y': 0, 'n': 1},
                            rows=info, danger=item.kind == 'commander' and not stopping, default=1)
    except Cancelled:
        return
    if choice != verb:
        return
    try:
        stop(app, key) if stopping else start(app, key)
    except ComponentError as error:
        app.say(str(error))


def carrier(stack, key):
    """The key of the component whose launch brings ``key`` (the Odom stack for the safety watchdogs), or None."""
    title = stack.items[key].carried_by
    return next((other for other, item in stack.items.items() if item.title == title and other != key), None)


def toggle_carried(app, key, status):
    """Enter on a part another component's launch brings: never a second copy, so starting it starts that
    carrier (the same confirm as its own row); a carrier up without it says so with the fix."""
    stack, item = app.stack, app.stack.items[key]
    host = carrier(stack, key)
    if host is None or status in UP:
        app.say(f'{item.title} run with {item.carried_by}: start or stop that from its row')
        return
    lead = stack.items[host]
    lead_status, _ = stack.state(host, app.ros.nodes())
    where = lead.remote or 'this computer'
    info = [('State', f'{status} ({item.title} start with {lead.title})'), ('Computer', where)]
    if lead_status == 'unknown':
        app.say(f'{lead.title} on {where} not read yet; try again shortly')
        return
    if lead_status in UP:
        info += [('Problem', f'{lead.title} is {lead_status} but {item.title} are not up: it was started '
                             'outside the TUI or without them'),
                 ('Fix', f'restart {lead.title} from the TUI: Enter on its row stops it, Enter again starts '
                         f'it with {item.title} (one started outside the TUI stops where it was started)')]
        try:
            app.choose(f'{item.title}: not running', ['Close'], {'c': 0}, rows=info)
        except Cancelled:
            pass
        return
    try:
        choice = app.choose(f'{item.title} start with {lead.title}: start {lead.title}?', ['Start', 'Cancel'],
                            {'y': 0, 'n': 1}, rows=info, default=1)
    except Cancelled:
        return
    if choice != 'Start':
        return
    try:
        start(app, host)
    except ComponentError as error:
        app.say(str(error))


def toggle_stack(app):
    """Enter on the Stack row: one confirm listing each part, then start the off ones or stop the up ones."""
    stack = app.stack
    keys = stack_keys(stack)
    nodes = app.ros.nodes()
    states = {key: stack.state(key, nodes) for key in keys}
    off = [key for key in keys if states[key][0] not in UP and states[key][0] != 'disabled']
    up = [key for key in keys if states[key][0] in UP
          or (states[key][0] == 'unknown' and stack.running(key))]
    info = [(stack.items[key].title, f'{states[key][0]} on {stack.items[key].remote or "this computer"}')
            for key in keys]
    info += [(item.title, f'with {item.carried_by}') for item in stack.items.values()
             if item.carried_by in {stack.items[key].title for key in keys}]
    others = [stack.items[other].title for key in up for other in stack._dependants(key)
              if other not in keys and stack.running(other)]
    if others:
        info.append(('Stop also stops', ', '.join(dict.fromkeys(others))))
    options = [*(['Start'] if off else []), *(['Stop'] if up else []), 'Cancel']
    shortcuts = {{'Start': 's', 'Stop': 'x', 'Cancel': 'n'}[name]: index for index, name in enumerate(options)}
    try:
        choice = app.choose('Stack: ' + ' or '.join(options[:-1]) + '?' if len(options) > 1 else 'Stack',
                            options, shortcuts, rows=info, default=len(options) - 1)
    except Cancelled:
        return
    if choice == 'Start':
        start_stack(app)
    elif choice == 'Stop':
        stop_stack(app)


def start_stack(app):
    """Start the Stack row's parts that are off, needs first, each as its own row starts it; a part
    that does not start skips the ones that need it. Returns (started keys, {key: why not})."""
    stack = app.stack
    started, failed = [], {}
    for key in stack_keys(stack):
        item = stack.items[key]
        need = next((need for need in item.needs if need in failed), None)
        if need is not None:
            failed[key] = f'{stack.items[need].title} did not start'
            continue
        if stack.state(key, app.ros.nodes())[0] in UP | {'disabled'}:
            continue
        try:
            start(app, key)
            started.append(key)
        except ComponentError as error:
            failed[key] = str(error)
    app.say('; '.join(filter(None, [
   f"Stack: starting {', '.join(stack.items[key].title for key in started)}" if started else
   'Stack: nothing to start' if not failed else '',
   *(f'{stack.items[key].title} not started: {why}' for key, why in failed.items())])))
    return started, failed


def stop_stack(app):
    """Stop the Stack row's parts that are up, last started first (each with its dependants, as its own
    row stops it). Returns (stopped keys, {key: why not})."""
    stack = app.stack
    stopped, failed = [], {}
    for key in reversed(stack_keys(stack)):
        status = stack.state(key, app.ros.nodes())[0]
        if key in stopped or not (status in UP or (status == 'unknown' and stack.running(key))):
            continue
        try:
            stopped += stop(app, key)
        except ComponentError as error:
            failed[key] = str(error)
    app.say('; '.join(filter(None, [
   f"Stack: stopping {', '.join(stack.items[key].title for key in dict.fromkeys(stopped))}"
   if stopped else 'Stack: nothing to stop' if not failed else '',
   *(f'{stack.items[key].title} not stopped: {why}' for key, why in failed.items())])))
    return stopped, failed


def start(app, key, who=None):
    """Start ``key`` where it runs (the operator's confirmed Enter, or agent ``who``); ComponentError refuses.

    The operator's confirm is the go-ahead a remote commander part needs; an agent never starts one
    (the desk refuses commanders before this)."""
    stack, item = app.stack, app.stack.items[key]
    where = item.remote or 'this computer'
    plan = stack.start(key, app.ros.nodes(), commander=who is None, by=who)
    needs = [stack.items[name].title for name in plan[:-1]]
    app.say(f"{f'Agent {who}: s' if who else 'S'}tarting {item.title} on {where}"
            + (f" after {', '.join(needs)}" if needs else ''))
    app.ros.emit('component', action='start', component=key, host=where,
                 **({'needs': plan[:-1]} if needs else {}), **({'by': 'agent', 'who': who} if who else {}))
    return plan


def stop(app, key, who=None):
    """Stop ``key`` and its running dependants (the operator's, or agent ``who``); returns the keys stopped."""
    stack, item = app.stack, app.stack.items[key]
    where = item.remote or 'this computer'
    done = stack.stop(key, app.ros.nodes())
    app.say(f"{f'Agent {who}: s' if who else 'S'}topping "
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
