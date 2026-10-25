"""Issues: device, part and transport problems that outlast a grace period, apart from part status.

A part row says whether the part runs, as its computer reports it; why that report is old or why
the part is missing here is an issue. Issues come from the Stack each tick:

``device:<host>``      the host's status reads fail (ssh); its parts keep their last report
``discovery:<key>``    the part runs, but its proof nodes are not in this computer's DDS graph
``part:<key>``         the part exited
``stray:<key>``        its computer reports it off, but its proof nodes are in the graph: another
                       copy runs somewhere (its start is refused meanwhile)
``probe:<host>``       the host answers, but its process probe fails: parts not run as units there
                       read unknown, never off

Each one shows only once it has lasted its grace (``probes: devices`` in config/health/network.yaml:
``grace_s``, and ``discovery_grace_s`` for discovery), so a blip under it shows nothing. A shown
issue puts the issue glyph in the header and on its row and raises a toast with the cause; it
clears, with a "resolved" toast, as soon as its cause is gone.
"""

from mhseals_testing.harness.components import _up

GRACE_S = 15.0              # a status read failing, or a part exited
DISCOVERY_GRACE_S = 20.0    # nodes of a running part missing from the graph (dropouts last ~5 s)


def settings():
    """``probes: devices`` of the health config ({} when unreadable): period_s, grace_s, discovery_grace_s."""
    try:
        from mhseals_testing.health import config
        return dict(config.load().probes.get('devices') or {})
    except Exception:  # noqa: BLE001 - a bad health file is reported by Health; issues keep defaults
        return {}


def collect(stack, nodes):
    """``{id: (text, kind)}`` of the problems now, before any grace; kind is device, discovery or part."""
    found = {}
    units = stack.units
    if units is not None:
        for host, (why, _, read) in units.trouble().items():
            age = f'from {units.clock() - read:.0f} s ago' if read is not None else 'not read yet'
            found[f'device:{host}'] = (f'{host} does not answer ({why}); its parts show its last '
                                       f'report ({age})', 'device')
        for host, (why, _) in getattr(units, 'probe_trouble', dict)().items():
            found[f'probe:{host}'] = (f'{host} cannot probe its processes ({why}); its parts not '
                                      'run as units read unknown', 'device')
    names = {node.rsplit('/', 1)[-1] for node in nodes}
    for key, item in stack.items.items():
        status, detail = stack.state(key, nodes)
        where = item.remote or 'this computer'
        if status == 'running' and item.nodes and not _up(item, names):
            missing = ', '.join(node for node in item.nodes if node not in names)
            found[f'discovery:{key}'] = (f'{item.title} runs on {where}, but {missing} is not '
                                         'discovered here (DDS)', 'discovery')
        elif stack.stray(key, nodes):
            found[f'stray:{key}'] = (f'{item.title} reads {status} on {where}, but '
                                     f'{", ".join(item.nodes)} in the graph: another copy runs '
                                     'somewhere (its start is refused)', 'stray')
        elif status == 'exited':
            found[f'part:{key}'] = (f'{item.title} exited on {where}: {detail}', 'part')
    return found


class Issues:
    """The grace and clear rules, apart from where problems come from (pure; the clock is passed in)."""

    def __init__(self, grace_s=GRACE_S, discovery_grace_s=DISCOVERY_GRACE_S):
        self.grace = {'device': float(grace_s), 'part': float(grace_s), 'stray': float(grace_s),
                      'discovery': float(discovery_grace_s)}
        self.first = {}       # id -> when the problem was first seen (shown or not)
        self.shown = {}       # id -> its current text, once past its grace

    def update(self, problems, now):
        """Feed ``collect()``; returns (raised [(id, text)], cleared [(id, last text)])."""
        raised, cleared = [], []
        for id in [id for id in self.first if id not in problems]:
            del self.first[id]
            if id in self.shown:
                cleared.append((id, self.shown.pop(id)))
        for id, (text, kind) in problems.items():
            first = self.first.setdefault(id, now)
            if id in self.shown:
                self.shown[id] = text
            elif now - first >= self.grace[kind]:
                self.shown[id] = text
                raised.append((id, text))
        return raised, cleared

    def on_row(self, item):
        """The shown issues about one Stack component (its own, and its host's)."""
        ids = (f'discovery:{item.key}', f'part:{item.key}', f'stray:{item.key}',
               f'device:{item.remote}', f'probe:{item.remote}')
        return [self.shown[id] for id in ids if id in self.shown]


class IssuesFeed(Issues):
    """Ticks ``Issues`` from the App's stack; raised and cleared issues become toasts."""

    def tick(self, app, snapshot, now):
        stack = getattr(app, 'stack', None)
        if stack is None or not stack.items:
            return
        raised, cleared = self.update(collect(stack, app.ros.nodes()), now)
        for id, text in raised:
            app.toasts.push(id, 'warn', text, now)
            app.ros.emit('issue', id=id, detail=text)
        for id, text in cleared:
            app.toasts.push(id, 'ok', f'resolved: {text}', now)
            app.ros.emit('issue_resolved', id=id, detail=text)

    def view(self):
        return {'issues': dict(self.shown)}
