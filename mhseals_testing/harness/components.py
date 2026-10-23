"""Stack components (odometry, sensors, navigation, ...) a profile can start and stop.

A profile's ``components:`` mapping replaces the old ``bringup:`` and the measurement
launch file: each entry is one ``ros2 launch`` (ours to start and stop), or a
``remote`` source that runs on another host (never ours). ``state`` classifies a
component from the node graph and our own children, and ``Stack`` starts and stops
them. A process we did not start is ``external`` and is never stopped.
"""

from dataclasses import dataclass

STARTING_LIMIT = 20.0   # s before a started component without its proof nodes warns
KEYS = {'title', 'launch', 'args', 'nodes', 'needs', 'limits', 'remote', 'topic', 'start'}


@dataclass(frozen=True)
class Component:
    key: str
    title: str
    argv: tuple = ()          # empty for a remote component: nothing here can start it
    nodes: tuple = ()         # nodes whose presence proves it is running
    needs: tuple = ()         # component keys that must be up first
    remote: str | None = None
    topic: str | None = None
    limits: bool = False      # takes the session's Nav2 limits file (set by the caller)
    start: bool = False       # started by --start-stack


class ComponentError(RuntimeError):
    """A start or stop the stack refuses; ``keys`` names the components involved."""

    def __init__(self, message, keys=()):
        super().__init__(message)
        self.keys = tuple(keys)


def parse_components(data, where='components'):
    """Validate a profile's ``components`` mapping; returns it unchanged (or {})."""
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f'{where}: mapping of component name to settings')
    for key, spec in data.items():
        here = f'{where}.{key}'
        if not isinstance(spec, dict) or not set(spec) <= KEYS:
            raise ValueError(f'{here}: mapping with keys from {sorted(KEYS)}')
        launch = spec.get('launch')
        if (launch is None) == (spec.get('remote') is None):
            raise ValueError(f'{here}: exactly one of launch or remote')
        if launch is not None and not (isinstance(launch, list) and len(launch) == 2):
            raise ValueError(f'{here}.launch: [package, launch file]')
        if not isinstance(spec.get('args', {}), dict):
            raise ValueError(f'{here}.args: mapping of launch arguments')
        for name in ('nodes', 'needs'):
            if not isinstance(spec.get(name, []), list):
                raise ValueError(f'{here}.{name}: list')
        for need in spec.get('needs', []):
            if need not in data or need == key:
                raise ValueError(f'{here}.needs: unknown component {need!r}')
    for key in data:
        if _cycle(data, key):
            raise ValueError(f'{where}.{key}.needs: cycle through {key!r}')
    return data


def _cycle(data, start):
    """True when ``start`` is reachable from its own needs."""
    seen, todo = set(), list(data[start].get('needs', []))
    while todo:
        key = todo.pop()
        if key == start:
            return True
        if key not in seen:
            seen.add(key)
            todo += data[key].get('needs', [])
    return False


def components(profile, overlay=None):
    """The profile's components in file order. ``overlay`` is {key: {arg: value}}."""
    overlay = overlay or {}
    result = []
    for key, spec in profile.components.items():
        argv = ()
        if 'launch' in spec:
            args = {**spec.get('args', {}), **overlay.get(key, {})}
            argv = ('ros2', 'launch', *spec['launch'], *(f'{k}:={v}' for k, v in args.items()))
        result.append(Component(
            key, spec.get('title', key.capitalize()), argv, tuple(spec.get('nodes', ())),
            tuple(spec.get('needs', ())), spec.get('remote'), spec.get('topic'),
            bool(spec.get('limits', False)), bool(spec.get('start', False))))
    return tuple(result)


def state(component, nodes, processes, now, started=None):
    """(state, detail): off, starting, ours, external or exited, as the Status row shows it.

    ``processes`` is the ProcessManager, ``nodes`` the node names in the graph (namespaces are ignored),
    ``started`` the time we started the component (same clock as ``now``).
    """
    nodes = {node.rsplit('/', 1)[-1] for node in nodes}
    up = bool(component.nodes) and all(node in nodes for node in component.nodes)
    if component.remote:
        return ('external', component.remote) if up else ('off', '')
    if processes.running(component.key):
        if up or not component.nodes:
            return 'ours', 'ours'
        if started is not None and now - started > STARTING_LIMIT:
            missing = next(node for node in component.nodes if node not in nodes)
            return 'starting', f'WARN: no {missing}'
        return 'starting', ''
    code = processes.returncode(component.key)
    if code is not None:
        return 'exited', f'exit {code} {processes.log_directory / (component.key + ".log")}'
    return ('external', 'external') if up else ('off', '')


class Stack:
    """Starts and stops a profile's components; remembers when each one started."""

    def __init__(self, items, processes, clock):
        self.items = {item.key: item for item in items}
        self.processes = processes
        self.clock = clock
        self.started = {}

    def state(self, key, nodes):
        return state(self.items[key], nodes, self.processes, self.clock(), self.started.get(key))

    def start(self, key, nodes):
        """Start an off component; refuses (naming them) while its needs are off."""
        item = self.items[key]
        status = self.state(key, nodes)[0]
        if not item.argv:
            raise ComponentError(f'{item.title} runs on {item.remote}; start it there', [key])
        if status in ('ours', 'starting', 'external'):
            raise ComponentError(f'{item.title} is already running', [key])
        off = [need for need in item.needs if self.state(need, nodes)[0] in ('off', 'exited')]
        if off:
            raise ComponentError(f'Start {self.items[off[0]].title} first', off)
        self.processes.start(key, item.argv)
        self.started[key] = self.clock()

    def stop(self, key, nodes):
        """Stop ours, dependants first. Never stops an external process; returns the keys stopped."""
        status, _ = self.state(key, nodes)
        if status == 'external':
            raise ComponentError('running outside this session; stop it where it was started', [key])
        order = [other for other in self._dependants(key) if self.processes.running(other)]
        if self.processes.running(key):
            order.append(key)
        for name in order:
            self.processes.stop(name)
            self.started.pop(name, None)
        return order

    def _dependants(self, key):
        """Components that need ``key``, directly or not, deepest first."""
        found = []
        for item in self.items.values():
            if key in item.needs:
                found += [*self._dependants(item.key), item.key]
        return list(dict.fromkeys(found))

    def start_order(self):
        """Keys marked ``start``, each after its needs."""
        order = []

        def visit(key):
            for need in self.items[key].needs:
                visit(need)
            if key not in order:
                order.append(key)
        for item in self.items.values():
            if item.start:
                visit(item.key)
        return [key for key in order if self.items[key].argv]
