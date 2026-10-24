"""Stack components (odometry, sensors, navigation, ...) a profile can start and stop.

A profile's ``components:`` mapping replaces the old ``bringup:`` and the measurement
launch file: each entry is one ``ros2 launch`` (ours to start and stop), or a
``remote`` source that runs on another host (never ours). ``state`` classifies a
component from the node graph and our own children, and ``Stack`` starts and stops
them. A process we did not start is ``external`` and is never stopped.
"""

from dataclasses import dataclass

from mhseals_testing import nav2_limits

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


ODOMETRY_LAUNCHES = {('mhseals_nav', name)
                     for name in ('odom.launch.py', 'ekf.launch.py', 'robot.launch.py')}
ODOMETRY_INPUTS = frozenset({
    'odometry_startup_file', 'odometry_deployment_file', 'ekf_startup_context',
    'robot_state_publish_frequency', 'mavros_velocity_source', 'mavros_velocity_stddev',
    'mavros_velocity_max_age', 'mavros_velocity_min_stddev', 'mavros_velocity_no_fix_stddev',
    'ekf_delay', 'navsat_transform_config_file', 'ekf_overlay_file',
    'ekf_local_config_file', 'ekf_global_config_file',
})


def validate_localization_profile(profile, arguments):
    """Refuse unsupported managed localization graphs and contradictory clocks."""
    managed = [key for key, spec in (profile.components or {}).items()
               if tuple(spec.get('launch', ())) in ODOMETRY_LAUNCHES]
    if not managed:
        return
    expected = {'map_frame': 'map', 'odom_frame': 'odom',
                'base_frame': 'base_link', 'odometry': '/odom/local'}
    for name, value in expected.items():
        if getattr(profile, name, None) != value:
            raise ValueError(f'managed localization requires {name}={value}')
    if type(profile.use_sim_time) is not bool:
        raise ValueError('managed localization use_sim_time must be a boolean')
    clock = 'true' if profile.use_sim_time else 'false'
    for key in managed:
        supplied = arguments[key].get('sim', clock)
        if type(supplied) is bool:
            supplied = 'true' if supplied else 'false'
        if supplied not in ('true', 'false') or supplied != clock:
            raise ValueError(f'managed localization {key}.sim must match use_sim_time')
        arguments[key]['sim'] = clock


def odometry_arguments(profile, overlay):
    """One explicit odometry selection for both parent and separately managed EKFs."""
    arguments = {key: {**spec.get('args', {}), **overlay.get(key, {})}
                 for key, spec in (profile.components or {}).items()}
    selected = {}
    for key, spec in (profile.components or {}).items():
        if tuple(spec.get('launch', ())) not in ODOMETRY_LAUNCHES:
            continue
        for name, value in arguments[key].items():
            if name not in ODOMETRY_INPUTS or value == '':
                continue
            if name in selected and str(selected[name]) != str(value):
                raise ValueError(f'conflicting odometry selection: {name}')
            selected[name] = value
    for key, spec in (profile.components or {}).items():
        if tuple(spec.get('launch', ())) in ODOMETRY_LAUNCHES:
            arguments[key].update(selected)
    return arguments


def components(profile, overlay=None, selection=None):
    """The profile's components in file order. ``overlay`` is {key: {arg: value}}."""
    overlay = overlay or {}
    result = []
    arguments = odometry_arguments(profile, overlay)
    validate_localization_profile(profile, arguments)
    for key, spec in (profile.components or {}).items():
        argv = ()
        if 'launch' in spec:
            args = dict(arguments[key])
            if selection is not None:
                from mhseals_testing.startup_owner import NAV_LAUNCHES, RESERVED
                if tuple(spec['launch']) in NAV_LAUNCHES:
                    if 'navigation' not in selection.domains or RESERVED & args.keys():
                        raise ValueError('invalid managed navigation selection arguments')
                    args.update(selection.arguments())
            argv = ('ros2', 'launch', *spec['launch'], *(f'{k}:={v}' for k, v in args.items()))
            if spec.get('limits'):   # the lasting nav2.yaml must survive every (re)start
                argv += (nav2_limits.launch_argument(),)
        result.append(Component(
            key, spec.get('title', key.capitalize()), argv, tuple(spec.get('nodes', ())),
            tuple(spec.get('needs', ())), spec.get('remote'), spec.get('topic'),
            bool(spec.get('limits', False)), bool(spec.get('start', False))))
    return tuple(result)


GHOST_S = 25.0   # robot_localization exits without announcing itself; its names wait for the DDS lease (~20 s)


def stopped_recently(component, processes, now):
    """True while the names of a child we just stopped may still be on the graph."""
    at = getattr(processes, 'stopped_at', {}).get(component.key)
    return at is not None and now - at < GHOST_S


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
    if up and stopped_recently(component, processes, now):
        return 'off', ''                       # leftover names of our own stopped child, not a foreign process
    return ('external', 'external') if up else ('off', '')


class Stack:
    """Starts and stops a profile's components; remembers when each one started."""

    def __init__(self, items, processes, clock, selection=None):
        self.items = {item.key: item for item in items}
        self.processes = processes
        self.clock = clock
        self.started = {}
        self.selection = selection

    def state(self, key, nodes):
        return state(self.items[key], nodes, self.processes, self.clock(), self.started.get(key))

    def ghosts(self):
        """Node names of components we stopped within GHOST_S and did not restart: they may linger."""
        now = self.clock()
        return {node for item in self.items.values() if stopped_recently(item, self.processes, now)
                for node in item.nodes}

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
        if self.selection is not None:
            self.selection.validate()
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
