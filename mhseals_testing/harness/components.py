"""Stack components (odometry, sensors, navigation, ...) a profile can start and stop.

A profile's ``components:`` mapping keeps only TUI behaviour (title, needs, start, limits, topic):
each entry is one device part (``part:``, default the key) of mhseals_nav's device profiles, and
the registry (mhseals_testing.devices) says which computer runs it. The launch file, its args,
the proof nodes and the host all come from there; the session's own args (the odometry/endpoint
unification, the Nav2 limits file, the command line) travel as ``overrides`` to
``ros2 launch mhseals_nav device.launch.py device:=<profile> only:=<part>``.

A part on this computer is a keeper child (ours to start and stop); a part on another computer is
a named unit there (harness/remote.py), which any TUI may start and stop. ``state`` classifies a
component from the node graph, our own children and the units; a process neither we nor a unit
started is ``external`` and is never stopped.
"""

from dataclasses import dataclass, field
import json

from mhseals_testing import nav2_limits

STARTING_LIMIT = 20.0   # s before a started component without its proof nodes warns
KEYS = {'title', 'part', 'needs', 'limits', 'topic', 'start'}


@dataclass(frozen=True)
class Component:
    key: str
    title: str
    argv: tuple = ()          # ros2 launch mhseals_nav device.launch.py device:= only:= [overrides:=]
    nodes: tuple = ()         # nodes whose presence proves it is running
    needs: tuple = ()         # component keys that must be up first
    host: object = None       # devices.Host of the computer that runs it; None for this one
    topic: str | None = None
    limits: bool = False      # takes the session's Nav2 limits file
    start: bool = False       # started with the session unless a stack is already up
    part: str = ''
    profile: str = ''         # the device profile that claims the part
    kind: str = 'sensing'     # or 'commander' (Nav2, the mission runner)
    enabled: bool = True
    launch: str | None = None   # the part's launch file in mhseals_nav (None for a run part)
    arguments: dict = field(default_factory=dict, compare=False, hash=False)  # the include's, final

    @property
    def remote(self):
        """The host name when another computer runs it, else None."""
        return self.host.name if self.host is not None else None

    @property
    def unit(self):
        return f'mhseals-{self.part}'


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
        if spec is None:
            data[key] = spec = {}
        if not isinstance(spec, dict) or not set(spec) <= KEYS:
            raise ValueError(f'{here}: mapping with keys from {sorted(KEYS)} '
                             '(launch, args, nodes and host come from the device profile)')
        if not isinstance(spec.get('part', key), str):
            raise ValueError(f'{here}.part: a device part name')
        if not isinstance(spec.get('needs', []), list):
            raise ValueError(f'{here}.needs: list')
        if spec.get('start', False) not in (True, False, 'here'):
            raise ValueError(f'{here}.start: true, false or here (only when this computer runs it)')
        for need in spec.get('needs', []):
            if need not in data or need == key:
                raise ValueError(f'{here}.needs: unknown component {need!r}')
    parts = [spec.get('part', key) for key, spec in data.items()]
    if len(set(parts)) != len(parts):
        raise ValueError(f'{where}: two components name the same part')
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


ODOMETRY_LAUNCHES = {'odom.launch.py', 'ekf.launch.py', 'robot.launch.py'}
ODOMETRY_INPUTS = frozenset({
    'odometry_startup_file', 'odometry_deployment_file', 'ekf_startup_context',
    'robot_state_publish_frequency', 'mavros_velocity_source', 'mavros_velocity_stddev',
    'mavros_velocity_max_age', 'mavros_velocity_min_stddev', 'mavros_velocity_no_fix_stddev',
    'ekf_delay', 'navsat_transform_config_file', 'ekf_overlay_file',
    'ekf_local_config_file', 'ekf_global_config_file',
})


def _text(value):
    """A launch argument as device.launch.py passes it (booleans lower case)."""
    return str(value).lower() if isinstance(value, bool) else str(value)


def validate_localization_profile(profile, launches, arguments):
    """Refuse unsupported managed localization graphs and contradictory clocks."""
    managed = [key for key, launch in launches.items() if launch in ODOMETRY_LAUNCHES]
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
        supplied = _text(arguments[key].get('sim', clock))
        if supplied not in ('true', 'false') or supplied != clock:
            raise ValueError(f'managed localization {key}.sim must match use_sim_time')
        arguments[key]['sim'] = clock


def odometry_arguments(launches, arguments):
    """One explicit odometry selection for both parent and separately managed EKFs."""
    selected = {}
    for key, launch in launches.items():
        if launch not in ODOMETRY_LAUNCHES:
            continue
        for name, value in arguments[key].items():
            if name not in ODOMETRY_INPUTS or value == '':
                continue
            if name in selected and str(selected[name]) != str(value):
                raise ValueError(f'conflicting odometry selection: {name}')
            selected[name] = value
    for key, launch in launches.items():
        if launch in ODOMETRY_LAUNCHES:
            arguments[key].update(selected)


ENDPOINT_LAUNCHES = {'odom.launch.py', 'robot.launch.py', 'sensors.launch.py'}
ENDPOINT_INPUTS = {'endpoint_deployment_file', 'endpoint_startup_file', 'endpoint_context',
                   'fcu_url', 'gcs_url', 'ros_ip', 'ros_port'}


def endpoint_arguments(launches, arguments):
    """Share one pure endpoint selection before constructing managed children."""
    managed = [key for key, launch in launches.items() if launch in ENDPOINT_LAUNCHES]
    if not managed:
        return
    from mhseals_nav.endpoint_configuration import resolve_endpoints
    selected = {}
    for key in managed:
        for name, value in arguments[key].items():
            if name not in ENDPOINT_INPUTS:
                continue
            if value == ('__inherit__' if name == 'gcs_url' else ''):
                continue
            if name in selected and str(selected[name]) != str(value):
                raise ValueError(f'conflicting endpoint selection: {name}')
            selected[name] = value
    selected.setdefault('endpoint_context', 'tui')
    resolve_endpoints(deployment_file=selected.get('endpoint_deployment_file', ''),
                      operator_file=selected.get('endpoint_startup_file', ''),
                      context=selected['endpoint_context'],
                      aliases={key: value for key, value in selected.items()
                               if key in {'fcu_url', 'gcs_url', 'ros_ip', 'ros_port'}})
    for key in managed:
        arguments[key].update(selected)


def device_argv(part, session):
    """``ros2 launch`` of one part through device.launch.py; ``session`` args go in ``overrides``."""
    argv = ('ros2', 'launch', 'mhseals_nav', 'device.launch.py',
            f'device:={part.profile}', f'only:={part.name}')
    if session:
        layer = {'parts': {part.name: {'args': session}}}
        argv += (f'overrides:={json.dumps(layer, separators=(",", ":"), sort_keys=True)}',)
    return argv


def components(profile, overlay=None, placed=None):
    """The profile's components in file order. ``overlay`` is {key: {arg: value}} (the session).

    ``placed`` is (this machine's Host, {part: devices.Part}); by default the profile's own.
    """
    specs = profile.components or {}
    if not specs:
        return ()
    me, parts = placed if placed is not None else profile.placed
    overlay = overlay or {}
    resolved = {key: parts[spec.get('part', key)] for key, spec in specs.items()}
    base = {key: part.arguments for key, part in resolved.items()}
    arguments = {key: {**base[key], **overlay.get(key, {})} for key in specs}
    launches = {key: part.spec.get('launch') for key, part in resolved.items()}
    odometry_arguments(launches, arguments)
    validate_localization_profile(profile, launches, arguments)
    endpoint_arguments(launches, arguments)
    result = []
    for key, spec in specs.items():
        part = resolved[key]
        kind = part.spec.get('kind', 'sensing')
        here = part.host.name == me.name
        # start: here comes up with the session only on the computer that runs the part (the
        # operator heartbeat, from an operator's TUI); from another computer it stays a Status row.
        start = spec.get('start') is True or (spec.get('start') == 'here' and here)
        if spec.get('start') and kind == 'commander' and profile.use_sim_time is not True:
            raise ValueError(f'components.{key}.start: {part.name} can command motion; only a '
                             'simulator profile (use_sim_time) starts it with the session')
        if spec.get('limits'):   # the lasting nav2.yaml must survive every (re)start
            arguments[key]['limits_file'] = str(nav2_limits.path())
        final = {name: _text(value) for name, value in arguments[key].items()}
        session = {name: value for name, value in final.items() if base[key].get(name) != value}
        result.append(Component(
            key, spec.get('title', key.capitalize()), device_argv(part, session),
            tuple(part.spec.get('nodes', ())), tuple(spec.get('needs', ())),
            None if here else part.host, spec.get('topic'),
            bool(spec.get('limits', False)), start, part.name,
            part.profile, kind, part.enabled, launches[key], final))
    return tuple(result)


def moves(component):
    """True for a component that can command the boat: Nav2 and the mission runner."""
    return component.limits or component.kind == 'commander' or component.launch == 'navigation.launch.py'


GHOST_S = 25.0   # robot_localization exits without announcing itself; its names wait for the DDS lease (~20 s)


def stopped_recently(component, processes, now):
    """True while the names of a child we just stopped may still be on the graph."""
    at = getattr(processes, 'stopped_at', {}).get(component.key)
    return at is not None and now - at < GHOST_S


def _up(component, nodes):
    """Every proof node of ``component`` is in ``nodes`` (bare names)."""
    return bool(component.nodes) and all(node in nodes for node in component.nodes)


def remote_state(component, nodes, units, now, started):
    """(state, detail) of a part another computer runs, from the graph and its unit's status.

    The graph proves up; the unit says why not. Unreachable (ssh failed) is not off. Nodes in the
    graph read as external only once the unit is known not to run them: while ssh is down or
    before the first status call it may be ours, so the row stays stoppable.
    """
    up = _up(component, nodes)
    record, error = units.status(component) if units is not None else (None, None)
    where = component.remote
    if record is not None and record.get('state') == 'running':
        if up or not component.nodes:
            return 'ours', f'unit on {where}'
        if started is not None and now - started > STARTING_LIMIT:
            missing = next(node for node in component.nodes if node not in nodes)
            return 'starting', f'WARN: no {missing} (unit on {where})'
        return 'starting', f'unit on {where}'
    if error:
        return 'unreachable', error
    if units is not None and record is None:
        return 'unknown', f'unit status on {where} not read yet'
    if up:
        return 'external', where
    if record is not None and record.get('state') == 'exited':
        return 'exited', f'unit exited {record.get("code")} on {where}: {record.get("log")}'
    return 'off', ''


def state(component, nodes, processes, now, started=None, units=None):
    """(state, detail): off, starting, ours, external, exited, unreachable, unknown or disabled.

    ``processes`` is the ProcessManager, ``nodes`` the node names in the graph (namespaces are ignored),
    ``started`` the time we started the component (same clock as ``now``), ``units`` the remote
    units (harness/remote.Units) for a part on another computer.
    """
    if not component.enabled:
        return 'disabled', f'disabled on {component.profile}'
    nodes = {node.rsplit('/', 1)[-1] for node in nodes}
    if component.host is not None:
        return remote_state(component, nodes, units, now, started)
    up = bool(component.nodes) and all(node in nodes for node in component.nodes)
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
    """Starts and stops a profile's components; remembers when each one started.

    ``units`` (harness/remote.Units) starts and stops the parts other computers run; without it
    those parts cannot be started from here.
    """

    def __init__(self, items, processes, clock, units=None):
        self.items = {item.key: item for item in items}
        self.processes = processes
        self.clock = clock
        self.units = units
        self.started = {}

    def state(self, key, nodes):
        return state(self.items[key], nodes, self.processes, self.clock(), self.started.get(key),
                     self.units)

    def ghosts(self):
        """Node names of components we stopped within GHOST_S and did not restart: they may linger."""
        now = self.clock()
        return {node for item in self.items.values() if stopped_recently(item, self.processes, now)
                for node in item.nodes}

    def start(self, key, nodes, commander=False):
        """Start an off component; refuses (naming them) while its needs are off.

        A commander part (Nav2, the mission runner) on another computer starts only with
        ``commander``, which the TUI passes after the operator's keypress and confirm.
        """
        item = self.items[key]
        status, detail = self.state(key, nodes)
        if status == 'disabled':
            raise ComponentError(f'{item.title} is {detail}', [key])
        if status == 'unreachable':
            raise ComponentError(f'{item.title}: {item.remote} is unreachable ({detail})', [key])
        # Not read yet: a start adopts a running unit, but nodes already up may be someone else's.
        if status == 'unknown' and _up(item, {node.rsplit('/', 1)[-1] for node in nodes}):
            raise ComponentError(f'{item.title} is up and its {detail}; try again shortly', [key])
        if status in ('ours', 'starting', 'external'):
            raise ComponentError(f'{item.title} is already running', [key])
        off = [need for need in item.needs
               if self.state(need, nodes)[0] in ('off', 'exited', 'unreachable', 'disabled')]
        if off:
            raise ComponentError(f'Start {self.items[off[0]].title} first', off)
        if item.host is None:
            self.processes.start(key, item.argv)
        elif self.units is None:
            raise ComponentError(f'{item.title} runs on {item.remote}; start it there', [key])
        elif item.kind == 'commander' and not commander:
            raise ComponentError(f'{item.title} can command motion: start it on {item.remote} '
                                 'from its Status row, with a confirm', [key])
        else:
            self.units.start(item, commander=item.kind == 'commander')
        self.started[key] = self.clock()

    def running(self, key):
        """Ours to stop: a keeper child here, or a unit not known to be stopped (stop always tries)."""
        item = self.items[key]
        if item.host is None:
            return self.processes.running(key)
        if self.units is None:
            return False
        record, _ = self.units.status(item)
        return record is None or record.get('state') == 'running'

    def stop(self, key, nodes):
        """Stop ours, dependants first; returns the keys stopped.

        Never stops an external local process. A remote part's stop always goes out: ``unit
        stop`` of a unit that is not running is harmless, and ssh may be down or not yet read.
        """
        item = self.items[key]
        status, _ = self.state(key, nodes)
        if status == 'external' and item.host is None:
            raise ComponentError('running outside this session; stop it where it was started', [key])
        if item.host is not None and self.units is None:
            raise ComponentError(f'{item.title} runs on {item.remote}; stop it there', [key])
        order = [other for other in self._dependants(key) if self.running(other)]
        if self.running(key) or item.host is not None:
            order.append(key)
        for name in order:
            if self.items[name].host is None:
                self.processes.stop(name)
            else:
                self.units.stop(self.items[name])
            self.started.pop(name, None)
        return order

    def _dependants(self, key):
        """Components that need ``key``, directly or not, deepest first."""
        found = []
        for item in self.items.values():
            if key in item.needs:
                found += [*self._dependants(item.key), item.key]
        return list(dict.fromkeys(found))

    def autostart(self, nodes, publishers, sim):
        """Start the profile's ``start`` components with the session; returns lines to report.

        A component already running (ours, a unit's or another process's) is attached to. Off the
        simulator nothing that commands motion auto-starts, needs pulled in included. A failed
        start skips the components that need it. Off the simulator a part another computer runs
        is never auto-started: slow discovery is not proof it is off. ``publishers`` are the
        odometry topic's nodes: one no component claims (a bag play, another stack) starts nothing.
        """
        order = self.start_order()
        known = {node for item in self.items.values() for node in item.nodes}
        foreign = sorted({node for node in publishers if node.rsplit('/', 1)[-1] not in known})
        if order and foreign:
            return [f'odometry already published by {", ".join(foreign)}: attaching, '
                    'starting nothing (the Status rows start components)']
        lines, failed = [], set()
        # Without proof nodes a component's state is unknowable: beside a stack that is
        # already up it may be running too, so it is left to its Status row.
        up = any(self.state(key, nodes)[0] == 'external' for key in order)
        for key in order:
            item = self.items[key]
            if not sim and moves(item):
                failed.add(key)
                lines.append(f'refused to auto-start {item.title}: it can command motion '
                             '(start it from its Status row)')
                continue
            need = next((need for need in item.needs if need in failed), None)
            if need is not None:
                failed.add(key)
                lines.append(f'skipped {item.title}: {self.items[need].title} did not start')
                continue
            status = self.state(key, nodes)[0]
            if status in ('ours', 'starting', 'external'):
                lines.append(f'attached to {item.title} ({status})')
                continue
            # A part another computer runs may be up there and just not discovered yet (over the
            # radio this outlasts any wait): a second copy (MAVROS on the FCU link) kills both.
            if not sim and item.host is not None:
                failed.add(key)
                lines.append(f'left {item.title} to {item.remote}: discovery cannot prove it is off '
                             'there (start it from its Status row)')
                continue
            if up and not item.nodes:
                lines.append(f'left {item.title} off: it names no nodes to tell whether it '
                             'already runs beside this stack (start it from its Status row)')
                continue
            try:
                self.start(key, nodes, commander=sim)
            except Exception as error:  # noqa: BLE001 - one bad component must not stop the TUI
                failed.add(key)
                lines.append(f'could not start {item.title}: {error}')
        return lines

    def start_order(self):
        """Enabled keys marked ``start``, each after its needs."""
        order = []

        def visit(key):
            for need in self.items[key].needs:
                visit(need)
            if key not in order:
                order.append(key)
        for item in self.items.values():
            if item.start:
                visit(item.key)
        return [key for key in order if self.items[key].enabled]
