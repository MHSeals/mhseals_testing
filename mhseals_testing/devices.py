"""The device registry, and the part -> host map derived from the device profiles (device-topology.md).

``devices.yaml`` says which computer has which role and how to reach it. Layers, lowest first:
the package config/devices.yaml, ``$ROS_WS/config/devices.yaml`` (the deployment), then
``~/.config/mhseals/devices.yaml`` (the operator, usually only ``self:``). Addresses come from
``$ROS_WS/config/boat-network.env``, joined on the hostname, so no address is written twice.

What a role runs is never written here: each role names a device profile (mhseals_nav
config/devices/<profile>.yaml), whose ``parts`` are the only place a responsibility is assigned.
``assign`` inverts the active profiles into part -> (profile, role, host) and refuses a part claimed
twice, a part nobody claims, a role without one host, and a host lacking a profile's ``requires``.
``python3 -m mhseals_testing.devices [testing profile]`` prints the map (with that profile's
``placement``). ROS-free.
"""

from dataclasses import dataclass
import os
from pathlib import Path
import re
import shlex
import sys

from mhseals_testing import data
from mhseals_testing.strict_yaml import load_strict

TOP = {'self', 'defaults', 'roles', 'hosts', 'ping'}
DEFAULT_KEYS = {'user', 'container', 'workspace', 'ssh'}
HOST_KEYS = {'roles', 'capabilities', 'address', 'via'} | DEFAULT_KEYS
CAPABILITIES = {'thrusters', 'mavros', 'gpu', 'zed', 'lidar'}
WORD = re.compile(r'[A-Za-z0-9._-]+')   # host names, users, addresses, containers
FALLBACK = 'local'           # the package host: a role lands here only while no other host claims it


class DeviceError(ValueError):
    """A registry or assignment the harness refuses; the message names the file or both sides."""


@dataclass(frozen=True)
class Host:
    name: str
    roles: tuple = ()
    capabilities: frozenset = frozenset()
    address: str = ''
    source: str = ''          # literal, env, lease or mdns: where the address came from
    user: str = ''
    via: str | None = None    # ssh ProxyJump host key
    container: str = ''
    workspace: str = ''
    ssh: bool = True


@dataclass(frozen=True)
class Registry:
    hosts: dict               # name -> Host
    roles: dict               # role -> device profile name
    ping: tuple = ()          # roles the Network row pings
    self_name: str | None = None

    def hosts_for(self, role):
        """Hosts claiming ``role``; the fallback host only when no other does."""
        claimed = [host for host in self.hosts.values() if role in host.roles]
        real = [host for host in claimed if host.name != FALLBACK]
        return real or claimed

    def host_for(self, role):
        found = self.hosts_for(role)
        if len(found) != 1:
            names = ', '.join(host.name for host in found) or 'none'
            raise DeviceError(f'role {role!r} needs exactly one host (has {names})')
        return found[0]


@dataclass(frozen=True)
class Part:
    name: str
    profile: str
    role: str | None          # None under a profile's placement
    host: Host
    spec: dict                # the merged device-profile entry: launch|run, args, nodes, kind, enabled
    defaults: dict = None     # the device profile's top-level args, under the part's own

    @property
    def arguments(self):
        """The include's arguments as device.launch.py passes them: profile args, then the part's."""
        merged = {**(self.defaults or {}), **self.spec.get('args', {})}
        return {key: str(value).lower() if isinstance(value, bool) else str(value)
                for key, value in merged.items()}

    @property
    def kind(self):
        return self.spec.get('kind', 'sensing')

    @property
    def enabled(self):
        return self.spec.get('enabled', True)


# --- boat-network.env --------------------------------------------------------------------------

def env_values(path):
    """``{KEY: value}`` from a shell env file; {} if unreadable. A value that does not parse is left out."""
    try:
        text = Path(path).read_text()
    except OSError:
        return {}
    values = {}
    for line in text.splitlines():
        name, equals, value = line.partition('=')
        name = name.strip()
        if not equals or not name.isidentifier():
            continue
        try:
            words = shlex.split(value, comments=True)
        except ValueError:       # an unclosed quote: no value, never a crash
            continue
        values[name] = words[0] if words else ''
    return values


def leases(values):
    """``{hostname: address}`` from STATIC_LEASES (``MAC=hostname=address ...``)."""
    found = (lease.split('=') for lease in values.get('STATIC_LEASES', '').split())
    return {lease[1]: lease[2] for lease in found if len(lease) == 3}


# --- layers ------------------------------------------------------------------------------------

def _config_dir():
    """config/ in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parent.parent / 'config'
    if (source / 'devices.yaml').is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config'
    except (ImportError, KeyError):
        return source


def layer_paths():
    """(package, deployment, operator, env) default paths."""
    config = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    workspace = data.workspace() / 'config'
    return (_config_dir() / 'devices.yaml', workspace / 'devices.yaml',
            config / 'mhseals' / 'devices.yaml', workspace / 'boat-network.env')


def _strings(value, where):
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise DeviceError(f'{where}: list of names')
    return value


def _word(value, where):
    """Refuse anything but a plain name: these reach ssh's argv and the remote shell."""
    if not isinstance(value, str) or not WORD.fullmatch(value) or value.startswith('-'):
        raise DeviceError(f'{where}: {value!r} is not a plain name ([A-Za-z0-9._-], no leading -)')
    return value


def validate_layer(layer, where):
    """Refuse unknown keys and wrong types in one devices.yaml layer."""
    if layer is None:
        return {}
    if not isinstance(layer, dict) or not set(layer) <= TOP:
        raise DeviceError(f'{where}: mapping with keys from {sorted(TOP)}')
    if 'self' in layer and not isinstance(layer['self'], str):
        raise DeviceError(f'{where}.self: a host name')
    defaults = layer.get('defaults', {})
    if not isinstance(defaults, dict) or not set(defaults) <= DEFAULT_KEYS:
        raise DeviceError(f'{where}.defaults: keys from {sorted(DEFAULT_KEYS)}')
    for key in ('user', 'container'):
        if defaults.get(key):
            _word(defaults[key], f'{where}.defaults.{key}')
    for role, spec in (layer.get('roles') or {}).items():
        if spec is not None and (not isinstance(spec, dict) or set(spec) != {'profile'}
                                 or not isinstance(spec['profile'], str)):
            raise DeviceError(f'{where}.roles.{role}: {{profile: <device profile>}} or null')
    for name, spec in (layer.get('hosts') or {}).items():
        here = f'{where}.hosts.{name}'
        _word(name, here)
        if spec is None:
            continue
        if not isinstance(spec, dict) or not set(spec) <= HOST_KEYS:
            raise DeviceError(f'{here}: mapping with keys from {sorted(HOST_KEYS)}')
        _strings(spec.get('roles', []), f'{here}.roles')
        unknown = set(_strings(spec.get('capabilities', []), f'{here}.capabilities')) - CAPABILITIES
        if unknown:
            raise DeviceError(f'{here}.capabilities: unknown {sorted(unknown)}; known {sorted(CAPABILITIES)}')
        for key in ('address', 'user', 'via', 'container', 'workspace'):
            if spec.get(key) is not None and not isinstance(spec[key], str):
                raise DeviceError(f'{here}.{key}: string')
        for key in ('user', 'via', 'container'):
            if spec.get(key):
                _word(spec[key], f'{here}.{key}')
        if spec.get('address'):
            _word(spec['address'].removeprefix('$'), f'{here}.address')
        if 'ssh' in spec and not isinstance(spec['ssh'], bool):
            raise DeviceError(f'{here}.ssh: true or false')
    _strings(layer.get('ping', []), f'{where}.ping')
    return layer


def merge(base, layer):
    """Overlay ``layer`` on ``base``: hosts and roles per key (null deletes), host fields one by one."""
    out = {'self': layer.get('self', base.get('self')),
           'defaults': {**base.get('defaults', {}), **layer.get('defaults', {})},
           'roles': dict(base.get('roles', {})), 'hosts': dict(base.get('hosts', {})),
           'ping': layer.get('ping', base.get('ping', []))}
    for key in ('roles', 'hosts'):
        for name, spec in (layer.get(key) or {}).items():
            if spec is None:
                out[key].pop(name, None)
            elif key == 'hosts':
                out[key][name] = {**out[key].get(name, {}), **spec}
            else:
                out[key][name] = spec
    return out


def _address(name, spec, env, reserved, where):
    value = spec.get('address')
    if value and value.startswith('$'):
        if not env.get(value[1:]):
            raise DeviceError(f'{where}.address: {value} is not set in boat-network.env')
        return env[value[1:]], 'env'
    if value:
        return value, 'literal'
    if name in reserved:
        return reserved[name], 'lease'
    return f'{name}.local', 'mdns'


def build(merged, env):
    """A Registry from merged layers and the env file's values."""
    defaults = {'user': '', 'container': '', 'workspace': '', 'ssh': True, **merged['defaults']}
    reserved = leases(env)
    hosts = {}
    for name, spec in merged['hosts'].items():
        address, source = _address(name, spec, env, reserved, f'hosts.{name}')
        _word(address, f'hosts.{name}.address ({source})')    # env and lease values too
        settings = {**defaults, **{k: v for k, v in spec.items() if k in DEFAULT_KEYS}}
        hosts[name] = Host(name, tuple(spec.get('roles', ())), frozenset(spec.get('capabilities', ())),
                           address, source, settings['user'], spec.get('via'), settings['container'],
                           settings['workspace'], settings['ssh'])
    for name, host in hosts.items():
        if host.via is not None and host.via not in hosts:
            raise DeviceError(f'hosts.{name}.via: unknown host {host.via!r}')
    if merged['self'] is not None and merged['self'] not in hosts:
        raise DeviceError(f'self: unknown host {merged["self"]!r}')
    roles = {role: spec['profile'] for role, spec in merged['roles'].items()}
    return Registry(hosts, roles, tuple(merged['ping']), merged['self'])


def load(package=None, deployment=None, operator=None, env=None):
    """The merged Registry. Paths default to ``layer_paths()``; missing deployment/operator files are skipped."""
    defaults = layer_paths()
    package, deployment, operator, env = (given if given is not None else default for given, default
                                          in zip((package, deployment, operator, env), defaults))
    merged = {}
    for label, path in (('package', package), ('deployment', deployment), ('operator', operator)):
        path = Path(path).expanduser()
        if not path.is_file():
            if label == 'package':
                raise DeviceError(f'package devices.yaml missing: {path}')
            continue
        try:
            layer = load_strict(path.read_text(), path)
        except ValueError as error:
            raise DeviceError(str(error)) from None
        merged = merge(merged, validate_layer(layer, str(path)))
    return build(merged, env_values(env))


# --- who am I, and the derived map ------------------------------------------------------------

def local_addresses():
    """IPv4 addresses this machine holds (via interfaces), plus the DDS pin from ros_net."""
    import socket
    from mhseals_testing.health.netstat import interface_addresses
    try:
        found = set(interface_addresses([name for _, name in socket.if_nameindex()]).values())
    except OSError:
        found = set()
    pinned = os.environ.get('ASTRO_DDS_ADDRESS')
    return found | ({pinned} if pinned else set())


def whoami(registry, addresses=()):
    """This machine's host: the operator's ``self:``, else the host holding one of ``addresses``, else the fallback.

    gethostname() is not used: inside the container it names the container, not the machine.
    """
    if registry.self_name:
        return registry.hosts[registry.self_name]
    for host in registry.hosts.values():
        if host.name != FALLBACK and host.address in addresses:
            return host
    return registry.hosts.get(FALLBACK) or Host(FALLBACK, address='127.0.0.1', source='literal')


def load_device_profile(name):
    """The merged mhseals_nav device profile ``name`` (its ``parts``, ``requires``, ``args``)."""
    try:
        from mhseals_nav.device_profile import load as load_profile
    except ImportError:
        raise DeviceError('mhseals_nav device profiles are not installed (device.launch.py)') from None
    return load_profile(name)


def assign(registry, me, profiles=load_device_profile, placement=None, wanted=()):
    """``{part: Part}`` from the active device profiles; refuses ambiguous or unclaimed parts.

    Active profiles are the registry's roles, each on its role's host; a profile's ``placement``
    ``{self: [names]}`` replaces them with ``names``, all on ``me``. ``wanted`` are the parts the
    session uses: each must be claimed by exactly one active profile.
    """
    if placement is not None:
        if not isinstance(placement, dict) or set(placement) != {'self'}:
            raise DeviceError('placement: {self: [device profiles]}')
        active = [(name, None, me) for name in _strings(placement['self'], 'placement.self')]
    else:
        active = [(name, role, registry.host_for(role)) for role, name in registry.roles.items()]
    parts = {}
    for name, role, host in active:
        profile = profiles(name)
        missing = set(profile.get('requires', ())) - host.capabilities
        if missing:
            raise DeviceError(f'device profile {name} requires {sorted(missing)}; '
                              f'host {host.name} has {sorted(host.capabilities) or "none"}')
        for part, spec in (profile.get('parts') or {}).items():
            if part in parts:
                raise DeviceError(f'part {part!r} is claimed by both {parts[part].profile} '
                                  f'and {name}; give it to one device profile')
            parts[part] = Part(part, name, role, host, spec, dict(profile.get('args') or {}))
    unclaimed = [part for part in wanted if part not in parts]
    if unclaimed:
        raise DeviceError(f'no active device profile runs {", ".join(unclaimed)} '
                          f'(active: {", ".join(name for name, _, _ in active)})')
    return parts


def resolve(placement=None, wanted=()):
    """(this machine's Host, ``{part: Part}``) from the layered registry; DeviceError when refused."""
    registry = load()
    me = whoami(registry, local_addresses())
    return me, assign(registry, me, placement=placement, wanted=wanted)


def reach(host, me):
    """How the TUI reaches ``host``: 'local', or the ssh + docker route it would take."""
    if host.name == me.name:
        return 'local'
    if not host.ssh:
        return 'no ssh'
    jump = f'-J {host.via} ' if host.via else ''
    user = f'{host.user}@' if host.user else ''
    return f'ssh {jump}{user}{host.address} docker {host.container}'


def ping_targets(registry, me):
    """``{host name: address}`` for the hosts of the ``ping`` roles, never this machine."""
    return {host.name: host.address for role in registry.ping for host in registry.hosts_for(role)
            if host.name != me.name}


def table(parts, me):
    """Rows (part, profile, host, reach) for printing, in profile order."""
    return [(part.name + ('' if part.enabled else ' (disabled)'), part.profile, part.host.name,
             reach(part.host, me)) for part in parts.values()]


def main(argv=None):
    """Print the map; a testing profile name (``boat``, ``crane_full_stack``) applies its placement."""
    argv = sys.argv[1:] if argv is None else argv
    try:
        placement = None
        if argv:
            from mhseals_testing.profiles import load_profile
            placement = load_profile(argv[0]).placement
        me, parts = resolve(placement)
        rows = table(parts, me)
    except (DeviceError, FileNotFoundError, ValueError) as error:
        print(f'devices: {error}', file=sys.stderr)
        return 1
    rows.insert(0, ('part', 'profile', 'host', 'reach'))
    widths = [max(len(row[column]) for row in rows) for column in range(3)]
    print(f'this machine: {me.name}')
    for row in rows:
        print('  '.join(cell.ljust(width) for cell, width in zip(row, widths)) + '  ' + row[3])
    return 0


if __name__ == '__main__':
    sys.exit(main())
