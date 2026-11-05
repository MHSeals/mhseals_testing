"""Host resolution: this machine, the notes' target, and every name a boat computer goes by (``boats``,
``is_boat``): the device registry's boat compute hosts (``registry.fleet``), never a name written in code. Stdlib only."""

import os
from pathlib import Path
import re

from .registry import fleet
from .rules import DEFAULTS

NETWORK = 'config/boat-network.env'    # legacy path; STATIC_LEASES "MAC=hostname=address ...": the boat computers


def target_of(config):
    """The host whose TUI gets the notes: ``$BOAT_AGENT_TARGET``, else ``hook.target``; 'local' (or
    empty) is this machine."""
    return (os.environ.get('BOAT_AGENT_TARGET') or config.get('target') or 'local').strip() or 'local'


SIM = 'sim'          # $BOAT_AGENT_TARGET=sim: tools/boat_tui sim's TUI on this machine (its container), never a host


def remote(config):
    """The target host when it is another machine, else None (the sim runs here)."""
    target = target_of(config)
    return None if target in ('local', SIM) or hostname().split('.')[0] == target else target


def hostname():
    return os.uname().nodename


def network_file():
    """The vessel's network env: ``$ASTRO_NETWORK_ENV`` (None when set but missing, so ``denied``
    fails closed), else the legacy config/boat-network.env of this workspace (``src/<repo>/<package>/``
    up three), of ``$ROS_WS`` or of ``~/astro_dock``."""
    explicit = os.environ.get('ASTRO_NETWORK_ENV')
    if explicit:
        path = Path(explicit).expanduser()
        return path if path.is_file() else None
    ws = os.environ.get('ROS_WS')
    for root in (Path(__file__).resolve().parents[4], ws if ws and os.path.isabs(ws) else None,
                 Path.home() / 'astro_dock'):
        if root and (Path(root) / NETWORK).is_file():
            return Path(root) / NETWORK
    return None


class Names(set):
    """``boats``: every boat name, with the registry's ``hosts`` (boat compute hosts, role order), ``vessels``
    (the profile set's and each host's prefix: a vessel's name reaches its hosts), ``read`` and ``allow``
    (``hook.ssh_allow``: the names an unread registry still lets ssh reach)."""


def boats(config):
    """Every name a boat computer goes by: the registry's boat compute hosts (``hook.boat_roles``) and their vessel
    names (a host's prefix, ``VESSEL-...``: the former ssh aliases; blocking more is the safe side), a boat
    ``$BOAT_AGENT_TARGET``, the hostnames and addresses of STATIC_LEASES in the vessel's network env (``network_file``;
    a checkout without it, such as CI, knows no boat addresses, so ``is_boat`` then treats any private-LAN address as
    a boat), and ~/.ssh/config aliases for any of them."""
    found = fleet(config.get('boat_roles') or DEFAULTS['boat_roles'])
    vessels = set(found.vessels) | {host.split('-')[0] for host in found.hosts}
    names = Names(set(found.hosts) | vessels | {target_of(config)} - {'local', SIM})
    names.hosts, names.vessels, names.read = found.hosts, vessels, found.read
    names.allow = set(config.get('ssh_allow', DEFAULTS['ssh_allow']))
    try:
        leases = re.search(r'^STATIC_LEASES="?([^"\n]*)', network_file().read_text(), re.M).group(1)
        names |= {part for lease in leases.split() for part in lease.split('=')[1:]}
    except (AttributeError, OSError):
        pass
    try:
        aliases = []
        for line in (Path.home() / '.ssh' / 'config').read_text().splitlines():
            words = line.split()
            if words[:1] and words[0].lower() == 'host':
                aliases = words[1:]
            elif words[:1] and words[0].lower() == 'hostname' and len(words) > 1 and (
                    words[1] in names or words[1].split('.')[0] in names):
                names |= {a for a in aliases if not set('*?!') & set(a)}
    except OSError:
        pass
    return names


PRIVATE_LAN = re.compile(r'(?:192\.168|10|172\.(?:1[6-9]|2\d|3[01]))(?:\.\d{1,3}){2,3}')


def known_boat(host, names):
    """``host`` is a boat by name: one of ``names``, or ``VESSEL-...`` of a vessel (a renamed or added host too)."""
    short = host.split('.')[0]
    return host in names or short in names or any(short.startswith(v + '-') for v in getattr(names, 'vessels', ()))


def is_boat(target, names):
    """The boat name an ssh destination (``user@host``, ``host:path``) reaches, else None; without a network env
    (``network_file``) any private-LAN address is a boat, and with an unread registry any name but ``hook.ssh_allow``
    (fail closed). ``$H`` names nothing the hook can read: a boat."""
    host = target.rsplit('@', 1)[-1].split(':', 1)[0]
    if '$' in host or known_boat(host, names):
        return host
    if not getattr(names, 'read', False) and host not in getattr(names, 'allow', ()):
        return host
    return host if network_file() is None and PRIVATE_LAN.fullmatch(host) else None


DEVICE_TREE = Path('/proc/device-tree/model')    # a board computer (Jetson, ODROID); a laptop or station has none


def this_boat(names):
    """This machine's name when it is a boat computer: a boat name, or, with an unread registry, any board computer
    (``DEVICE_TREE``: fail closed), else None."""
    here = hostname().split('.')[0]
    return here if known_boat(here, names) or (not getattr(names, 'read', False) and DEVICE_TREE.exists()) else None


def note_match(config, names=None):
    """``hook.match`` and, first, ``ssh`` to any boat name (every ssh while the registry is unread)."""
    names = boats(config) if names is None else names
    if not getattr(names, 'read', False):
        ssh = r'\bssh\b'
    elif names:
        ssh = r'\bssh\s+(-\S+\s+)*(' + '|'.join(re.escape(n) for n in sorted(names, key=lambda n: (-len(n), n))) + r')\b'
    else:
        ssh = r'(?!)'
    return [ssh] + list(config['match'])
