"""Host resolution: this machine, the notes' target, and every name a boat computer goes by (``boats``,
``is_boat``). Stdlib only."""

import os
from pathlib import Path
import re

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


def boats(config):
    """Every name a boat computer goes by: ``hook.hosts`` and their vessel names (``blastoise-odroid`` ->
    ``blastoise``, the former ssh aliases: blocking more is the safe side), a boat ``$BOAT_AGENT_TARGET``, the hostnames
    and addresses of STATIC_LEASES in the vessel's network env (``network_file``; a checkout without it, such as CI,
    knows no boat addresses, so ``is_boat`` then treats any private-LAN address as a boat), and ~/.ssh/config
    aliases for any of them."""
    names = set(config['hosts']) | {host.split('-')[0] for host in config['hosts']} | {target_of(config)} - {'local', SIM}
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


def is_boat(target, names):
    """The boat name an ssh destination (``user@host``, ``host:path``) reaches, else None; without a network env
    (``network_file``) any private-LAN address is a boat (fail closed). ``$H`` names nothing the hook can read: a
    boat."""
    host = target.rsplit('@', 1)[-1].split(':', 1)[0]
    if '$' in host or host in names or host.split('.')[0] in names:
        return host
    return host if network_file() is None and PRIVATE_LAN.fullmatch(host) else None
