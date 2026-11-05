"""The device registry as the hook reads it, stdlib only (``.agents/hook.py`` loads the hook under ``python3 -I``, maybe
without PyYAML): the hosts that hold a boat compute role (``hook.boat_roles``) and the profile set's vessels.

Sources, every one found: ``$ASTRO_REGISTRY`` (one vessel's devices.yaml, as ``hw.sh env`` sets it), the profile set
(``$ASTRO_PROFILES``, else the ``ASTRO_PROFILES`` of astro's profiles.env, else ``profiles/`` of this workspace, of
``$ROS_WS`` or of ``~/astro_dock``: a ``kind: vessel`` directory is a vessel and its ``registry:`` file its hosts) and
the legacy ``config/devices.yaml`` of those workspaces. A source that is named but missing, or that this reader cannot
parse, or no registry at all, leaves the fleet unread (``read`` False): the hook then fails closed (``scan.denied``).
The reader knows the registry's YAML subset (block mappings, flow mappings and lists, plain and quoted scalars,
comments) and refuses anything else rather than guess, so a registry it misreads never blocks less."""

import os
from pathlib import Path
import re
from typing import NamedTuple

LEGACY = 'config/devices.yaml'      # a workspace's deployment registry before the profile set
FALLBACK = 'local'                  # devices.py's package host: this machine, never a boat to ssh to
KEY = re.compile(r'''("[^"\\]*"|'[^']*'|[^\s'"#:{}\[\],&*!|>%@`][^:#]*?)\s*:(?:\s+(.*))?$''')


class Fleet(NamedTuple):
    hosts: tuple        # boat compute hosts: each registry's in hook.boat_roles order, then file order
    vessels: tuple      # the profile set's kind: vessel directories
    read: bool          # every source found was read and one at least names hosts; False: fail closed


def uncomment(line):
    """``line`` without its ``#`` comment (a ``#`` at the start or after a blank, outside quotes)."""
    quote = None
    for i, c in enumerate(line):
        if quote:
            quote = None if c == quote else quote
        elif c in '\'"':
            quote = c
        elif c == '#' and (i == 0 or line[i - 1] in ' \t'):
            return line[:i]
    return line


def scalar(text):
    """A plain or quoted scalar; ValueError for an anchor, alias, tag, block scalar or escape."""
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in '\'"' and '\\' not in text:
        return text[1:-1]
    if not text or text[0] in '&*!|>%@`\'"{}[],' or '\\' in text:
        raise ValueError(f'not a plain scalar: {text!r}')
    return text


def flow(text):
    """A flow value (``{roles: [a, b], user: x}``, ``[a, b]`` or a scalar) as dict, list or str."""
    value, end = _flow(text, 0)
    if text[end:].strip():
        raise ValueError(f'trailing text: {text[end:]!r}')
    return value


def _flow(s, i):
    while i < len(s) and s[i] in ' \t':
        i += 1
    if i >= len(s):
        raise ValueError('a value is missing')
    if s[i] not in '{[':
        end = i
        while end < len(s) and s[end] not in ',]}':
            end += 1
        return scalar(s[i:end]), end
    close, out, i = '}' if s[i] == '{' else ']', {} if s[i] == '{' else [], i + 1
    while True:
        while i < len(s) and s[i] in ' \t':
            i += 1
        if i < len(s) and s[i] == close:
            return out, i + 1
        if isinstance(out, dict):
            colon = s.find(':', i)
            if colon < 0:
                raise ValueError('a key without a value')
            key, (value, i) = scalar(s[i:colon]), _flow(s, colon + 1)
            if key in out:
                raise ValueError(f'duplicate key {key!r}')
            out[key] = value
        else:
            value, i = _flow(s, i)
            out.append(value)
        while i < len(s) and s[i] in ' \t':
            i += 1
        if i < len(s) and s[i] == ',':
            i += 1
        elif i >= len(s) or s[i] != close:
            raise ValueError(f'expected , or {close}')


def roles_of(spec):
    """The roles of one host's mapping (none: no roles); ValueError unless a list of names."""
    roles = spec.get('roles', []) if isinstance(spec, dict) else None
    if not isinstance(roles, list) or not all(isinstance(r, str) for r in roles):
        raise ValueError(f'roles must be a list of names: {spec!r}')
    return list(roles)


def registry_hosts(text):
    """``[(host, [roles])]`` of a devices.yaml's ``hosts:`` mapping, in file order; ValueError on any doubt."""
    hosts, in_hosts, indent, key_depth, listing = [], False, None, None, False
    for raw in text.splitlines():
        line = uncomment(raw).rstrip()
        body = line.strip()
        if not body:
            continue
        depth = len(line) - len(line.lstrip(' '))
        if line[depth:depth + 1] == '\t' or body in ('---', '...'):
            raise ValueError('tabs and documents are not read')
        if depth == 0 and body[:1] == '-' and not in_hosts:
            continue                    # an item of another top-level list (ping:)
        if depth == 0:
            key = KEY.match(body)
            if not key:
                raise ValueError(f'not a key: {body!r}')
            in_hosts, indent = scalar(key.group(1)) == 'hosts', None
            if in_hosts and key.group(2):
                found = flow(key.group(2))
                if not isinstance(found, dict):
                    raise ValueError('hosts must be a mapping')
                hosts += [(name, roles_of(spec)) for name, spec in found.items()]
                in_hosts = False
            continue
        if not in_hosts:
            continue
        indent = depth if indent is None else indent
        if depth < indent:
            raise ValueError('hosts is indented unevenly')
        if body.startswith('- ') or body == '-':
            if depth == indent or not hosts:
                raise ValueError('hosts must be a mapping')
            if listing:
                hosts[-1][1].append(scalar(body[1:]))
            continue
        key = KEY.match(body)
        if not key:
            raise ValueError(f'not a key: {body!r}')
        name, value = scalar(key.group(1)), key.group(2)
        if depth == indent:
            if any(name == h for h, _ in hosts):
                raise ValueError(f'host {name} twice')
            hosts.append((name, roles_of(flow(value)) if value else []))
            key_depth, listing = None, False
            continue
        key_depth = depth if key_depth is None else key_depth
        if depth == key_depth:          # one key of the host's own mapping; deeper ones belong to another key
            listing = name == 'roles' and not value
            if name == 'roles' and value:
                hosts[-1][1][:] = roles_of({'roles': flow(value)})
    return [(name, tuple(roles)) for name, roles in hosts]


def top(text, key):
    """A top-level ``key: value`` (a scalar or a flow value) of a small YAML file (profile.yaml), or None."""
    for line in text.splitlines():
        found = KEY.match(uncomment(line).rstrip()) if line[:1] not in ' \t' else None
        if found and scalar(found.group(1)) == key and found.group(2):
            return flow(found.group(2))
    return None


def workspaces():
    """This checkout's workspace (``src/<repo>/<package>/hook/`` up four), ``$ROS_WS`` and ``~/astro_dock``."""
    ws = os.environ.get('ROS_WS')
    found = []
    for root in (Path(__file__).resolve().parents[4], Path(ws) if ws and os.path.isabs(ws) else None,
                 Path.home() / 'astro_dock'):
        if root and root.is_dir() and root.resolve() not in found:
            found.append(root.resolve())
    return found


def profile_roots():
    """The profile set: ``$ASTRO_PROFILES`` (kept when missing: unread), else profiles.env's, else a workspace's."""
    explicit = os.environ.get('ASTRO_PROFILES')
    if not explicit:
        config = os.environ.get('ASTRO_PROFILES_CONFIG') or Path(
            os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config') / 'astro' / 'profiles.env'
        try:
            explicit = next((m.group(1) for m in re.finditer(r'^ASTRO_PROFILES=["\']?([^"\'\n]+)',
                                                             Path(config).read_text(), re.M)), None)
        except OSError:
            pass
    if explicit:
        return [Path(explicit).expanduser()]
    return [ws / 'profiles' for ws in workspaces() if (ws / 'profiles').is_dir()]


def sources():
    """``(registry files, vessels, ok)``: every registry found, the kind: vessel profiles, and False when a named
    source is missing or unreadable."""
    files, vessels, ok = [], [], True
    explicit = os.environ.get('ASTRO_REGISTRY')
    if explicit:
        files.append(Path(explicit).expanduser())
    for root in profile_roots():
        if not root.is_dir():
            ok = False
            continue
        for profile in sorted(root.glob('*/profile.yaml')):
            try:
                text = profile.read_text()
                kind, registry = top(text, 'kind'), top(text, 'registry')
            except (OSError, ValueError):
                ok = False
                continue
            if kind == 'vessel':
                vessels.append(profile.parent.name)
            if registry:
                files.append(profile.parent / registry)
    files += [ws / LEGACY for ws in workspaces() if (ws / LEGACY).is_file()]
    return list(dict.fromkeys(f.resolve() for f in files)), vessels, ok


def boat_domains():
    """Every vessel's ROS domain: ``ROS_DOMAIN_ID`` of each ``kind: vessel`` profile's network.env in the profile set
    (none without one). A sim or preview never takes one."""
    found = set()
    for root in profile_roots():
        for profile in sorted(root.glob('*/profile.yaml')) if root.is_dir() else ():
            try:
                if top(profile.read_text(), 'kind') == 'vessel':
                    found.update(int(m) for m in re.findall(r'^ROS_DOMAIN_ID=["\']?(\d+)', (
                        profile.parent / 'network.env').read_text(), re.M))
            except (OSError, ValueError):
                continue
    return tuple(sorted(found))


def dev_container():
    """The dev container boat_test runs in: ``defaults.container`` of the first registry that names one (a vessel's,
    then this package's config/devices.yaml), else ''."""
    package = Path(__file__).resolve().parents[2] / LEGACY
    for path in [*sources()[0], package]:
        try:
            defaults = top(path.read_text(), 'defaults')
        except (OSError, ValueError):
            continue
        if isinstance(defaults, dict) and defaults.get('container'):
            return defaults['container']
    return ''


def fleet(boat_roles):
    """The boat compute hosts (a role in ``boat_roles``) of every registry, and the vessels; ``read`` False on doubt,
    and on any error (the dispatcher lets a call through when a hook raises: here that would block less)."""
    try:
        return _fleet(boat_roles)
    except Exception:
        return Fleet((), (), False)


def _fleet(boat_roles):
    files, vessels, ok = sources()
    rank = {role: i for i, role in enumerate(boat_roles)}
    hosts = []
    for path in files:
        try:
            found = registry_hosts(path.read_text())
        except (OSError, ValueError):
            ok = False
            continue
        boats = [(min(rank[r] for r in roles if r in rank), i, name) for i, (name, roles) in enumerate(found)
                 if name != FALLBACK and any(r in rank for r in roles)]
        hosts += [name for _, _, name in sorted(boats) if name not in hosts]
    return Fleet(tuple(hosts), tuple(vessels), ok and bool(files))
