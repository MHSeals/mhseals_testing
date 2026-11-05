"""Profile sets for the agent hook's tests: the device registry the hook derives its boat hosts from (hook/registry.py).

``SHIPPED`` mirrors the workspace's profile set as of 11-04 (blastoise's devices.yaml verbatim; squirtle a stub
vessel with no registry yet). ``THREE`` adds a registry for squirtle with the one computer the hook named in code
before 11-04 (squirtle-jetson), so the hook's own tests keep their three hosts. ``RENAMED`` is two other boats.
"""

from pathlib import Path

BLASTOISE = '''# Which computer has which role on the boat (the deployment layer of the device registry).
hosts:
  blastoise-odroid: {roles: [boat_computer, time_server], user: roboboat, capabilities: [mavros, thrusters]}
  blastoise-jetson: {roles: [camera_host], user: roboboat, capabilities: [gpu, zed, lidar]}
  tranquility: {roles: [operator, gateway], user: peace, address: $GATEWAY_IP}   # RTK base, operator heartbeat
  router: {roles: [router], address: $ROUTER_IP, ssh: false}
  rocket: {roles: [radio], address: $UBNT_ROCKET_IP, ssh: false}
  bullet: {roles: [radio], address: $UBNT_BULLET_IP, ssh: false}
  vlp16: {roles: [lidar_sensor], address: $LIDAR_IP, ssh: false}
ping: [camera_host, boat_computer, router, radio, lidar_sensor]   # this machine is skipped
'''
SQUIRTLE = '''hosts:
  squirtle-jetson:
    roles: [camera_host]
    user: roboboat
'''
KESTREL = '''hosts:
  kestrel-cam:          # the camera first in the file: the vessel's name still reaches its boat computer
    roles:
      - camera_host
  kestrel-main: {roles: [time_server, boat_computer]}
  shore: {roles: [operator]}
'''
HERON = '''hosts:
  "heron-pc": {roles: [boat_computer], user: crew}
'''
VESSEL = 'extends: fleet\nkind: vessel\n'
# {profile directory: (profile.yaml, devices.yaml or None)}
SHIPPED = {'roboboat': ('# the fleet\n', None), 'control_station': ('extends: roboboat\nkind: station\n', None),
           'blastoise': (VESSEL + 'network_env: network.env\nregistry: devices.yaml\n', BLASTOISE),
           'squirtle': (VESSEL, None)}
THREE = dict(SHIPPED, squirtle=(VESSEL + 'registry: devices.yaml\n', SQUIRTLE))
RENAMED = {'fleet': ('# the fleet\n', None), 'kestrel': (VESSEL + "registry: 'devices.yaml'\n", KESTREL),
           'heron': (VESSEL + 'registry: devices.yaml   # its hosts\n', HERON)}


def profile_set(root, profiles):
    """Write ``profiles`` under ``root``; returns ``root``."""
    for name, (profile, devices) in profiles.items():
        (Path(root) / name).mkdir(parents=True, exist_ok=True)
        (Path(root) / name / 'profile.yaml').write_text(profile)
        if devices is not None:
            (Path(root) / name / 'devices.yaml').write_text(devices)
    return Path(root)


def use(monkeypatch, hook, root):
    """Make the profile set at ``root`` the hook's only registry source (no $ASTRO_REGISTRY, no workspace's)."""
    monkeypatch.setenv('ASTRO_PROFILES', str(root))
    monkeypatch.delenv('ASTRO_REGISTRY', raising=False)
    monkeypatch.setattr(hook, 'workspaces', lambda: [])
