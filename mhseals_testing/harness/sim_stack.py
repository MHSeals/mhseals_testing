"""config/sim/stack.yaml for tools/boat_sim: the settings as shell assignments, and a free sim domain. ROS-free.

    python3 -m mhseals_testing.harness.sim_stack env      # SIM_PLAYER=... lines for the script to eval
    python3 -m mhseals_testing.harness.sim_stack domain   # a ROS_DOMAIN_ID in ``domains`` no other sim holds

``BOAT_SIM_<KEY>`` in the environment wins over the file. ``image: auto`` is this machine's newest dev container
image matching ``image_match`` (Dev Containers names them vsc-<workspace>-<hash>-uid), and ``player`` may list candidates: the first that
exists here (a directory: its first executable ``*.x86_64``), so no machine's image hash or install path is code. A domain is held by a ``domain.<N>`` file (its pid) in
``DOMAINS`` while its sim runs (boat_sim removes it on down); a stale one (its pid gone) is free again.
"""

import os
import random
import shlex
import subprocess
import sys
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'sim' / 'stack.yaml'
PATHS = ('player', 'run', 'overlay')
DOMAINS = Path(f'/tmp/boat-sim-{os.getuid()}')


def local_image(match, run=subprocess.run):
    """This machine's newest image matching ``match`` (``docker images`` lists newest first), or '' without one."""
    try:
        out = run(['docker', 'images', '--filter', f'reference={match}', '--format', '{{.Repository}}:{{.Tag}}'],
                  capture_output=True, text=True, timeout=10).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return ''
    return out[0] if out else ''


def first_player(candidates):
    """The first candidate that exists (a directory: its first executable ``*.x86_64``), else the first one, so the
    caller's missing-player message names it."""
    for path in map(Path, candidates):
        if path.is_dir():
            path = next((p for p in sorted(path.glob('*.x86_64')) if os.access(p, os.X_OK)), None)
        if path is not None and path.is_file():
            return str(path)
    return str(candidates[0]) if candidates else ''


def settings(path=CONFIG, environ=None, image=local_image):
    environ = os.environ if environ is None else environ
    found = yaml.safe_load(Path(path).read_text()) or {}
    out = {}
    for key, value in found.items():
        value = environ.get(f'BOAT_SIM_{key.upper()}', value)
        if key == 'domains' and isinstance(value, str):
            value = [int(v) for v in value.replace(',', ' ').split()]
        if key == 'player':
            candidates = value.split(':') if isinstance(value, str) else list(value)
            value = first_player([os.path.expanduser(str(v)) for v in candidates])
        out[key] = os.path.expanduser(str(value)) if key in PATHS else value
    if out.get('image') == 'auto':
        out['image'] = image(out.get('image_match') or 'vsc-*-uid')
    return out


def shell(values):
    """``SIM_KEY='value'`` lines; a list is space-joined."""
    return '\n'.join(f"SIM_{key.upper()}={shlex.quote(' '.join(map(str, value)) if isinstance(value, list) else str(value))}"
                     for key, value in values.items())


def held(directory):
    """Domains a live sim holds: ``domain.<N>`` files whose pid still runs."""
    out = set()
    for path in Path(directory).glob('domain.*'):
        try:
            pid = int(path.read_text().split()[0])
            os.kill(pid, 0)
        except (ValueError, IndexError, OSError):
            continue
        out.add(int(path.suffix[1:]))
    return out


def free_domain(low, high, taken, choose=random.choice):
    """A domain in [low, high] not in ``taken`` and never 42 (the boat's); None when all are taken."""
    free = [d for d in range(low, high + 1) if d not in taken and d != 42]
    return choose(free) if free else None


def main(argv):
    values = settings()
    if argv[:1] == ['env']:
        print(shell(values))
        return 0
    if argv[:1] == ['domain']:
        low, high = values['domains']
        found = free_domain(low, high, held(DOMAINS))
        if found is None:
            print('no free sim domain', file=sys.stderr)
            return 1
        print(found)
        return 0
    print(__doc__.split('\n\n')[1], file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
