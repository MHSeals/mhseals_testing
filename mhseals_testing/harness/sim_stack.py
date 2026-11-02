"""config/sim/stack.yaml for tools/boat_sim: the settings as shell assignments, and a free sim domain. ROS-free.

    python3 -m mhseals_testing.harness.sim_stack env      # SIM_PLAYER=... lines for the script to eval
    python3 -m mhseals_testing.harness.sim_stack domain   # a ROS_DOMAIN_ID in ``domains`` no other sim holds

``BOAT_SIM_<KEY>`` in the environment wins over the file. A domain is held by a ``domain.<N>`` file (its pid) in
``DOMAINS`` while its sim runs (boat_sim removes it on down); a stale one (its pid gone) is free again.
"""

import os
import random
import shlex
import sys
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'sim' / 'stack.yaml'
PATHS = ('player', 'run', 'overlay', 'base')
DOMAINS = Path(f'/tmp/boat-sim-{os.getuid()}')


def settings(path=CONFIG, environ=None):
    environ = os.environ if environ is None else environ
    found = yaml.safe_load(Path(path).read_text()) or {}
    out = {}
    for key, value in found.items():
        value = environ.get(f'BOAT_SIM_{key.upper()}', value)
        if key == 'domains' and isinstance(value, str):
            value = [int(v) for v in value.replace(',', ' ').split()]
        out[key] = os.path.expanduser(str(value)) if key in PATHS else value
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
