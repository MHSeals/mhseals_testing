"""config/sim/stack.yaml for tools/boat_sim: the settings as shell assignments, and a free sim domain. ROS-free.

    python3 -m mhseals_testing.harness.sim_stack env [--resolve [--pull]]  # SIM_PLAYER=... lines to eval; --resolve:
        # with the workspace's image (--pull: newer from the registry first) and repos (SIM_REPOS name=ref ...)
    python3 -m mhseals_testing.harness.sim_stack domain   # a ROS_DOMAIN_ID in ``domains`` no other sim holds

``BOAT_SIM_<KEY>`` in the environment wins over the file. Nothing the workspace already knows is written here
(docs/preview.md): ``image: auto`` is the workspace's dev image (``hw.sh image --local``: the one Compose names, or
its local UID variant built on it), the repositories are the workspace's manifest (``manifest``: ``hw.sh hosts
repos --role`` of ``roles`` and astro_dock's submodule pins), and ``player`` may list candidates: the first that
exists here (a directory: its first executable ``*.x86_64``), so no machine's image hash or install path is code. A domain is held by a ``domain.<N>`` file (its pid) in
``DOMAINS`` while its sim runs (boat_sim removes it on down); a stale one (its pid gone) is free again.
"""

import os
import random
import re
import shlex
import subprocess
import sys
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'sim' / 'stack.yaml'
PATHS = ('player', 'run', 'overlay')
DOMAINS = Path(f'/tmp/boat-sim-{os.getuid()}')


class Missing(RuntimeError):
    """What the workspace lacks, with the one command that fixes it."""


def src_dir(environ=None):
    """The workspace's src/ (``BOAT_SIM_SRC``, else that of this checkout's main clone: a worktree's too)."""
    environ = os.environ if environ is None else environ
    if environ.get('BOAT_SIM_SRC'):
        return Path(os.path.expanduser(environ['BOAT_SIM_SRC']))
    here = Path(__file__).resolve().parents[2]
    try:
        common = subprocess.run(['git', '-C', str(here), 'rev-parse', '--path-format=absolute', '--git-common-dir'],
                                capture_output=True, text=True, timeout=10, check=True).stdout.strip()
        return Path(common).parent.parent
    except (OSError, subprocess.SubprocessError):
        return here.parent


def workspace(src):
    """The astro_dock root that holds ``src`` (its hw.sh), or Missing."""
    root = Path(src).parent
    if not (root / 'hw.sh').is_file():
        raise Missing(f'no astro_dock workspace around {src} (no hw.sh in {root}); set BOAT_SIM_SRC=<astro_dock>/src')
    return root


def _run(cmd, run, timeout=60):
    try:
        return run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as error:
        return subprocess.CompletedProcess(cmd, 1, '', str(error))


def _last(text):
    lines = [re.sub(r'\x1b\[[0-9;]*m', '', line) for line in (text or '').splitlines() if line.strip()]
    return lines[-1] if lines else 'no output'


def manifest(root, roles, run=subprocess.run):
    """The workspace's repositories: ``[(name, ref)]``. A team repo's ref is ``origin/<branch>`` as ``hw.sh hosts
    repos --role ROLE...`` lists it (repos.all and those roles' of the profile's hosts.yaml, what setup and hosts
    apply check out); a submodule of astro_dock (a gitlink under src/) is its pinned commit. Missing names the fix."""
    root = Path(root)
    cmd = [str(root / 'hw.sh'), 'hosts', 'repos'] + [arg for role in roles or [] for arg in ('--role', str(role))]
    done = _run(cmd, run)
    if done.returncode != 0:
        raise Missing(f'{" ".join(cmd[1:])} failed ({_last(done.stderr)}); check out the profiles: '
                      f'{root}/hw.sh profile sync URL [REF]')
    found = [(line.split()[0], f'origin/{line.split()[2]}') for line in done.stdout.splitlines()
             if len(line.split()) == 3]
    pins = _run(['git', '-C', str(root), 'ls-tree', 'HEAD', 'src/'], run, timeout=20)
    for line in pins.stdout.splitlines():
        meta, _, path = line.partition('\t')
        if meta.split()[1:2] == ['commit']:
            found.append((Path(path).name, meta.split()[2]))
    for name, ref in found:
        if not (root / 'src' / name / '.git').exists():
            fix = (f'git -C {root} submodule update --init src/{name}' if len(ref) == 40 and '/' not in ref
                   else f'{root}/setup.linux.sh --components repos')
            raise Missing(f'the workspace manifest lists {name}, which {root}/src lacks; {fix}')
    return found


def dev_image(root, pull=False, run=subprocess.run):
    """The workspace's dev image to run here (``hw.sh image --local``, after ``--pull`` with ``pull``), or Missing."""
    cmd = [str(Path(root) / 'hw.sh'), 'image', '--local'] + (['--pull'] if pull else [])
    done = _run(cmd, run, timeout=1800 if pull else 60)
    if done.returncode != 0 or not done.stdout.strip():
        raise Missing(f'no dev image: {_last(done.stderr)}')
    return done.stdout.split()[-1]


def vessel(root, run=subprocess.run):
    """This machine's vessel files as the workspace resolves them for its dev container (dev.bashrc): ``registry``
    (``hw.sh whoami --registry``) and ``network_env`` (``hw.sh env``), each '' without a vessel (generic mode), so
    a station's ``self:`` in ~/.config/mhseals/devices.yaml names a host the stack knows."""
    out = {}
    for key, cmd in (('registry', ['whoami', '--registry']), ('network_env', ['env'])):
        done = _run([str(Path(root) / 'hw.sh'), *cmd], run, timeout=30)
        found = done.stdout.strip().splitlines()[-1:] if done.returncode == 0 else []
        out[key] = found[0] if found and Path(found[0]).is_file() else ''
    return out


def resolve(values, root, pull=False, image=dev_image, repos=manifest):
    """``values`` with ``image: auto`` and the ``repos`` the workspace at ``root`` names."""
    out = dict(values)
    if out.get('image') == 'auto':
        out['image'] = image(root, pull=pull)
    out['repos'] = repos(root, out.get('roles') or [])
    return out


def first_player(candidates):
    """The first candidate that exists (a directory: its first executable ``*.x86_64``), else the first one, so the
    caller's missing-player message names it."""
    for path in map(Path, candidates):
        if path.is_dir():
            path = next((p for p in sorted(path.glob('*.x86_64')) if os.access(p, os.X_OK)), None)
        if path is not None and path.is_file():
            return str(path)
    return str(candidates[0]) if candidates else ''


def settings(path=CONFIG, environ=None):
    """The file's settings with ``BOAT_SIM_<KEY>`` over them; ``image: auto`` and the repos are ``resolve``'s."""
    environ = os.environ if environ is None else environ
    found = yaml.safe_load(Path(path).read_text()) or {}
    out = {}
    for key, value in found.items():
        value = environ.get(f'BOAT_SIM_{key.upper()}', value)
        if key in ('domains', 'roles') and isinstance(value, str):
            value = value.replace(',', ' ').split()
        if key == 'domains':
            value = [int(v) for v in value]
        if key == 'player':
            candidates = value.split(':') if isinstance(value, str) else list(value)
            value = first_player([os.path.expanduser(str(v)) for v in candidates])
        out[key] = os.path.expanduser(str(value)) if key in PATHS else value
    return out


def flat(value):
    """A value as one shell word: a list space-joined, a (name, ref) pair ``name=ref``."""
    if isinstance(value, (list, tuple)):
        return ' '.join(flat(v) if not isinstance(v, (list, tuple)) else '='.join(map(str, v)) for v in value)
    return str(value)


def shell(values, prefix='SIM'):
    """``SIM_KEY='value'`` lines; a list is space-joined, the repos ``name=ref`` words."""
    return '\n'.join(f"{prefix}_{key.upper()}={shlex.quote(flat(value))}" for key, value in values.items()
                     if not (isinstance(value, list) and value and isinstance(value[0], dict)))


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
    if argv[:1] == ['env']:     # env [--resolve [--pull]]: with --resolve, the workspace's image and repos
        src = src_dir()
        try:
            root = workspace(src)
            if '--resolve' in argv:
                values = resolve(values, root, pull='--pull' in argv)
            values = dict(values, src=str(src), workspace=str(root))
        except Missing as error:
            print(f'boat_sim: {error}', file=sys.stderr)
            return 2
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
