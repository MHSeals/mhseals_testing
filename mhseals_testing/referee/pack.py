"""Load a year's challenge pack: ``config/challenges/<pack>/pack.yaml`` plus one YAML per challenge.

A challenge names a ``kind``, the module in ``challenges/`` that judges it; the
directory listing is the registry. A typo in a key or a missing kind fails at load.
"""

import importlib
from dataclasses import dataclass
from pathlib import Path

import yaml

from mhseals_testing.profiles import PROFILE_DIR

CHALLENGE_DIR = PROFILE_DIR.parent / 'challenges'
KEYS = {'schema', 'name', 'title', 'kind', 'tree', 'truth_task', 'time_limit', 'judge'}
KINDS = Path(__file__).parent / 'challenges'


@dataclass(frozen=True)
class Spec:
    name: str
    title: str
    kind: str
    module: object
    params: dict
    truth_task: str
    time_limit: float | None = None
    tree: str | None = None


@dataclass(frozen=True)
class Pack:
    name: str
    challenges: dict  # name -> Spec, in the pack's order


def kinds():
    return sorted(p.stem for p in KINDS.glob('*.py') if p.stem != '__init__')


def load_spec(path):
    data = yaml.safe_load(Path(path).read_text())
    if data.get('schema') != 'mhseals.challenge/1' or set(data) - KEYS:
        raise ValueError(f'{path}: not a mhseals.challenge/1 file or has unknown keys {set(data) - KEYS}')
    if data['kind'] not in kinds():
        raise ValueError(f"{path}: unknown kind {data['kind']!r}; available: {', '.join(kinds())}")
    module = importlib.import_module(f"mhseals_testing.referee.challenges.{data['kind']}")
    given = data.get('judge', {})
    unknown = set(given) - set(module.PARAMS) - {'count'}
    if unknown:
        raise ValueError(f'{path}: unknown judge keys {sorted(unknown)}')
    params = {**module.PARAMS, **given}
    if 'count' in given and set(given['count']) != {'label', 'event', 'name', 'key'}:
        raise ValueError(f'{path}: count needs label, event, name and key')
    return Spec(data['name'], data.get('title', data['name']), data['kind'], module, params,
                data.get('truth_task', data['name']), data.get('time_limit'), data.get('tree'))


def load(name, directory=CHALLENGE_DIR):
    root = Path(directory) / name
    head = yaml.safe_load((root / 'pack.yaml').read_text())
    if head.get('schema') != 'mhseals.pack/1':
        raise ValueError(f'{root / "pack.yaml"}: not a mhseals.pack/1 file')
    specs = [load_spec(root / f'{item}.yaml') for item in head['order']]
    return Pack(head['name'], {s.name: s for s in specs})
