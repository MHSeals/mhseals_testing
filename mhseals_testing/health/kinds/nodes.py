"""nodes: is the node graph sane?

Incidents: a second node under one FQN (duplicate `mavros`, see the verify note
in health.md); a second TUI hidden by per-instance node names (counted by glob);
a Foxy container on our domain whose endpoints carry a zero type hash.
"""

from collections import Counter
from dataclasses import dataclass
from fnmatch import fnmatchcase

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    unique: bool = False               # no FQN appears twice
    count: dict | None = None          # {glob: [min, max]} nodes matching the glob
    require: list | None = None        # FQNs that must exist
    forbid: list | None = None         # FQN globs that must not exist
    legacy: bool = False               # no endpoint with a zero type hash


@kind('nodes', Params, needs=('graph',), fields=('count', 'nodes'))
def evaluate(params, facts):
    fact = facts.get('graph.nodes')
    if fact is None:
        return Result('unknown', detail='no graph node list')
    names = fact.value
    problems = []
    if params.unique:
        dupes = sorted(n for n, c in Counter(names).items() if c > 1)
        if dupes:
            problems.append('duplicate node ' + ', '.join(dupes))
    for glob, (low, high) in (params.count or {}).items():
        found = sum(fnmatchcase(n, glob) for n in names)
        if not low <= found <= high:
            problems.append(f'{found} nodes match {glob}, want {low}-{high}')
    problems += [f'{n} missing' for n in params.require or () if n not in names]
    problems += [f'{n} present' for n in names if any(fnmatchcase(n, g) for g in params.forbid or ())]
    if params.legacy:
        legacy = facts.get('graph.legacy_endpoints')
        if legacy is None:
            return Result('unknown', detail='no graph legacy count')
        if legacy.value:
            problems.append(f'{legacy.value} endpoints without a type hash (first: {legacy.label})')
    data = {'count': len(problems), 'nodes': ', '.join(sorted(set(names)))}
    if problems:
        return Result('fail', value=f'{len(problems)} issue', detail='; '.join(problems), data=data)
    return Result('ok', value=f'{len(names)} nodes', detail=f'{len(names)} nodes', data=data)
