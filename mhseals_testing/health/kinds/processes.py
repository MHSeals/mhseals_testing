"""processes: are the right processes running, once, from the current install?

Facts:
  host.processes        list of ``{pid, cmd, start, managed}``; ``managed`` is true for a session
                        child or a child adopted from a taken-over instance
  install.<package>     epoch mtime of a package's installed files (for ``newer_than``)
"""

from dataclasses import dataclass
import re

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    match: str                   # regex searched in the command line
    max: int = 1                 # more matching processes than this is a duplicate
    managed: bool = False        # a match that is not a (adopted) session child is an orphan
    newer_than: tuple = ()       # packages; a match started before their install is stale


@kind('processes', Params, needs=('host',), fields=('count', 'orphans', 'stale'))
def evaluate(params, facts):
    fact = facts.get('host.processes')
    if fact is None:
        return Result('unknown', detail='no host.processes fact')
    found = [p for p in fact.value if re.search(params.match, p['cmd'])]
    orphans = [p['pid'] for p in found if params.managed and not p.get('managed')]
    installed = [facts[f'install.{name}'].value for name in params.newer_than
                 if f'install.{name}' in facts]
    stale = [p['pid'] for p in found if installed and p['start'] < max(installed)]
    problems = []
    if orphans:
        problems.append(f'orphan pid {", ".join(map(str, orphans))}')
    if stale:
        problems.append(f'older than install: pid {", ".join(map(str, stale))}')
    if len(found) > params.max:
        problems.append(f'{len(found)} running, at most {params.max}')
    data = {'count': len(found), 'orphans': orphans, 'stale': stale}
    return Result('warn' if problems else 'ok', value=str(len(found)),
                  detail='; '.join(problems) or f'{len(found)} matching', data=data)
