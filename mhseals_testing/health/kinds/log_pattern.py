"""log_pattern: did a log line we know means trouble appear recently?

Answers questions like "is the FCU refusing to arm" (PreArm lines), "did Fast DDS
complain on a child's stderr" (RTPS errors) and "is the EKF missing its rate" from
/rosout and the managed child logs, which the ``logs`` probe matches. Motivated by the
2026-10 sessions where the same handful of lines explained every failed run and nobody
saw them scroll past. The check is active while at least ``min_count`` matches of one
``key`` fall within the last ``hold`` seconds; named groups of the pattern are in
``data`` and usable in ``message``. A ``suppress`` pattern never alerts: it swallows
known noise and reports how many lines it ate.
"""

from dataclasses import dataclass, field

from mhseals_testing.health.model import Result
from mhseals_testing.health.probes.logs import fact_name
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    pattern: str                 # regex searched in each line
    nodes: list = field(default_factory=list)     # node globs the line must come from; empty: any
    key: str = ''                # named group that splits distinct messages (counted apart)
    hold: float = 30.0           # seconds a match keeps the check active
    min_count: int = 1           # matches of one key within hold
    suppress: bool = False       # swallow matching lines instead of alerting


@kind('log_pattern', Params, needs=('logs',), watch=lambda p: {'logs': [p]}, fields=('count',))
def evaluate(params, facts):
    fact = facts.get(fact_name(params))
    if fact is None:
        return Result('unknown', detail='logs probe is not watching this pattern')
    if params.suppress:
        return Result('ok', detail=f'{fact.value} suppressed')
    if fact.value < params.min_count:
        return Result('ok')
    return Result('fail', value=f'x{fact.value}', detail=fact.data['line'][:160],
                  data={**fact.data['groups'], 'count': fact.value})
