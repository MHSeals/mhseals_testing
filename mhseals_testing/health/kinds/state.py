"""state: a text fact that must be one of the expected values; ``warn`` values only warn."""

from dataclasses import dataclass

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    fact: str                    # name of the fact to read
    expect: tuple                # values that are fine; anything else takes the check's severity
    warn: tuple = ()             # values that read warn rather than fail


@kind('state', Params, needs=(), fields=('reason',))
def evaluate(params, facts):
    fact = facts.get(params.fact)
    if fact is None or not isinstance(fact.value, str):
        return Result('unknown', detail=f'no {params.fact} fact')
    reason = str(fact.data.get('reason', ''))
    if fact.value in params.expect:
        return Result('ok', value=fact.value[:10], data={'reason': reason})
    return Result('warn' if fact.value in params.warn else 'fail', value=fact.value[:10], detail=reason or fact.value, data={'reason': reason})
