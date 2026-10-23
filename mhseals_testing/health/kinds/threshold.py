"""threshold: one numeric fact judged against warn and fail bounds."""

from dataclasses import dataclass

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    fact: str                    # name of the fact to read
    warn: float                  # beyond this is warn
    fail: float                  # beyond this is fail
    unit: str = ''
    below: bool = True           # True: low values are bad; False: high values are bad


def _worse(params, value, bound):
    return value < bound if params.below else value > bound


@kind('threshold', Params, needs=(), fields=('value',))
def evaluate(params, facts):
    fact = facts.get(params.fact)
    if fact is None or not isinstance(fact.value, (int, float)):
        return Result('unknown', detail=f'no {params.fact} fact')
    value = fact.value
    status = ('fail' if _worse(params, value, params.fail)
              else 'warn' if _worse(params, value, params.warn) else 'ok')
    text = f'{value:.2f}{params.unit}'
    return Result(status, value=text, detail=f'{params.fact} {text}')
