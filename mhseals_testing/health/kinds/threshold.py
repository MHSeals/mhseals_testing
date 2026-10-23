"""threshold: one named fact judged against bands or an expected value.

Answers "is this number inside its band?" or "is this value the one we expect?".
It stays deliberately dumb: one fact, no expressions. A derived quantity (load
per CPU, the worst sensor stamp offset) is computed by a probe and published
as a fact. Motivated by the advisory rows (battery, flight controller, clock,
compute) that were each a bespoke function before the engine.

Numeric: ``warn`` and ``fail`` are bounds; ``below: true`` (default) means
values under a bound are bad, ``false`` means values over it are bad. ``abs``
judges the magnitude and the text keeps the sign; offsets of 1000 or more print
without decimals so the dashboard cell stays short. Categorical: ``expect`` is a
value or a list of values; anything else is a fail (the check's severity caps it). The fact's ``label``
joins the detail (after the value, or before it with ``label_first``), so a
probe can say who or what the value is about; ``quiet`` leaves the label and the
detail out when another check on the same row already says it.
``probe`` names the probe that owns the fact, so a silent probe reads as
unknown instead of showing its last value forever.
"""

from dataclasses import dataclass

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    fact: str                    # name of the fact to read
    warn: float | None = None    # beyond this is warn
    fail: float | None = None    # beyond this is fail
    unit: str = ''
    below: bool = True           # True: low values are bad; False: high values are bad
    abs: bool = False            # judge the magnitude of the value
    digits: int = 2              # decimals shown
    expect: object = None        # value or list of values that are fine (replaces the bounds)
    probe: str = ''              # probe whose sample age makes the fact stale
    label_first: bool = False    # detail reads 'GPS -0.80s' instead of '0.80s GPS'
    quiet: bool = False          # contribute no detail to the dashboard row


def _worse(params, value, bound):
    return bound is not None and (value < bound if params.below else value > bound)


def _detail(params, text, label):
    if params.quiet:
        return ''
    return ' '.join(p for p in ((label, text) if params.label_first else (text, label)) if p)


@kind('threshold', Params, needs=(), fields=('value',))
def evaluate(params, facts):
    fact = facts.get(params.fact)
    if fact is None:
        return Result('unknown', detail=f'no {params.fact} fact')
    if params.expect is not None:
        wanted = params.expect if isinstance(params.expect, list) else [params.expect]
        fine = fact.value in wanted
        return Result('ok' if fine else 'fail', value=str(fact.value)[:10],
                      detail=_detail(params, '', fact.label or ('' if fine else f'{params.fact} is {fact.value}')))
    if isinstance(fact.value, bool) or not isinstance(fact.value, (int, float)):
        return Result('unknown', detail=f'no {params.fact} fact')
    value = fact.value
    judged = abs(value) if params.abs else value
    status = ('fail' if _worse(params, judged, params.fail)
              else 'warn' if _worse(params, judged, params.warn) else 'ok')
    digits = 0 if abs(value) >= 1000 else params.digits
    text = f'{value:{"+" if params.abs else ""}.{digits}f}{params.unit}'
    return Result(status, value=text, detail=_detail(params, text, fact.label))
