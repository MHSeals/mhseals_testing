"""threshold: one named fact judged against bands or an expected value.

Answers "is this number inside its band?" or "is this value the one we expect?".
It stays deliberately dumb: one fact, no expressions. A derived quantity (load
per CPU, the worst sensor stamp offset) is computed by a probe and published
as a fact. Motivated by the advisory rows (battery, flight controller, clock,
compute) that were each a bespoke function before the engine.

Numeric: ``warn`` and ``fail`` are either a single bound (``below: true``, the
default, means values under it are bad, ``false`` means values over it are bad)
or a ``[lo, hi]`` range the value must stay inside. ``scale`` divides the number
first (a field strength over its expected total) and ``abs`` judges the
magnitude while the text keeps the sign; offsets of 1000 or more print without
decimals so the dashboard cell stays short, and ``fmt`` replaces the default
formatting. Categorical: ``expect`` is a value or a list of values; anything
else is a fail (the check's severity caps it). The fact's ``label`` joins the
detail (after the value, or before it with ``label_first``), so a probe can say
who or what the value is about; ``quiet`` leaves the label and the detail out
when another check on the same row already says it. ``probe`` names the probe
that owns the fact, so a silent probe reads as unknown instead of showing its
last value forever.
"""

from dataclasses import dataclass

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    fact: str                    # name of the fact to read
    warn: object = None          # bound, or [lo, hi] that is fine; beyond it is warn
    fail: object = None          # bound, or [lo, hi] that is fine; beyond it is fail
    unit: str = ''
    below: bool = True           # single bounds: True low values are bad, False high are bad
    scale: float = 1.0           # the fact is divided by this
    abs: bool = False            # judge the magnitude of the value
    digits: int = 2              # decimals shown
    fmt: str = ''                # e.g. '{:.2f}×'; replaces digits and unit when set
    expect: object = None        # value or list of values that are fine (replaces the bounds)
    probe: str = ''              # probe whose sample age makes the fact stale
    label_first: bool = False    # detail reads 'GPS -0.80s' instead of '0.80s GPS'
    quiet: bool = False          # contribute no detail to the dashboard row


def _worse(params, value, bound):
    if bound is None:
        return False
    if isinstance(bound, (list, tuple)):
        return not bound[0] <= value <= bound[1]
    return value < bound if params.below else value > bound


def _detail(params, text, label):
    if params.quiet:
        return ''
    return ' '.join(p for p in ((label, text) if params.label_first else (text, label)) if p)


def _text(params, value):
    if params.fmt:
        return params.fmt.format(value)
    digits = 0 if abs(value) >= 1000 else params.digits
    return f'{value:{"+" if params.abs else ""}.{digits}f}{params.unit}'


@kind('threshold', Params, needs=(), fields=('value', 'label'))
def evaluate(params, facts):
    fact = facts.get(params.fact)
    if fact is None:
        return Result('unknown', detail=f'no {params.fact} fact')
    data = {'label': fact.label}
    if params.expect is not None:
        wanted = params.expect if isinstance(params.expect, list) else [params.expect]
        fine = fact.value in wanted
        return Result('ok' if fine else 'fail', value=str(fact.value)[:10], data=data,
                      detail=_detail(params, '', fact.label or ('' if fine else f'{params.fact} is {fact.value}')))
    if isinstance(fact.value, bool) or not isinstance(fact.value, (int, float)):
        return Result('unknown', detail=f'no {params.fact} fact')
    value = fact.value / params.scale
    judged = abs(value) if params.abs else value
    status = ('fail' if _worse(params, judged, params.fail)
              else 'warn' if _worse(params, judged, params.warn) else 'ok')
    text = _text(params, value)
    return Result(status, value=text, data=data, detail=_detail(params, text, fact.label))
