"""topic_rate: is a topic arriving at its nominal rate, evenly and freshly stamped?

Facts come from the ``rates`` probe, keyed by topic: value is the measured Hz,
``data`` holds ``max_gap`` (s, rolling 10 s) and ``stamp_age`` (s). The same
``classify`` colours the Hz on the dashboard, so both read one rule.
"""

from dataclasses import dataclass

from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind
from mhseals_testing.profiles import MIN_DEADLINE


@dataclass(frozen=True)
class Params:
    topic: str
    hz: float                    # nominal rate
    warn: float = 0.85           # below this fraction of hz is warn
    fail: float = 0.5            # below this fraction is fail
    high: float = 1.6            # above this fraction is warn (a doubled publisher)
    gap: float = 3.0             # a silence longer than this many periods is warn
    stamp_age: float = 0.0       # s; 0 derives max(3/hz, 1.5)


def classify(nominal, measured, max_gap=0.0, stamp_age=None, warn=0.85, fail=0.5,
             high=1.6, gap=3.0, limit=0.0):
    """(status, mark) for ``measured`` Hz against ``nominal``: mark is one of ' ↓↑~×'."""
    if not measured:
        return 'fail', '×'
    ratio = measured / nominal
    if ratio < fail:
        return 'fail', '↓'
    if ratio < warn:
        return 'warn', '↓'
    if ratio > high:
        return 'warn', '↑'
    if max_gap > gap / nominal:
        return 'warn', '~'
    if stamp_age is not None and stamp_age > (limit or max(3 / nominal, MIN_DEADLINE)):
        return 'warn', '~'
    return 'ok', ''


@kind('topic_rate', Params, needs=('rates',), fields=('hz', 'max_gap', 'stamp_age'),
      watch=lambda p: {'rates': [p.topic]})
def evaluate(params, facts):
    fact = facts.get(params.topic)
    if fact is None or not isinstance(fact.value, (int, float)):
        return Result('unknown', detail=f'no rate sample for {params.topic}')
    max_gap, stamp_age = fact.data.get('max_gap', 0.0), fact.data.get('stamp_age')
    status, mark = classify(params.hz, fact.value, max_gap, stamp_age, params.warn,
                            params.fail, params.high, params.gap, params.stamp_age)
    detail = f'{params.topic} {fact.value:.1f} Hz of {params.hz:g}, worst gap {max_gap:.2f} s'
    return Result(status, value=f'{fact.value:.1f} Hz', detail=detail, mark=mark,
                  data=dict(hz=fact.value, max_gap=max_gap, stamp_age=stamp_age))
