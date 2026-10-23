"""params: do live parameters match a reference?

The reference is an ArduPilot ``.param`` file (``NAME,value`` lines) and/or an
inline ``expect`` mapping, which wins. Incident 10-19: EK3_SRC1_YAW 1->8 and
COMPASS_USE 1->0 froze the EKF yaw at 0.
A mismatch returns ``fail``; the check's severity caps it. Names the probe has not read give
``unknown`` unless something already differs.
"""

from dataclasses import dataclass, field
import functools
from pathlib import Path

from mhseals_testing.health.model import Result
from mhseals_testing.health.probes.params import fact_name
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    node: str                                    # e.g. /mavros/param
    names: list                                  # parameters to compare
    reference: str = ''                          # .param file
    expect: dict = field(default_factory=dict)   # inline reference, overrides the file
    tol: float = 1e-6


@functools.lru_cache(maxsize=8)
def _parse(path, mtime):
    values = {}
    for line in Path(path).read_text().splitlines():
        line = line.split('#')[0].strip()
        if line:
            name, _, text = line.replace(',', ' ').partition(' ')
            try:
                values[name] = float(text)
            except ValueError:
                pass
    return values


def reference_values(path):
    """``{name: value}`` from a .param file, re-read when it changes."""
    path = Path(path).expanduser()
    return _parse(str(path), path.stat().st_mtime_ns)


def _watch(params):
    return {'params': [f'{params.node}:{name}' for name in params.names]}


@kind('params', Params, needs=('params',), watch=_watch, fields=('value',))
def evaluate(params, facts):
    reference = reference_values(params.reference) if params.reference else {}
    reference = {**reference, **params.expect}
    differ, unread = [], []
    for name in params.names:
        if name not in reference:
            unread.append(f'{name} not in the reference')
            continue
        fact = facts.get(fact_name(params.node, name))
        if fact is None:
            unread.append(f'{name} not read yet')
        elif abs(fact.value - reference[name]) > params.tol:
            differ.append(f'{name} {fact.value:g} (want {reference[name]:g})')
    if differ:
        return Result('fail', value=f'{len(differ)} differ', detail='; '.join(differ))
    if unread:
        return Result('unknown', detail='; '.join(unread))
    return Result('ok', value='match', detail=f'{len(params.names)} parameters match')
