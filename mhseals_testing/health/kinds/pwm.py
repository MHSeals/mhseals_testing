"""pwm_owner and pwm_readback: who drives the thrusters, and what the PWM outputs are doing.

Facts (published by the graph, snapshot and host probes):
  graph.nodes                list of node names
  graph.subscribers:<topic>  list of ``{node, gid, self}`` endpoints; ``self`` ones are ours and skipped
  session.armed              bool
  session.driver_managed     bool, the driver is a session child or an adopted one
  pwm.readback               value is the legacy level (OK/WARN/FAIL/N/A), data has ``detail``
"""

from dataclasses import dataclass

from mhseals_testing.health.legacy import pwm_driver
from mhseals_testing.health.model import Result
from mhseals_testing.health.registry import kind

STATUS = {'OK': 'ok', 'WARN': 'warn', 'FAIL': 'fail', 'N/A': 'unknown'}


@dataclass(frozen=True)
class OwnerParams:
    topic: str                   # the topic the driver subscribes to
    driver: str                  # node name of the PWM driver


@kind('pwm_owner', OwnerParams, needs=('graph', 'host', 'snapshot'), fields=('detail',))
def owner(params, facts):
    """Exactly one managed driver, subscribed to the command topic (incident 10-16: two drivers)."""
    needed = ('graph.nodes', f'graph.subscribers:{params.topic}', 'session.armed',
              'session.driver_managed')
    missing = [name for name in needed if facts.get(name) is None]
    if missing:
        return Result('unknown', detail='no ' + ', '.join(missing))
    subscribers = [e['node'] for e in facts[needed[1]].value if not e.get('self')]
    level, detail = pwm_driver(facts['graph.nodes'].value, subscribers, params.topic,
                               bool(facts['session.armed'].value),
                               bool(facts['session.driver_managed'].value), params.driver)
    return Result(STATUS[level], value=detail.split(';')[0], detail=detail, data={'detail': detail})


@dataclass(frozen=True)
class ReadbackParams:
    pass


@kind('pwm_readback', ReadbackParams, needs=('host',), fields=('detail',))
def readback(params, facts):
    """Sysfs PWM outputs against neutral or the command; the host probe runs ``legacy.pwm_readback``."""
    fact = facts.get('pwm.readback')
    if fact is None:
        return Result('unknown', detail='no pwm.readback fact')
    detail = fact.data.get('detail', '')
    return Result(STATUS[fact.value], value=detail.split(';')[0][:10], detail=detail,
                  data={'detail': detail})
