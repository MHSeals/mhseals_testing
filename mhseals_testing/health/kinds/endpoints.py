"""endpoints: who publishes and subscribes to a topic, and can they talk?

Incidents: two TUIs both commanding /cmd_vel (10-18); a lifespan publisher
dropping RTCM under clock skew; a reliable subscriber starved by a best-effort
publisher. Counts include our own endpoints. An unknown QoS field is
compatible, and a zero type hash is the `nodes` legacy signal, not a type mismatch.
"""

from dataclasses import dataclass
from fnmatch import fnmatchcase

from mhseals_testing.health.model import Result
from mhseals_testing.health.probes.graph import topic_fact_name
from mhseals_testing.health.registry import kind


@dataclass(frozen=True)
class Params:
    topic: str
    publishers: list | None = None     # [min, max] publisher count
    subscribers: list | None = None    # [min, max] subscriber count
    nodes: list | None = None          # FQN globs the publishers must match
    qos: bool = False                  # every publisher must be able to reach every subscriber
    forbid_qos: list | None = None     # 'lifespan': no publisher may set it
    type: bool = False                 # one message type on the topic


def _range_problem(side, count, bounds):
    low, high = bounds
    if count < low or count > high:
        return f'{count} {side}s, want {low}-{high}'


def qos_problems(pubs, subs):
    """Reliability and durability pairs that DDS would refuse to match."""
    out = []
    for pub in pubs:
        for sub in subs:
            if pub.qos.get('reliability') == 'best_effort' and sub.qos.get('reliability') == 'reliable':
                out.append(f'{pub.node} best_effort cannot feed reliable {sub.node}')
            if pub.qos.get('durability') == 'volatile' and sub.qos.get('durability') == 'transient_local':
                out.append(f'{pub.node} volatile cannot feed transient_local {sub.node}')
    return out


@kind('endpoints', Params, needs=('graph',), watch=lambda p: {'graph': [p.topic]},
      fields=('topic', 'count', 'nodes'))
def evaluate(params, facts):
    fact = facts.get(topic_fact_name(params.topic))
    if fact is None:
        return Result('unknown', detail=f'no graph sample of {params.topic}')
    pubs, subs = fact.value['pub'], fact.value['sub']
    names = ', '.join(sorted(ep.node for ep in pubs))
    problems = []
    for side, found, bounds in (('publisher', pubs, params.publishers),
                                ('subscriber', subs, params.subscribers)):
        if bounds:
            problems.append(_range_problem(side, len(found), bounds))
    for ep in pubs:
        if params.nodes and not any(fnmatchcase(ep.node, glob) for glob in params.nodes):
            problems.append(f'unexpected publisher {ep.node}')
        if 'lifespan' in (params.forbid_qos or ()) and ep.qos.get('lifespan', 0) > 0:
            problems.append(f'{ep.node} sets lifespan {ep.qos["lifespan"]:g} s')
    if params.qos:
        problems += qos_problems(pubs, subs)
    types = {ep.type for ep in (*pubs, *subs) if ep.type}
    if params.type and len(types) > 1:
        problems.append('types differ: ' + ', '.join(sorted(types)))
    problems = [p for p in problems if p]
    data = {'topic': params.topic, 'count': len(pubs), 'nodes': names}
    if problems:
        return Result('fail', value=f'{len(pubs)} pub', detail=f'{params.topic}: ' + '; '.join(problems), data=data)
    return Result('ok', value=f'{len(pubs)} pub', detail=f'{params.topic}: {len(pubs)} publishers', data=data)
