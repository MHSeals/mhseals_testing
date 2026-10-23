"""graph probe: who publishes and subscribes to the watched topics, and who is on the graph.

The source is anything with ``endpoints(topic) -> (publishers, subscribers)``,
``node_names() -> [fqn]`` (duplicates repeated) and ``topics() -> [name]``;
ros/graph.py adapts rclpy to it and tests pass a fake. Our own endpoints are
tagged ``own`` and never dropped, so a check can count them or not.

Facts:
  graph.topic:<topic>        value {'pub': (Endpoint, ...), 'sub': (Endpoint, ...)}
  graph.nodes                value [fqn, ...] with duplicates kept (full pass only)
  graph.legacy_endpoints     value N endpoints whose type hash is all zeros, labelled with the
                             first node: a Foxy/Humble peer on our domain (full pass only)
"""

from dataclasses import dataclass, field

from mhseals_testing.health.model import Fact


@dataclass(frozen=True)
class Endpoint:
    node: str                    # node FQN
    type: str = ''               # message type name
    hash: str | None = None      # type hash in hex; None when the RMW reports none
    qos: dict = field(default_factory=dict)    # reliability, durability, lifespan (s, 0 = none)
    gid: str = ''
    own: bool = False            # created by this process

    @property
    def legacy(self):
        """A zero type hash: an older peer. Unknown (None) is not legacy."""
        return self.hash is not None and not self.hash.strip('0')


def topic_fact_name(topic):
    return f'graph.topic:{topic}'


def collect(source, topics, now=0.0, full=False):
    """Facts for ``topics``; ``full`` adds the graph-wide nodes and legacy count."""
    facts = {}
    for topic in topics:
        pubs, subs = source.endpoints(topic)
        facts[topic_fact_name(topic)] = Fact({'pub': tuple(pubs), 'sub': tuple(subs)}, topic, time=now)
    if full:
        facts['graph.nodes'] = Fact(list(source.node_names()), time=now)
        legacy = [ep for topic in source.topics() for side in source.endpoints(topic)
                  for ep in side if ep.legacy]
        facts['graph.legacy_endpoints'] = Fact(
            len(legacy), legacy[0].node if legacy else '',
            {'nodes': sorted({ep.node for ep in legacy})}, now)
    return facts
