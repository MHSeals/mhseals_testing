"""Adapt an rclpy node's graph API to the graph probe's source (see health/probes/graph.py)."""

from mhseals_testing.health.probes.graph import Endpoint


def _name(value):
    return getattr(value, 'name', str(value)).lower()


def _hash(info):
    """Hex type hash, or None when this rclpy does not report one (Humble)."""
    value = getattr(info, 'topic_type_hash', None)
    if value is None:
        return None
    return bytes(getattr(value, 'value', b'')).hex()


def endpoint(info, own):
    """An Endpoint from an rclpy TopicEndpointInfo; ``own`` is this node's FQN."""
    fqn = f'{info.node_namespace.rstrip("/")}/{info.node_name}'
    qos = getattr(info, 'qos_profile', None)
    lifespan = getattr(getattr(qos, 'lifespan', None), 'nanoseconds', 0)
    return Endpoint(
        fqn, info.topic_type, _hash(info),
        {'reliability': _name(getattr(qos, 'reliability', 'unknown')),
         'durability': _name(getattr(qos, 'durability', 'unknown')),
         'lifespan': lifespan / 1e9},
        bytes(getattr(info, 'endpoint_gid', b'')).hex(), fqn == own)


class RclpyGraph:
    """Reads publishers, subscriptions and nodes from ``node``."""

    def __init__(self, node):
        self.node = node

    def _own(self):
        return f'{self.node.get_namespace().rstrip("/")}/{self.node.get_name()}'

    def endpoints(self, topic):
        own = self._own()
        return (tuple(endpoint(i, own) for i in self.node.get_publishers_info_by_topic(topic)),
                tuple(endpoint(i, own) for i in self.node.get_subscriptions_info_by_topic(topic)))

    def node_names(self):
        return [f'{namespace.rstrip("/")}/{name}'
                for name, namespace in self.node.get_node_names_and_namespaces()]

    def topics(self):
        return [name for name, _ in self.node.get_topic_names_and_types()]
