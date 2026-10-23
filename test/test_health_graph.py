"""The graph probe, the endpoints and nodes kinds, and the shipped ROS graph checks."""

from types import SimpleNamespace

import pytest

from mhseals_testing.health import registry
from mhseals_testing.health.config import load, lint
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.probes.graph import Endpoint, collect, topic_fact_name
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.graph import RclpyGraph

registry.discover()
ZERO, REAL = '00' * 32, 'ab' * 32
RELIABLE = {'reliability': 'reliable', 'durability': 'volatile', 'lifespan': 0.0}
BEST_EFFORT = {**RELIABLE, 'reliability': 'best_effort'}


class FakeGraph:
    def __init__(self, nodes, topics):
        self.nodes, self.by_topic = nodes, topics

    def endpoints(self, topic):
        return self.by_topic.get(topic, ((), ()))

    def node_names(self):
        return self.nodes

    def topics(self):
        return list(self.by_topic)


def ep(node, hash=REAL, qos=RELIABLE, type='geometry_msgs/msg/Twist', own=False):
    return Endpoint(node, type, hash, qos, own=own)


def checks(profile='boat'):
    return {s.id: s for s in load(user=False, profile=load_profile(profile)).specs}


def run(spec_id, graph, topics=()):
    spec = checks()[spec_id]
    topics = topics or [getattr(spec.params, 'topic', None)] * hasattr(spec.params, 'topic')
    facts = collect(graph, topics, full=True)
    probes = {'graph': (0.0, None)}
    times = iter([0.0, 10.0])               # long enough to pass any raise_after
    engine = Engine([spec], clock=lambda: next(times))
    engine.tick(facts, probes)
    return engine.tick(facts, probes).results[spec_id]


def test_shipped_graph_checks_lint_clean():
    assert lint(load(user=False, profile=load_profile('boat')).specs) == []


def test_clean_graph_is_ok():
    graph = FakeGraph(['/a', '/b'], {'/cmd_vel': ([ep('/a')], [ep('/b')])})
    assert run('cmd.owner', graph).status == 'ok'
    assert run('node.unique', graph).status == 'ok'
    assert run('graph.legacy', graph, topics=['/cmd_vel']).status == 'ok'


def test_duplicate_fqn_fails_node_unique_():
    graph = FakeGraph(['/mavros', '/mavros', '/x'], {})
    result = run('node.unique', graph)
    assert result.status == 'fail' and 'duplicate node /mavros' in result.detail


def test_second_publisher_fails_cmd_owner_and_names_it():
    graph = FakeGraph([], {'/cmd_vel': ([ep('/tui1', own=True), ep('/tui2')], [])})
    result = run('cmd.owner', graph)
    assert result.status == 'fail' and '2 publishers' in result.detail
    assert result.data['nodes'] == '/tui1, /tui2'


def test_zero_type_hash_is_legacy_and_unknown_hash_is_not():
    graph = FakeGraph(['/foxy'], {'/t': ([ep('/foxy', hash=ZERO)], [ep('/n', hash=None)])})
    result = run('graph.legacy', graph)
    assert result.status == 'fail' and '1 endpoints without a type hash (first: /foxy)' in result.detail
    assert collect(graph, [], full=True)['graph.legacy_endpoints'].data == {'nodes': ['/foxy']}


def test_zero_hash_is_not_a_type_mismatch():
    graph = FakeGraph([], {'/detections': ([ep('/p', hash=ZERO, type='a/msg/X')], [ep('/s', type='a/msg/X')])})
    assert run('type.detections', graph).status == 'ok'
    other = FakeGraph([], {'/detections': ([ep('/p', type='a/msg/X')], [ep('/s', type='b/msg/Y')])})
    assert run('type.detections', other).status == 'warn'    # severity caps a fail at warn


def test_qos_mismatch_and_forbidden_lifespan():
    mismatch = FakeGraph([], {'/odom/local': ([ep('/ekf', qos=BEST_EFFORT)], [ep('/tui', qos=RELIABLE)])})
    assert 'best_effort cannot feed reliable /tui' in run('qos.odom', mismatch).detail
    unknown = {'reliability': 'unknown', 'durability': 'unknown', 'lifespan': 0.0}
    assert run('qos.odom', FakeGraph([], {'/odom/local': ([ep('/ekf', qos=unknown)], [ep('/tui')])})).status == 'ok'
    rtcm = FakeGraph([], {'/mavros/gps_rtk/send_rtcm': ([ep('/rtk', qos={**RELIABLE, 'lifespan': 1.0})], [])})
    assert 'sets lifespan 1 s' in run('qos.rtcm', rtcm).detail


def test_tui_single_counts_matching_nodes():
    assert run('tui.single', FakeGraph(['/mhseals_testing_h_1'], {})).status == 'ok'
    two = FakeGraph(['/mhseals_testing_h_1', '/mhseals_testing_h_2'], {})
    assert run('tui.single', two).status == 'fail'


def test_missing_graph_sample_is_unknown():
    spec = checks()['cmd.owner']
    assert Engine([spec], clock=lambda: 0.0).tick({}, {'graph': (0.0, None)}).results['cmd.owner'].status == 'unknown'


def test_rclpy_adapter_tags_own_endpoint_and_reads_hash_and_qos():
    def info(name, life=2_000_000_000, **extra):
        qos = SimpleNamespace(reliability=SimpleNamespace(name='RELIABLE'),
                              durability=SimpleNamespace(name='VOLATILE'),
                              lifespan=SimpleNamespace(nanoseconds=life))
        return SimpleNamespace(node_name=name, node_namespace='/', topic_type='t/msg/T',
                               endpoint_gid=b'\x01\x02', qos_profile=qos, **extra)

    node = SimpleNamespace(
        get_name=lambda: 'me', get_namespace=lambda: '/',
        get_publishers_info_by_topic=lambda t: [info('me', topic_type_hash=SimpleNamespace(value=bytes(32))),
                                                info('other'),
                                                info('inf', life=9223372036854775807)],
        get_subscriptions_info_by_topic=lambda t: [],
        get_node_names_and_namespaces=lambda: [('me', '/'), ('me', '/')],
        get_topic_names_and_types=lambda: [('/t', ['t/msg/T'])])
    pubs, _ = RclpyGraph(node).endpoints('/t')
    assert [(p.node, p.own, p.hash is None) for p in pubs] == [('/me', True, False), ('/other', False, True),
                                                                  ('/inf', False, True)]
    assert pubs[0].legacy and not pubs[1].legacy
    assert pubs[0].qos == {'reliability': 'reliable', 'durability': 'volatile', 'lifespan': 2.0}
    assert pubs[2].qos['lifespan'] == 0.0  # Duration.Infinite means no lifespan
    assert pubs[0].gid == '0102'
    assert RclpyGraph(node).node_names() == ['/me', '/me']
