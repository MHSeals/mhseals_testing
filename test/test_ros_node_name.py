"""Two suite instances must not share a node name, or the command-topic interlock is blind."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from mhseals_testing.ros.node import EVENTS_TOPIC, RosIO, node_name


def test_same_pid_on_two_hosts_gets_two_names():
    assert node_name(1, 'odroid') != node_name(1, 'jetson.local')


def test_node_name_is_per_process():
    assert node_name(1) != node_name(2)


def infos(*pairs):
    """Endpoint infos for (node name, gid prefix byte) pairs; the gid's tail is the endpoint."""
    return [SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=[prefix] * 12 + [0, 0, 0, i])
            for i, (name, prefix) in enumerate(pairs)]


def fake_io(topics):
    get = lambda topic: topics.get(topic, [])
    fake = SimpleNamespace(
        _prefix=None, node=SimpleNamespace(get_name=lambda: node_name(1), get_publishers_info_by_topic=get,
                                           get_subscriptions_info_by_topic=get))
    fake._own_prefix = lambda: RosIO._own_prefix(fake)
    fake.endpoints = lambda topic, subscribers=False: RosIO.endpoints(fake, topic, subscribers)
    return fake


def test_second_instance_is_reported_as_a_publisher():
    fake = fake_io({'/cmd_vel': infos((node_name(1), 1), (node_name(2), 2)),
                    EVENTS_TOPIC: infos((node_name(1), 1))})
    assert RosIO.publishers(fake, '/cmd_vel') == ['/' + node_name(2)]


def test_instances_lists_other_suite_nodes_only():
    names = [(node_name(1), '/'), (node_name(2), '/'), ('thruster_pwm_node', '/')]
    fake = SimpleNamespace(node=SimpleNamespace(
        get_name=lambda: node_name(1), get_node_names_and_namespaces=lambda: names))
    assert RosIO.instances(fake) == [node_name(2)]


def test_emit_journals_before_publishing_and_keeps_the_dashboard_line_clean(tmp_path):
    from mhseals_testing.harness import journal
    path, order, shown = tmp_path / 'j.jsonl', [], []
    fake = SimpleNamespace(
        journal_path=path, recording=True, now=lambda: 1.0,
        node=SimpleNamespace(get_name=lambda: 'n'),
        events=SimpleNamespace(publish=lambda msg: order.append(('publish', journal.read(path)))),
        monitor=SimpleNamespace(event=shown.append), profile=None, stale_after=2.0)
    fake._ingest = lambda *a: RosIO._ingest(fake, *a)
    RosIO.emit(fake, 'marker', id=3, t_ns=7)
    [event] = journal.read(path)
    assert event['t_ns'] == 7 and event['rec'] is True
    assert order == [('publish', [event])]  # the journal line existed when publish ran
    [text] = shown
    assert text == 'marker {"id": 3}'


def test_own_endpoints_are_tagged_by_gid_not_dropped_or_matched_by_name():
    # A second suite that shares our node name is still another participant.
    fake = fake_io({'/cmd_vel': infos((node_name(1), 1), (node_name(1), 2)),
                    EVENTS_TOPIC: infos((node_name(1), 1))})
    found = RosIO.endpoints(fake, '/cmd_vel')
    assert [e['self'] for e in found] == [True, False]
    assert found[0]['gid'] == '01' * 12 + '00000000' and found[1]['gid'].startswith('02' * 12)
    assert RosIO.publishers(fake, '/cmd_vel') == ['/' + node_name(1)]


def test_ambiguous_participant_tags_nothing_as_self():
    fake = fake_io({'/cmd_vel': infos((node_name(1), 1)),
                    EVENTS_TOPIC: infos((node_name(1), 1), (node_name(1), 2))})
    assert [e['self'] for e in RosIO.endpoints(fake, '/cmd_vel')] == [False]
