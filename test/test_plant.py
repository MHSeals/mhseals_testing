import json
from types import SimpleNamespace

import pytest

pytest.importorskip('geometry_msgs')

from mhseals_testing.health.model import Fact
from mhseals_testing.health.probes.graph import Endpoint, topic_fact_name
from mhseals_testing.ros.plant import Plant


class Node:
    """Records which publishers exist, as the graph would show them to a neighbour."""

    def __init__(self):
        self.live = []

    def create_publisher(self, kind, topic, depth):
        publisher = SimpleNamespace(topic=topic, sent=[], publish=lambda m: publisher.sent.append(m))
        self.live.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        self.live.remove(publisher)


class Ros:
    def __init__(self, node, name):
        self.node, self.name, self.peers = node, name, []

    def graph_facts(self, topics):
        """Every live publisher on the topic, ours tagged own, as the graph probe reports them."""
        pubs = [Endpoint(ros.name, own=ros is self) for ros in [self, *self.peers] if ros.node.live]
        return {topic_fact_name(t): Fact({'pub': tuple(pubs), 'sub': ()}) for t in topics}

    def publishers(self, topic):
        return [peer.name for peer in self.peers
                if any(p.topic == topic for p in peer.node.live)]


PROFILE = SimpleNamespace(command_type='twist', command_topic='/cmd_vel',
                          command_limit=(1.0, 1.0, 1.0), base_frame='base_link',
                          manual_topic='/cmd_vel/manual', muxed=True)


def pair():
    one, two = Ros(Node(), 'a'), Ros(Node(), 'b')
    one.peers, two.peers = [two], [one]
    return Plant(one, PROFILE), Plant(two, PROFILE)


def test_idle_plants_do_not_conflict_and_neutral_stays_silent():
    armed, idle = pair()
    armed.engage()
    assert idle.conflicts() == ['/cmd_vel also published by a']
    assert armed.conflicts() == []  # the idle suite publishes nothing
    idle.neutral()
    assert armed.conflicts() == []


def test_release_withdraws_the_publisher():
    armed, idle = pair()
    armed.engage()
    armed.release()
    assert idle.conflicts() == []
    armed.release()  # twice is harmless


def test_keys_publish_on_the_manual_topic_and_space_asks_the_mux_to_hold():
    plant, _ = pair()
    assert [p.topic for p in plant.ros.node.live] == ['/command_mux/request']  # before any hold
    plant.hold('keys')
    plant.hold('keys')
    topics = [p.topic for p in plant.ros.node.live]
    assert topics == ['/command_mux/request', '/cmd_vel/manual']
    request = plant.request_publisher.sent[0].data
    assert json.loads(request) == {'op': 'hold', 'cause': 'keys', 'by': 'tui'}


def test_unmuxed_plants_send_no_hold_request():
    plant = Plant(Ros(Node(), 'a'), SimpleNamespace(**dict(vars(PROFILE), muxed=False)))
    plant.hold()
    assert plant.request_publisher is None


def test_procedures_use_the_auto_topic_and_keys_and_space_the_manual_one():
    plant, _ = pair()
    plant.publish((0.5, 0.0, 0.0))  # a Runner procedure
    assert [p.topic for p in plant.ros.node.live if p.sent] == ['/cmd_vel']
    plant.publish((0.2, 0.0, 0.0), manual=True)  # keyboard drive
    plant.hold('keys')  # Space
    by_topic = {p.topic: p.sent for p in plant.ros.node.live}
    assert len(by_topic['/cmd_vel']) == 1 and by_topic['/cmd_vel'][0].linear.x == 0.5
    manual = by_topic['/cmd_vel/manual']
    assert [m.linear.x for m in manual] == [0.2, 0.0, 0.0, 0.0]  # drive, then Space neutral x3
    assert len(by_topic['/command_mux/request']) == 1
    plant.neutral()
    plant.release()
    assert [p.topic for p in plant.ros.node.live] == ['/command_mux/request']
