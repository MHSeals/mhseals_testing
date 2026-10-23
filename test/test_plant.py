from types import SimpleNamespace

import pytest

pytest.importorskip('geometry_msgs')

from mhseals_testing.ros.plant import Plant


class Node:
    """Records which publishers exist, as the graph would show them to a neighbour."""

    def __init__(self):
        self.live = []

    def create_publisher(self, kind, topic, depth):
        publisher = SimpleNamespace(topic=topic, sent=[], publish=lambda m: None)
        self.live.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        self.live.remove(publisher)


class Ros:
    def __init__(self, node, name):
        self.node, self.name, self.peers = node, name, []

    def publishers(self, topic):
        return [peer.name for peer in self.peers if peer.node.live]


PROFILE = SimpleNamespace(command_type='twist', command_topic='/cmd_vel',
                          command_limit=(1.0, 1.0, 1.0), base_frame='base_link')


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
