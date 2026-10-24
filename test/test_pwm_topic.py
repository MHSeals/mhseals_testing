from types import SimpleNamespace

import pytest

from mhseals_testing.ros.pwm_readback import TopicReadback


class Node:
    def __init__(self):
        self.destroyed = []

    def create_subscription(self, kind, topic, callback, depth):
        self.kind, self.topic, self.callback, self.depth = kind, topic, callback, depth
        return 'subscription'

    def destroy_subscription(self, handle):
        self.destroyed.append(handle)


def test_configured_subscription_receipt_expiry_and_cleanup():
    node, now = Node(), [10.]
    feed = TopicReadback(node, '/rig/custom_pwm', clock=lambda: now[0], message_type=object)
    assert node.topic == '/rig/custom_pwm' and node.kind is object
    assert feed.observation().neutral(10, 1) is None
    node.callback(SimpleNamespace(data=[1500] * 4))
    observation = feed.observation()
    assert observation.neutral(10, 1) and observation.health(10, 1, True)[0] == 'OK'
    assert [o.identity for o in observation.outputs] == [f'/rig/custom_pwm#{i}' for i in range(1, 5)]
    now[0] = 12
    assert feed.observation() is observation and observation.receipt == 10
    assert observation.neutral(12, 1) is None
    node.callback(SimpleNamespace(data=[1600, 1500, 1500, 1500]))
    assert feed.observation().neutral(12, 1) is False
    feed.close()
    feed.close()
    node.callback(SimpleNamespace(data=[1500] * 4))
    assert feed.observation().receipt is None
    assert node.destroyed == ['subscription']  # fake has no publisher API


@pytest.mark.parametrize('pulses', [[], [1500] * 3, [1500] * 5, [1500., 1500, 1500, 1500],
                                    [True] * 4, [float('nan')] * 4, [float('inf')] * 4,
                                    [1000] * 4, [2000] * 4, None])
def test_invalid_message_revokes_cached_good_evidence(pulses):
    node = Node()
    feed = TopicReadback(node, '/pwm', clock=lambda: 10, message_type=object)
    node.callback(SimpleNamespace(data=[1500] * 4))
    assert feed.observation().neutral(10, 1)
    node.callback(SimpleNamespace(data=pulses))
    assert feed.observation().neutral(10, 1) is None


def test_no_implicit_default_topic():
    with pytest.raises(ValueError):
        TopicReadback(Node(), '', message_type=object)
