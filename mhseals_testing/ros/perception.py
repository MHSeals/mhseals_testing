"""Publish ``perc.<source>.*`` health facts from /perception/status and the ZED /diagnostics.

Only DiagnosticArray topics are read (perception-v2.md section 7.2): no detections or
observations cross the network for health. ``observe_status`` and ``observe_diagnostics``
are pure (duck-typed messages, injected probe) and carry the tests.
"""

import time

from mhseals_testing.perception import clock
from mhseals_testing.perception.probe import Probe

STATUS = '/perception/status'
DIAGNOSTICS = '/diagnostics'
PERIOD = 1.0      # s between fact publishes


def source_name(profile):
    """The perception.yaml source a profile's backend is."""
    return 'sim' if profile.use_sim_time else 'zed_native'


def observe_status(probe, message, info=None):
    """Feed one /perception/status array; ``info`` is rclpy's MessageInfo (ns stamps).

    The clock offset is the DDS source_timestamp (the publisher's wall clock, the Orin's
    on the boat) against our receive time.
    """
    offset = None
    if info:
        offset = clock.offset(info.get('source_timestamp', 0) * 1e-9,
                              info.get('received_timestamp', 0) * 1e-9)
    for status in message.status:
        probe.status(status, offset)


def observe_diagnostics(probe, message):
    for status in message.status:
        probe.diagnostic(status)


def subscribe_detections(ros, facts, profile):
    """Subscribe the two status topics on ``ros.node`` and publish the facts once a second."""
    from diagnostic_msgs.msg import DiagnosticArray
    probe = Probe(source_name(profile), time.monotonic)
    ros.node.create_subscription(
        DiagnosticArray, STATUS, lambda message, info: observe_status(probe, message, info), 10)
    ros.node.create_subscription(
        DiagnosticArray, DIAGNOSTICS, lambda message: observe_diagnostics(probe, message), 10)
    ros.node.create_timer(PERIOD, lambda: facts.publish('perception', probe.facts()))
    return probe
