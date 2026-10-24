"""Publish ``perc.<source>.*`` health facts from the canonical /detections topic.

The ROS wiring is ``subscribe_detections``; ``observe_message`` is pure (duck-typed
message, injected probe) and carries the tests. Time comes from the node clock,
the timebase of the header stamps (sim time in the simulator).
"""

from mhseals_testing.perception.probe import Detection, Probe

TOPIC = '/detections'
PERIOD = 1.0      # s between fact publishes; the probe's own windows are longer


def source_name(profile):
    """The perception.yaml source a profile's detections belong to."""
    return 'sim' if profile.use_sim_time else 'zed_native'


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def observe_message(probe, message):
    """Feed one vision_msgs/Detection3DArray (possibly empty) to ``probe``."""
    found = []
    for detection in message.detections:
        if detection.results:
            best = detection.results[0].hypothesis
            found.append(Detection(best.class_id, float(best.score)))
    probe.observe(stamp_seconds(message.header), found, message.header.frame_id)


def subscribe_detections(ros, facts, profile):
    """Subscribe /detections on ``ros.node`` and publish the probe's facts once a second."""
    from rclpy.qos import qos_profile_sensor_data
    from vision_msgs.msg import Detection3DArray
    probe = Probe(source_name(profile), ros.now)
    ros.node.create_subscription(
        Detection3DArray, TOPIC, lambda message: observe_message(probe, message),
        qos_profile_sensor_data)
    ros.node.create_timer(PERIOD, lambda: facts.publish('perception', probe.facts()))
    return probe
