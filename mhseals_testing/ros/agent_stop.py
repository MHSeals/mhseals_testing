"""``boat_trial stop``'s backup path: ask the command mux for a hold over ROS (spec agent-control S11).

Used when the TUI did not answer within a second, or no TUI runs. It only requests a hold
(``cause agent_stop``); nothing here releases a hold or arms.
"""

import json
import time

from mhseals_testing.ros.plant import MUX_REQUEST_TOPIC

MATCH_S = 2.0
REPEATS = 3


def hold(who, match_s=MATCH_S):
    """Publish the hold request; True when the mux (a subscriber) was there to take it."""
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('boat_trial_agent_stop')
    try:
        publisher = node.create_publisher(String, MUX_REQUEST_TOPIC, 10)
        deadline = time.monotonic() + match_s
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not publisher.get_subscription_count():
            return False
        message = String(data=json.dumps({'op': 'hold', 'cause': 'agent_stop', 'by': who}))
        for _ in range(REPEATS):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        return True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
