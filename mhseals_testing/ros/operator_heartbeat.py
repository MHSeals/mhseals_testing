"""operator_heartbeat: /operator/heartbeat (std_msgs/String, the host name) for link_watchdog.

Runs on the operator's computer as the operator device profile's ``operator_heartbeat`` part
(mhseals_nav config/devices/operator.yaml). The boat's link_watchdog counts only beats naming one
of its ``operator_hosts`` and never the boat itself, so a stray copy on a boat computer cannot
hide a lost link; that filter lives there, not here.
"""

import socket
import sys

TOPIC = '/operator/heartbeat'
PERIOD_S = 0.5      # link_watchdog holds after max_age_s (3 s) without a beat


def bind(node, period=PERIOD_S):
    from std_msgs.msg import String
    pub = node.create_publisher(String, TOPIC, 1)
    msg = String(data=socket.gethostname())
    return node.create_timer(period, lambda: pub.publish(msg))


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = rclpy.create_node('operator_heartbeat')
    bind(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # launch's SIGINT/SIGTERM shuts the context down
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
