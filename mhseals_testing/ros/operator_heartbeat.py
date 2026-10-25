"""operator_heartbeat: /operator/heartbeat (std_msgs/String, the host name) for link_watchdog.

Runs on the operator's computer (``ros2 run mhseals_testing operator_heartbeat``). The boat's
link_watchdog holds the boat when the heartbeat stops, so this refuses to start on a boat host
(``boat_hosts`` in config/operator.yaml): a heartbeat from the boat would hide a lost link.
"""

from fnmatch import fnmatchcase
from pathlib import Path
import socket
import sys

from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'operator.yaml'
TOPIC = '/operator/heartbeat'


def settings(path=CONFIG):
    return load_strict(Path(path).read_text(), path) or {}


def on_boat_host(host, config):
    """Whether ``host`` matches a ``boat_hosts`` pattern; a list that cannot be read as one also refuses."""
    patterns = config.get('boat_hosts')
    if not isinstance(patterns, list) or not all(isinstance(p, str) for p in patterns):
        return True
    return any(fnmatchcase(host, p) for p in patterns)


def bind(node, period=0.5):
    from std_msgs.msg import String
    pub = node.create_publisher(String, TOPIC, 1)
    msg = String(data=socket.gethostname())
    return node.create_timer(period, lambda: pub.publish(msg))


def main(argv=None):
    host = socket.gethostname()
    try:
        config = settings()
    except (OSError, ValueError) as error:
        print(f'operator_heartbeat: cannot read {CONFIG}: {error}', file=sys.stderr)
        return 2
    if on_boat_host(host, config):
        print(f'operator_heartbeat: refusing to run on {host}: it is a boat host (boat_hosts in {CONFIG.name})',
              file=sys.stderr)
        return 2
    import rclpy
    rclpy.init(args=argv)
    node = rclpy.create_node('operator_heartbeat')
    bind(node, float(config.get('heartbeat_period_s', 0.5)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
