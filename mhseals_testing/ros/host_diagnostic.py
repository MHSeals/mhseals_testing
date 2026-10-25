"""host_diagnostic: the camera host's temperature, power mode and free disk on /diagnostics.

Runs on the Orin (``ros2 run mhseals_testing host_diagnostic``) and publishes one DiagnosticStatus
with ``hardware_id: host:<hostname>`` every 2 s, so blastoise reads the Orin without SSH.
Reads sysfs and the nvpmodel status only; ``nvpmodel``, ``jetson_clocks`` and the fan are
never touched. A missing source (a container without /var/lib/nvpmodel) is left out.
"""

import socket
import sys

PERIOD_S = 2.0


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    from mhseals_testing.health.probes.preflight import HOST_PREFIX, host_values

    rclpy.init(args=argv if argv is not None else sys.argv)
    node = rclpy.create_node('host_diagnostic')
    publisher = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
    host = socket.gethostname()
    data = node.declare_parameter('data_directory', '/').value

    def publish():
        values = host_values(data=data)
        message = DiagnosticArray()
        message.header.stamp = node.get_clock().now().to_msg()
        message.status = [DiagnosticStatus(
            level=DiagnosticStatus.OK, name='host_diagnostic', hardware_id=HOST_PREFIX + host,
            message=', '.join(f'{k} {v}' for k, v in values.items()),
            values=[KeyValue(key=k, value=v) for k, v in values.items()])]
        publisher.publish(message)

    node.create_timer(PERIOD_S, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # launch's SIGINT/SIGTERM shuts the context down
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
