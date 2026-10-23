"""Flight controller advisories from MAVROS, when the profile has an FCU."""

from rclpy.qos import qos_profile_sensor_data

from mhseals_testing.health import legacy as health


def subscribe_fcu(ros, monitor):
    """Feed /mavros/state and /mavros/battery into advisories; no-op without mavros_msgs."""
    try:
        from mavros_msgs.msg import State
        from sensor_msgs.msg import BatteryState
    except ImportError:
        monitor.advise('flight controller', 'N/A', 'mavros_msgs not installed')
        return

    def state(message):
        monitor.advise('flight controller',
                       *health.flight_controller(message.connected, message.armed, message.mode))

    def battery(message):
        monitor.advise('battery', *health.battery(
            message.voltage, message.percentage, message.current))

    ros.node.create_subscription(State, '/mavros/state', state, 10)
    ros.node.create_subscription(BatteryState, '/mavros/battery', battery,
                                 qos_profile_sensor_data)
