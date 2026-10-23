"""Flight controller and battery facts from MAVROS, when the profile has an FCU."""

from rclpy.qos import qos_profile_sensor_data

from mhseals_testing.health import probes


def subscribe_fcu(ros, monitor, facts, cells=None):
    """Publish /mavros/state and /mavros/battery as health facts; no-op without mavros_msgs."""
    try:
        from mavros_msgs.msg import State
        from sensor_msgs.msg import BatteryState
    except ImportError:
        monitor.advise('flight controller', 'N/A', 'mavros_msgs not installed')
        return

    def state(message):
        facts.publish('fcu', probes.fcu_facts(message.connected, message.armed, message.mode))

    def battery(message):
        facts.publish('battery', probes.battery_facts(
            message.voltage, message.percentage, message.current, cells))

    ros.node.create_subscription(State, '/mavros/state', state, 10)
    ros.node.create_subscription(BatteryState, '/mavros/battery', battery,
                                 qos_profile_sensor_data)
