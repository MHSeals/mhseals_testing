"""Flight controller and battery facts from MAVROS, when the profile has an FCU."""

from rclpy.qos import qos_profile_sensor_data

from mhseals_testing.analysis import power
from mhseals_testing.health import probes


def subscribe_fcu(ros, monitor, facts, battery=None):
    """Publish /mavros/state and /mavros/battery as health facts; no-op without mavros_msgs."""
    try:
        from mavros_msgs.msg import State
        from sensor_msgs.msg import BatteryState
    except ImportError:
        monitor.advise('flight controller', 'N/A', 'mavros_msgs not installed')
        return

    def state(message):
        facts.publish('fcu', probes.fcu_facts(message.connected, message.armed, message.mode))

    tracker = power.Power(battery)

    def battery(message):
        sample = power.from_mavros(message, ros.node.get_clock().now().nanoseconds * 1e-9)
        # None (no pack, 65.535 V) clears the facts so the popup never shows a stale pack
        ros.power_facts = None if sample is None else tracker.update(sample)    # read by the Power popup
        cells = None if sample is None else power.pack_cells(ros.power_facts, tracker.chemistry)
        facts.publish('battery', probes.battery_facts(
            message.voltage, message.percentage, message.current, cells))

    ros.node.create_subscription(State, '/mavros/state', state, 10)
    ros.node.create_subscription(BatteryState, '/mavros/battery', battery,
                                 qos_profile_sensor_data)
