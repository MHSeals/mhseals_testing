"""boat_agent fcu rates: topic rates, receive-gap p99, the FCU load and "Main loop slow" over a short window.

Read-only, on the TUI's own node: subscriptions live only for the window. The FCU load is MAVROS's
``sys_status`` (SYS_STATUS load, in 0.1 %); the slow-loop count is STATUSTEXT lines containing "Main loop slow".
"""

import time

from mhseals_testing.harness.agent_fcu import stats

SYS_STATUS = ('/mavros/sys_status', 'mavros_msgs/msg/SysStatus')
STATUSTEXT = ('/mavros/statustext/recv', 'mavros_msgs/msg/StatusText')
SLOW = 'main loop slow'


def measure(node, topics, window_s, clock=time.monotonic, sleep=time.sleep):
    """``{topics: {name: stats}, load_pct, load_max_pct, main_loop_slow, statustext}`` over ``window_s``."""
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    seen, loads, texts, subs = {name: [] for name in topics}, [], [], []

    def stamp(name):
        return lambda _message: seen[name].append(clock())
    try:
        for name, kind in topics.items():
            subs.append(node.create_subscription(get_message(kind), name, stamp(name), qos_profile_sensor_data))
        subs.append(node.create_subscription(get_message(SYS_STATUS[1]), SYS_STATUS[0],
                                             lambda m: loads.append(m.load / 10.0), qos_profile_sensor_data))
        subs.append(node.create_subscription(get_message(STATUSTEXT[1]), STATUSTEXT[0],
                                             lambda m: texts.append(m.text), 10))
        sleep(window_s)
    finally:
        for sub in subs:
            node.destroy_subscription(sub)
    return {'topics': {name: stats(times) for name, times in seen.items()},
            'load_pct': round(sum(loads) / len(loads), 1) if loads else None,
            'load_max_pct': max(loads) if loads else None,
            'main_loop_slow': sum(SLOW in t.lower() for t in texts),
            'statustext': texts[-5:]}
