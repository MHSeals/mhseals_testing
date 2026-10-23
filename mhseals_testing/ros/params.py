"""Read another node's parameters through its get_parameters service.

Synchronous and safe to call while the suite's executor spins on its own
thread: the reply is awaited on an event, never by spinning here.
"""

import threading

# A parameter the target does not declare comes back as this ParameterType.
NOT_SET = 0


def collect(names, values, convert):
    """Map names to Python values, leaving out the ones the target does not declare."""
    return {name: convert(value) for name, value in zip(names, values)
            if value.type != NOT_SET}


def get_parameters(node, target, names, timeout=3.0):
    """Values of ``names`` on node ``target`` (e.g. '/thruster_pwm_node'); {} when unreachable."""
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    names = list(names)
    client = node.create_client(GetParameters, f'{target.rstrip("/")}/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return {}
        done = threading.Event()
        future = client.call_async(GetParameters.Request(names=names))
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout) or future.result() is None:
            return {}
        return collect(names, future.result().values, parameter_value_to_python)
    finally:
        node.destroy_client(client)
