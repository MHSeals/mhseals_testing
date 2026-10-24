"""Other nodes' parameters: a one-shot read, and the client for hot-applying Nav2 settings.

``get_parameters`` is synchronous and safe to call while the suite's executor spins on its
own thread: the reply is awaited on an event, never by spinning here. ``Params`` calls run on
the RosIO executor thread and wait on futures, like ros/nav2.py. Atomic sets matter: plain
SetParameters applies key by key, so MPPI would reset once per key. ROS message types are
imported where they are used, so ``collect`` stays importable without ROS.
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


# ParameterType value -> rclpy Parameter.Type name, for the types a Nav2 tuning value can have.
KINDS = {1: 'BOOL', 2: 'INTEGER', 3: 'DOUBLE', 4: 'STRING', 6: 'BOOL_ARRAY', 7: 'INTEGER_ARRAY',
         8: 'DOUBLE_ARRAY', 9: 'STRING_ARRAY'}
ELEMENT = {'BOOL_ARRAY': 'BOOL', 'INTEGER_ARRAY': 'INTEGER', 'DOUBLE_ARRAY': 'DOUBLE',
           'STRING_ARRAY': 'STRING'}


def coerce(value, kind):
    """Return ``value`` as the declared type ``kind`` (a KINDS name): an int reaches a double.

    Raises ValueError when the value cannot be that type without changing its meaning.
    """
    if kind in ELEMENT:
        if not isinstance(value, (list, tuple)):
            raise ValueError(f'expected a list for {kind.lower()}, got {value!r}')
        return [coerce(item, ELEMENT[kind]) for item in value]
    number = isinstance(value, (int, float)) and not isinstance(value, bool)
    if kind == 'BOOL' and isinstance(value, bool):
        return value
    if kind == 'STRING' and isinstance(value, str):
        return value
    if kind == 'DOUBLE' and number:
        return float(value)
    if kind == 'INTEGER' and number and float(value) == int(value):
        return int(value)
    raise ValueError(f'{value!r} is not a valid {kind.lower()}')


MANAGER = 'lifecycle_manager_navigation'
COMMANDS = ('startup', 'reset')   # ManageLifecycleNodes.Request.STARTUP / RESET


class Params:
    def __init__(self, ros, manager=MANAGER):
        self.node, self.manager = ros.node, manager
        self._clients = {}

    def _client(self, kind, service, timeout):
        client = self._clients.get((kind, service))
        if client is None:
            client = self._clients[kind, service] = self.node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'{service} is not available')
        return client

    def get(self, node, names, timeout=3.0):
        """{name: value} for the names the node has set (unset ones are left out)."""
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import Parameter
        from mhseals_testing.ros.nav2 import wait
        client = self._client(GetParameters, f'{node}/get_parameters', timeout)
        reply = wait(client.call_async(GetParameters.Request(names=list(names))), timeout, f'{node} get')
        return {name: plain(Parameter.from_parameter_msg(_msg(name, value)).value)
                for name, value in zip(names, reply.values) if value.type != ParameterType.PARAMETER_NOT_SET}

    def declared_types(self, node, names, timeout=3.0):
        """Return {name: KINDS name} from describe_parameters; undeclared names are left out."""
        from rcl_interfaces.srv import DescribeParameters
        from mhseals_testing.ros.nav2 import wait
        client = self._client(DescribeParameters, f'{node}/describe_parameters', timeout)
        request = DescribeParameters.Request(names=list(names))
        reply = wait(client.call_async(request), timeout, f'{node} describe')
        return {name: KINDS[d.type] for name, d in zip(names, reply.descriptors)
                if d.type in KINDS}

    def list_parameters(self, node, timeout=3.0):
        """Every parameter name the node declares."""
        from rcl_interfaces.srv import ListParameters
        from mhseals_testing.ros.nav2 import wait
        client = self._client(ListParameters, f'{node}/list_parameters', timeout)
        request = ListParameters.Request(prefixes=[], depth=ListParameters.Request.DEPTH_RECURSIVE)
        return set(wait(client.call_async(request), timeout, f'{node} list').result.names)

    def set_atomically(self, node, values, timeout=5.0):
        """One request for all ``values``; (successful, reason). Nothing applies on failure.

        Each value is coerced to the type the node declares (an int reaches a double parameter);
        ValueError, before anything is sent, when one cannot be or is not declared.
        """
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        from mhseals_testing.ros.nav2 import wait
        kinds = self.declared_types(node, list(values), timeout)
        missing = [name for name in values if name not in kinds]
        if missing:
            raise ValueError(f'{node} does not declare {", ".join(missing)}')
        client = self._client(SetParametersAtomically, f'{node}/set_parameters_atomically', timeout)
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, Parameter.Type[kinds[name]],
                      coerce(value, kinds[name])).to_parameter_msg()
            for name, value in values.items()])
        result = wait(client.call_async(request), timeout, f'{node} set').result
        return result.successful, result.reason

    def manage_nodes(self, command, timeout=60.0):
        """Run 'reset' or 'startup' on every managed server; True when the manager reports success."""
        from nav2_msgs.srv import ManageLifecycleNodes
        from mhseals_testing.ros.nav2 import wait
        if command not in COMMANDS:
            raise ValueError(f'unknown lifecycle command {command!r}')
        code = getattr(ManageLifecycleNodes.Request, command.upper())
        client = self._client(ManageLifecycleNodes, f'/{self.manager}/manage_nodes', timeout)
        return wait(client.call_async(ManageLifecycleNodes.Request(command=code)),
                    timeout, f'{command}').success

    def is_active(self, timeout=3.0):
        """Whether the lifecycle manager reports all servers active."""
        from std_srvs.srv import Trigger
        from mhseals_testing.ros.nav2 import wait
        client = self._client(Trigger, f'/{self.manager}/is_active', timeout)
        return wait(client.call_async(Trigger.Request()), timeout, 'is_active').success


def plain(value):
    """Return lists for the array.array (or numpy) values that rclpy hands back."""
    return value.tolist() if hasattr(value, 'tolist') else value


def _msg(name, value):
    from rcl_interfaces.msg import Parameter as ParameterMsg
    return ParameterMsg(name=name, value=value)
