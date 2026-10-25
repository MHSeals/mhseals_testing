"""host_diagnostic publishes, then ends cleanly when launch shuts rclpy down. No ROS: rclpy and
diagnostic_msgs are stubs. On Jazzy, the SIGINT/SIGTERM that launch sends shuts the context down
and rclpy.spin raises ExternalShutdownException; uncaught, the node died with a traceback in spin."""

import socket
import sys
from types import ModuleType, SimpleNamespace

from mhseals_testing.ros import host_diagnostic


class ExternalShutdownException(Exception):
    pass


def test_publishes_then_survives_the_launch_shutdown(monkeypatch):
    monkeypatch.setattr(socket, 'gethostname', lambda: 'orin')
    published, timers, seen = [], [], []
    node = SimpleNamespace(
        create_publisher=lambda *a: SimpleNamespace(publish=published.append),
        declare_parameter=lambda name, value: SimpleNamespace(value=value),
        create_timer=lambda period, callback: timers.append(callback),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp')),
        destroy_node=lambda: seen.append('destroyed'))

    def spin(_):
        timers[0]()
        raise ExternalShutdownException

    rclpy = ModuleType('rclpy')
    rclpy.init, rclpy.create_node, rclpy.spin = (lambda args=None: None), (lambda name: node), spin
    rclpy.try_shutdown = lambda: seen.append('shutdown')
    executors = ModuleType('rclpy.executors')
    executors.ExternalShutdownException = ExternalShutdownException
    msg = ModuleType('diagnostic_msgs.msg')
    msg.DiagnosticStatus = lambda **fields: SimpleNamespace(**fields)
    msg.DiagnosticStatus.OK = b'\x00'
    msg.DiagnosticArray = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None), status=[])
    msg.KeyValue = lambda key, value: (key, value)
    for name, module in (('rclpy', rclpy), ('rclpy.executors', executors),
                         ('diagnostic_msgs', ModuleType('diagnostic_msgs')), ('diagnostic_msgs.msg', msg)):
        monkeypatch.setitem(sys.modules, name, module)
    host_diagnostic.main([])
    [status] = published[0].status
    assert status.level == b'\x00' and status.hardware_id == 'host:orin'
    assert seen == ['destroyed', 'shutdown']
