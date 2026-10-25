"""RosIO.get_parameters feeds the Odometry popup: every parameter of a node, None when it is silent."""

from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.node import Node

from mhseals_testing.ros.node import RosIO


def test_get_parameters_reads_a_live_node_and_none_for_a_silent_one():
    rclpy.init()
    io = server = None
    try:
        profile = SimpleNamespace(sensors=[], use_sim_time=False, base_frame='base_link')
        io = RosIO(profile, SimpleNamespace(event=lambda text: None))
        server = Node('ekf_params_probe')
        server.declare_parameter('two_d_mode', True)
        server.declare_parameter('frequency', 30.0)
        io.executor.add_node(server)
        params = io.get_parameters('ekf_params_probe', timeout=5.0)
        assert params['two_d_mode'] is True and params['frequency'] == 30.0
        assert io.get_parameters('no_such_ekf', timeout=0.2) is None
    finally:
        if server:
            io.executor.remove_node(server)
            server.destroy_node()
        if io:
            io.close()
        rclpy.shutdown()
