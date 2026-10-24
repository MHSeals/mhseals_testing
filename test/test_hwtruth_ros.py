"""The real thruster_pwm_node (topic backend, throwaway domain) against App.apply_hardware.

Container only: needs rclpy, an explicit ROS_DOMAIN_ID and localhost-only discovery."""

import json
import os
import signal
import subprocess
import sys
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
pytestmark = pytest.mark.skipif(
    not os.environ.get('ROS_DOMAIN_ID') or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='set ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 (throwaway domain)')

from rclpy.executors import SingleThreadedExecutor                       # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile                       # noqa: E402
from std_msgs.msg import String                                          # noqa: E402

from mhseals_hardware.configuration import load_defaults, save_config    # noqa: E402
from mhseals_testing.ui.popups import config as config_popup             # noqa: E402

from test_hwtruth import edit, rig, row                                  # noqa: E402,F401


class Graph:
    """The two RosIO members apply_hardware reads, over a real rclpy node."""
    fault = ''

    def __init__(self, node):
        self.node, self.driver_status = node, None
        node.create_subscription(String, '/thruster_pwm_node/status', self._status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _status(self, message):
        self.driver_status = json.loads(message.data)

    def nodes(self):
        return [f"{ns.rstrip('/')}/{name}" for name, ns in self.node.get_node_names_and_namespaces()]

    def emit(self, *args, **kwargs):
        pass


def test_real_driver_confirms_a_live_edit_and_refuses_a_restart_key(rig, tmp_path):
    app, operator = rig
    save_config(dict(load_defaults(), output_backend='topic'), app.paths.hardware)
    rclpy.init()
    node = rclpy.create_node('hwtruth_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    driver = subprocess.Popen(
        [sys.executable, '-m', 'mhseals_hardware.thruster_pwm_node', '--ros-args',
         '-p', f'config_file:={app.paths.hardware}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        app.ros = Graph(node)
        app.arming.armed = True
        deadline = time.monotonic() + 15
        while not (app.ros.driver_status and app.ros.nodes().count('/thruster_pwm_node')):
            assert time.monotonic() < deadline
            time.sleep(0.1)
        edit(operator, command_timeout=0.9)
        assert app.watcher.reload('hardware').ok
        assert 'live until re-arm' in row(app)
        edit(operator, command_timeout=0.8, channel_map=[4, 3, 2, 1])
        change = app.watcher.reload('hardware')
        assert not change.ok and 'channel_map' in change.error + str(app.ros.driver_status)
        assert row(app).startswith(config_popup.GLYPH['bad'])
    finally:
        driver.send_signal(signal.SIGINT)
        try:
            driver.wait(10)
        except subprocess.TimeoutExpired:
            driver.kill()
        executor.shutdown()
        node.destroy_node()
        thread.join(2)
