"""Helpers for the sim tests: find the rig, run a hold_still session, read its bag.

The rig lives outside the repos (``$SIMRIG``, normally /tmp/wt/simrig). Everything here is
import-safe without ROS: rosbag2_py is only imported by ``read_odometry``.
"""
import math
import os
import subprocess
from pathlib import Path

import numpy as np


def rig_dir(env=None):
    """The rig directory named by $SIMRIG when it has run_sim.sh, else None."""
    env = os.environ if env is None else env
    path = Path(env.get('SIMRIG', ''))
    return path if env.get('SIMRIG') and (path / 'run_sim.sh').is_file() else None


def run_hold_still(rig, out, seconds=45, domain=None):
    """Record a neutral boat (no drive) for ``seconds`` into ``out``.

    No ``--smoke``: its gates need thruster frames, which a neutral session lacks.
    """
    cmd = [str(rig / 'run_sim.sh'), '--no-drive', '-d', str(seconds), '-o', str(out)]
    if domain:
        cmd += ['--domain', str(domain), '--port', str(10000 + int(domain))]
    subprocess.run(cmd, check=True, timeout=seconds + 300)
    return Path(out)


def truth_current(truth):
    """World (E, N) current in m/s from course truth: ``currentToward`` is CCW from world +x."""
    return np.array([math.cos(truth['currentToward']), math.sin(truth['currentToward'])]) * truth['currentSpeed']


def world_arrays(t, quat, twist):
    """Estimator inputs from odometry rows: (t, vel E/N, compass heading, yaw rate).

    ``quat`` is (n, 4) x y z w, ``twist`` is (n, 3) body vx, vy, wz. The compass heading is
    clockwise from north, so it is pi/2 minus the ENU yaw.
    """
    x, y, z, w = np.asarray(quat, float).T
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    vx, vy, wz = np.asarray(twist, float).T
    vel = np.stack([vx * np.cos(yaw) - vy * np.sin(yaw), vx * np.sin(yaw) + vy * np.cos(yaw)], axis=1)
    return np.asarray(t, float), vel, math.pi / 2 - yaw, wz


def read_odometry(session, topic='/odom/mavros'):
    """Header-stamped rows of an Odometry topic in the session bag, as ``world_arrays`` inputs."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(session) / 'bag')),
                rosbag2_py.ConverterOptions('', ''))
    rows = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            m = deserialize_message(data, Odometry)
            q, v = m.pose.pose.orientation, m.twist.twist
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q.x, q.y, q.z, q.w,
                         v.linear.x, v.linear.y, v.angular.z))
    a = np.array(rows).reshape(-1, 8)
    return world_arrays(a[:, 0], a[:, 1:5], a[:, 5:8])
