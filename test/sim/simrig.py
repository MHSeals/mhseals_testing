"""Helpers for the sim tests: find the rig, run hold_still or perception sessions, read bags.

The rig lives outside the repos (``$SIMRIG``, normally /tmp/wt/simrig). Everything here is
import-safe without ROS: rosbag2_py is only imported by ``read_odometry``.
"""
import json
import math
import os
import shlex
import subprocess
import uuid
from pathlib import Path

import numpy as np
import yaml

PERCEPTION = Path(__file__).with_name('perception_sim.yaml')
RUN_RECORD = 'simrig_run.json'   # the session's DDS domain and the draw that chose it


def pick_domain():
    """(domain, seed): a DDS domain in the network rule's 200..228 (run_sim.sh refuses > 232)."""
    seed = uuid.uuid4().int
    return 200 + seed % 29, seed


def run_rig(cmd, out, timeout, domain=None, seed=None):
    """Run the rig, then write the domain and seed into the session dir, also when the run fails."""
    try:
        subprocess.run(cmd, check=True, timeout=timeout)
    finally:
        Path(out).mkdir(parents=True, exist_ok=True)
        (Path(out) / RUN_RECORD).write_text(json.dumps({'domain': domain, 'seed': seed}) + '\n')
    return Path(out)


def rig_dir(env=None):
    """The rig directory named by $SIMRIG when it has run_sim.sh, else None."""
    env = os.environ if env is None else env
    path = Path(env.get('SIMRIG', ''))
    return path if env.get('SIMRIG') and (path / 'run_sim.sh').is_file() else None


def run_hold_still(rig, out, seconds=45, domain=None, seed=None):
    """Record a neutral boat (no drive) for ``seconds`` into ``out``.

    No ``--smoke``: its gates need thruster frames, which a neutral session lacks.
    """
    cmd = [str(rig / 'run_sim.sh'), '--no-drive', '-d', str(seconds), '-o', str(out)]
    if domain:
        cmd += ['--domain', str(domain), '--port', str(10000 + int(domain))]
    return run_rig(cmd, out, seconds + 300, domain, seed)


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


def perception_command(mode, case, cfg):
    """Build the --run script: tracker, sim_backend and a bag, then emulate/case, the drive."""
    share = '"$(ros2 pkg prefix mhseals_nav)/share/mhseals_nav/config"'
    loops = math.ceil(cfg['seconds'] / sum(row[0] for row in cfg['drive']))
    return '\n'.join([
        f'share={share}',
        'ros2 bag record -o "$SIMRIG_DIR/perception" --topics ' + ' '.join(cfg['record']) + ' &',
        'ros2 run mhseals_nav object_tracker --ros-args --params-file "$share/tracker.yaml"'
        ' -p use_sim_time:=true &',
        'ros2 run mhseals_nav sim_backend --ros-args --params-file "$share/sim_backend.yaml"'
        ' -p use_sim_time:=true -p course_truth:="$SIMRIG_DIR/course_truth.json" &',
        f'for _ in $(seq {cfg["node_wait_s"]}); do'
        ' ros2 node list | grep -qx /sim_backend && break; sleep 1; done',
        f'ros2 param set /sim_backend emulate {shlex.quote(mode)} || exit 1',
        f'ros2 param set /sim_backend case {shlex.quote(case)} || exit 1',
        f'python3 "$SIMRIG_DIR/drive.py" {cfg["command_topic"]} '
        + shlex.quote(json.dumps(cfg['drive'] * loops)),
        'wait'])


def run_perception_case(rig, out, mode, case='metamorphic', seconds=300, domain=None,
                        overlay=None, seed=None):
    """Record one sim_backend ``mode`` session running ``case`` into ``out`` (bag: perception/).

    ``overlay`` is a container install with the perception-v2 mhseals_nav when the rig's
    base install predates it.
    """
    cfg = dict(yaml.safe_load(PERCEPTION.read_text()), seconds=seconds)
    cmd = [str(rig / 'run_sim.sh'), '--config', str(rig / cfg['rig_config']),
           '--run', perception_command(mode, case, cfg), '-d', str(seconds), '-o', str(out)]
    if domain:
        cmd += ['--domain', str(domain), '--port', str(10000 + int(domain))]
    if overlay:
        cmd += ['--overlay', str(overlay)]
    return run_rig(cmd, out, seconds + 300, domain, seed)


def read_bag(path, topics):
    """{topic: [message, ...]} for ``topics`` in the bag at ``path``, in bag order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {topic: [] for topic in topics}
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name in out:
            out[name].append(deserialize_message(data, types[name]))
    return out
