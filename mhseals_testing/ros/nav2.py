"""A thin Nav2 client: lifecycle states, planning, navigation goals, costmap and map pose.

Calls run on the RosIO executor thread; these methods wait on futures, so the
TUI thread can call them directly.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# The servers mhseals_nav's navigation.launch.py brings up.
SERVERS = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
           'waypoint_follower', 'velocity_smoother')


def wait(future, timeout, what):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f'{what} timed out after {timeout:g}s')
        time.sleep(0.02)
    return future.result()


def pose_stamped(frame, x, y, yaw, stamp):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


class Goal:
    """A running navigation goal: poll ``done``, read ``status``, ``cancel`` on the way out."""

    def __init__(self, handle):
        self.handle = handle
        self.result = handle.get_result_async()

    @property
    def done(self):
        return self.result.done()

    @property
    def status(self):
        if not self.done:
            return 'RUNNING'
        return {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(self.result.result().status, 'UNKNOWN')

    def cancel(self, timeout=3.0):
        if not self.done:
            try:
                wait(self.handle.cancel_goal_async(), timeout, 'goal cancel')
            except TimeoutError:
                pass


class Nav2:
    def __init__(self, ros, map_frame='map', base_frame='base_link', costmap='/local_costmap/costmap'):
        node = self.node = ros.node
        self.map_frame, self.base_frame = map_frame, base_frame
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.to_pose = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.through_poses = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        self.planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        self.states = {name: node.create_client(GetState, f'/{name}/get_state') for name in SERVERS}
        self.costmap, self.costmap_time = None, None
        # Costmaps publish transient-local, so a late subscriber still gets the last one.
        node.create_subscription(OccupancyGrid, costmap, self._costmap,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _costmap(self, message):
        self.costmap, self.costmap_time = message, time.monotonic()

    def now(self):
        return self.node.get_clock().now().to_msg()

    def lifecycle(self, timeout=2.0):
        """Each server's lifecycle state label ('active', ...) or 'unavailable'."""
        states = {}
        for name, client in self.states.items():
            if not client.wait_for_service(timeout_sec=timeout):
                states[name] = 'unavailable'
                continue
            try:
                states[name] = wait(client.call_async(GetState.Request()), timeout,
                                    f'{name} get_state').current_state.label
            except TimeoutError:
                states[name] = 'unavailable'
        return states

    def pose(self):
        """(x, y, yaw) of the base in the map frame, or None without TF."""
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def target(self, x, y, yaw):
        return pose_stamped(self.map_frame, x, y, yaw, self.now())

    def plan(self, goal, timeout=10.0):
        """Plan from the current pose to ``goal`` (x, y, yaw); returns the path as (x, y)."""
        if not self.planner.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('compute_path_to_pose is not available; is Nav2 running?')
        request = ComputePathToPose.Goal(goal=self.target(*goal), use_start=False)
        handle = wait(self.planner.send_goal_async(request), timeout, 'plan request')
        if not handle.accepted:
            raise RuntimeError('planner rejected the goal')
        result = wait(handle.get_result_async(), timeout, 'planning').result
        return [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]

    def navigate(self, goals, timeout=10.0):
        """Start NavigateToPose (one goal) or NavigateThroughPoses; returns a Goal."""
        poses = [self.target(*goal) for goal in goals]
        if len(poses) == 1:
            name, client = 'navigate_to_pose', self.to_pose
            request = NavigateToPose.Goal(pose=poses[0])
        else:
            name, client = 'navigate_through_poses', self.through_poses
            request = NavigateThroughPoses.Goal(poses=poses)
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'{name} is not available; is Nav2 running?')
        handle = wait(client.send_goal_async(request), timeout, 'goal request')
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        return Goal(handle)
