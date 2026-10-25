"""A thin Nav2 client: lifecycle states, planning, navigation goals, costmap and map pose.

Calls run on the RosIO executor thread; these methods wait on futures, so the
TUI thread can call them directly.
"""

from array import array
from contextlib import contextmanager
import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from mhseals_testing.harness.goals import fence_config
from mhseals_testing.ros.minimap_feeds import LazySub

# The servers mhseals_nav's navigation.launch.py brings up.
SERVERS = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
           'waypoint_follower', 'velocity_smoother')


COSTMAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FENCE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def wait(future, timeout, what):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f'{what} timed out after {timeout:g}s')
        time.sleep(0.02)
    return future.result()


def occupancy(costs):
    """Nav2 costs 0-255 as OccupancyGrid values, by costmap_2d_publisher's translation table:
    0 free, 1-252 to 1-98, 253 inscribed 99, 254 lethal 100, 255 unknown -1."""
    c = np.frombuffer(bytes(costs), dtype=np.uint8).astype(np.int32)
    out = np.where(c == 0, 0, 1 + (97 * (c - 1)) // 251)
    out = np.where(c == 253, 99, np.where(c == 254, 100, np.where(c == 255, -1, out)))
    return out.astype(np.int8)


def grid_of(costmap):
    """A nav_msgs/OccupancyGrid from a GetCostmap reply's nav2_msgs/Costmap."""
    meta, grid = costmap.metadata, OccupancyGrid()
    grid.header = costmap.header
    grid.info.resolution = meta.resolution
    grid.info.width, grid.info.height = meta.size_x, meta.size_y
    grid.info.origin = meta.origin
    grid.data = array('b', occupancy(costmap.data).tobytes())
    return grid


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
        # Costmaps publish transient-local, so a late subscriber still gets the last one. The grid
        # is 0.58 MB/s, so it is subscribed only inside costmap_lease().
        self.costmap_topic = costmap
        self._costmap_sub = LazySub(node, OccupancyGrid, costmap, self._costmap, COSTMAP_QOS)
        # The costmap topic sends a full grid only when a subscriber is there as the window moves, so a
        # late subscriber can get a grid latched minutes ago, far behind the boat. The costmap node's
        # GetCostmap service answers with the grid it holds now.
        self.costmap_client = node.create_client(GetCostmap, costmap.rsplit('/', 1)[0] + '/get_costmap')
        # fence_watchdog's fence, for the goal gates: small and 1 Hz, so always subscribed
        self.fence, self.fence_time = None, None
        found = fence_config()
        if found is not None:
            node.create_subscription(String, found[0], self._fence, FENCE_QOS)

    def _fence(self, message):
        try:
            state = json.loads(message.data)
        except ValueError:
            state = None
        self.fence = state if isinstance(state, dict) else {'valid': False, 'error': 'unreadable fence message'}
        self.fence_time = time.monotonic()

    def fence_state(self, timeout=1.0):
        """(fence_watchdog's last fence message or None, its receipt age in s). The first call waits up
        to ``timeout`` for one (the latched one arrives once the publisher is discovered); later calls
        never wait, so a missing watchdog stalls nothing."""
        if not getattr(self, 'fence_waited', False):
            self.fence_waited = True
            deadline = time.monotonic() + timeout
            while getattr(self, 'fence_time', None) is None and time.monotonic() < deadline:
                time.sleep(0.02)
        at = getattr(self, 'fence_time', None)
        return getattr(self, 'fence', None), (math.inf if at is None else time.monotonic() - at)

    def stamp_age(self, stamp):
        """Seconds from ``stamp`` to the node's clock (sim time in the sim); 0 when it is ahead or unset."""
        if not (stamp.sec or stamp.nanosec):
            return 0.0
        age = (self.node.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9
        return max(0.0, age)

    def _costmap(self, message):
        # judged by its stamp, not by its arrival: a latched grid arrives at once however old it is
        self.costmap, self.costmap_time = message, time.monotonic() - self.stamp_age(message.header.stamp)

    def served_costmap(self, timeout=2.0):
        """The grid the costmap node holds now (GetCostmap), with ``costmap_time`` from its stamp; None
        when the service is not there or does not answer in ``timeout``."""
        client = getattr(self, 'costmap_client', None)
        if client is None or not client.wait_for_service(timeout_sec=min(timeout, 0.5)):
            return None
        try:
            reply = wait(client.call_async(GetCostmap.Request()), timeout, 'get_costmap')
        except TimeoutError:
            return None
        grid = grid_of(reply.map)
        self.costmap, self.costmap_time = grid, time.monotonic() - self.stamp_age(reply.map.header.stamp)
        return grid

    @contextmanager
    def costmap_lease(self, who):
        """Subscribe to the costmap for the block; ``costmap`` holds the last grid afterwards.

        Opening the first lease drops the old grid, so a later wait sees only a fresh one."""
        if not self._costmap_sub.holders:
            self.costmap = self.costmap_time = None
        self._costmap_sub.lease(who)
        try:
            yield self
        finally:
            self._costmap_sub.lease(who, False)

    def fetch_costmap(self, timeout=2.0, topic=None):
        """The latest costmap, subscribing just long enough to get one; None if none arrives.

        ``topic`` reads another grid once (the camera host's /perception/costmap) without
        touching the leased one."""
        if topic is not None and topic != self.costmap_topic:
            got = []
            sub = self.node.create_subscription(OccupancyGrid, topic, got.append, COSTMAP_QOS)
            try:
                deadline = time.monotonic() + timeout
                while not got and time.monotonic() < deadline:
                    time.sleep(0.02)
            finally:
                self.node.destroy_subscription(sub)
            return got[-1] if got else None
        served = self.served_costmap(timeout) if hasattr(self, 'served_costmap') else None
        if served is not None:
            return served
        with self.costmap_lease('fetch'):
            deadline = time.monotonic() + timeout
            while self.costmap is None and time.monotonic() < deadline:
                time.sleep(0.02)
        return self.costmap

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
