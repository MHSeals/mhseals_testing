"""Nav2 procedures: bring-up checks (no motion), waypoint following and station keeping.

Nav2 drives /cmd_vel itself, so these supervise rather than command: the
interlocks and abort keys stay live, and the goal is cancelled on every exit.
Metrics: analysis/nav_metrics.py.
"""

import math

from mhseals_testing.analysis.nav_metrics import path_length, station_keeping, waypoint_metrics

PLAN_DISTANCE = 5.0  # m ahead for the bring-up planning check
COSTMAP_WAIT = 15.0  # s; a freshly launched costmap publishes only after its first update


def ahead(pose, forward, left=0.0):
    """A map point ``forward``/``left`` metres from ``pose`` (x, y, yaw), in its body frame."""
    x, y, yaw = pose
    return (x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw))


def square(pose, side):
    """A square to port: forward, left, back, home; each waypoint faces along its leg."""
    corners = [ahead(pose, *offset) for offset in ((side, 0), (side, side), (0, side), (0, 0))]
    route, previous = [], pose[:2]
    for corner in corners:
        route.append((*corner, math.atan2(corner[1] - previous[1], corner[0] - previous[0])))
        previous = corner
    return route


def current_pose(nav):
    pose = nav.pose()
    if pose is None:
        raise RuntimeError(f'no {nav.map_frame} -> {nav.base_frame} transform')
    return pose


def bringup(ctx):
    """Lifecycle, costmap and planning, without moving."""
    nav = ctx.services['nav2']()

    def record(check, ok, detail):
        ctx.record(dict(procedure='nav2 bring-up', check=check,
                        verdict='PASS' if ok else 'FAIL', detail=detail))

    states = nav.lifecycle()
    inactive = {name: state for name, state in states.items() if state != 'active'}
    record('lifecycle', not inactive,
           ', '.join(f'{name}: {state}' for name, state in inactive.items()) or 'all active')
    with ctx.observe('nav2_bringup') as runner:
        runner.supervise(COSTMAP_WAIT, done=lambda: nav.costmap is not None,
                         label='waiting for costmap')
    lidar = ctx.monitor.snapshot()['sensors'].get('LiDAR', {}).get('status', 'MISSING')
    record('costmap', nav.costmap is not None,
           f'local costmap received; LiDAR {lidar}' if nav.costmap is not None
           else f'no local costmap in {COSTMAP_WAIT:g} s')
    pose = nav.pose()
    if pose is None:
        record('plan', False, f'no {nav.map_frame} -> {nav.base_frame} transform')
        return
    try:
        path = nav.plan((*ahead(pose, PLAN_DISTANCE), pose[2]))
        record('plan', len(path) > 1, f'{len(path)} poses, {path_length(path):.1f} m '
                                       f'to {PLAN_DISTANCE:g} m ahead')
    except (RuntimeError, TimeoutError) as error:
        record('plan', False, str(error))


def waypoints(ctx):
    nav = ctx.services['nav2']()
    settings = ctx.settings
    start = current_pose(nav)
    route = square(start, settings.nav_distance)
    ctx.prompt(f'Waypoint square, {settings.nav_distance:g} m sides, to the port side. '
               'Clear the area and press Enter... ')
    times, points = [], []

    def sample(now):
        pose = nav.pose()
        if pose is not None:
            times.append(now)
            points.append(pose[:2])

    timeout = 60.0 + 15.0 * len(route)
    with ctx.supervise('waypoints', route=route) as runner:
        goal = nav.navigate(route)
        try:
            began, ended, _ = runner.supervise(timeout, done=lambda: goal.done,
                                               label='following waypoints', sample=sample)
        finally:
            goal.cancel()
    status = goal.status if goal.done else 'TIMEOUT'
    result = dict(waypoint_metrics(points, [corner[:2] for corner in route], start=start[:2]),
                  procedure='waypoints', status=status, duration=ended - began,
                  verdict='REACHED' if status == 'SUCCEEDED' else status)
    ctx.record(result)
    return result


def station(ctx):
    nav = ctx.services['nav2']()
    settings = ctx.settings
    center = current_pose(nav)
    ctx.prompt(f'Station keeping for {settings.station_hold:g} s within '
               f'{settings.station_tolerance:g} m of here. Press Enter... ')
    times, points = [], []
    state = dict(goal=None, resends=0)

    def sample(now):
        pose = nav.pose()
        if pose is None:
            return
        times.append(now)
        points.append(pose[:2])
        # Nav2 stops once a goal succeeds; re-send it when the boat drifts off.
        drift = math.dist(pose[:2], center[:2])
        if state['goal'].done and drift > settings.station_tolerance / 2:
            state['goal'] = nav.navigate([center])
            state['resends'] += 1

    with ctx.supervise('station_keeping', center=center, hold=settings.station_hold) as runner:
        state['goal'] = nav.navigate([center])
        try:
            runner.supervise(settings.station_hold, label='station keeping', sample=sample)
        finally:
            state['goal'].cancel()
    result = dict(station_keeping(times, points, center[:2], settings.station_tolerance),
                  procedure='station keeping', resends=state['resends'],
                  tolerance=settings.station_tolerance)
    ctx.record(result)
    return result
