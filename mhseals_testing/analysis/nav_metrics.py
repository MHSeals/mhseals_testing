"""Waypoint-following and station-keeping metrics (docs/math.tex, "Navigation metrics").

Positions are (x, y) in the map frame, in metres.
"""

import numpy as np


def segment_distances(points, waypoints):
    """Distance from each point to the nearest segment of the waypoint polyline."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    corners = np.asarray(waypoints, dtype=float).reshape(-1, 2)
    if len(corners) == 1:
        return np.linalg.norm(points - corners[0], axis=1)
    best = np.full(len(points), np.inf)
    for a, b in zip(corners[:-1], corners[1:]):
        direction = b - a
        length2 = float(direction @ direction)
        # Project onto the segment, clamped to its ends.
        t = np.clip(((points - a) @ direction) / length2, 0, 1) if length2 else np.zeros(len(points))
        best = np.minimum(best, np.linalg.norm(points - (a + t[:, None] * direction), axis=1))
    return best


def path_length(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def waypoint_metrics(points, waypoints, start=None):
    """Cross-track error along the route and the closest approach to each waypoint.

    ``start`` (the pose when the goal was sent) is prepended to the route, so the
    first leg is scored too.
    """
    if len(points) < 2:
        return dict(samples=len(points), verdict='INCONCLUSIVE')
    route = ([start] if start is not None else []) + list(waypoints)
    errors = segment_distances(points, route)
    points = np.asarray(points, dtype=float)
    closest = [float(np.min(np.linalg.norm(points - np.asarray(w, dtype=float), axis=1)))
               for w in waypoints]
    return dict(samples=len(points), path_length=path_length(points),
                route_length=path_length(route),
                cross_track_mean=float(errors.mean()),
                cross_track_rms=float(np.sqrt(np.mean(errors ** 2))),
                cross_track_max=float(errors.max()),
                waypoint_miss=closest, final_error=closest[-1])


def station_keeping(times, points, center, tolerance):
    """How well the boat holds ``center``: radius statistics and the drift velocity.

    The drift is the least-squares slope of x(t) and y(t): a steady push from
    current or wind that the controller does not cancel shows up here even when
    the radius stays inside the tolerance.
    """
    times = np.asarray(times, dtype=float)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 3 or times[-1] <= times[0]:
        return dict(samples=len(points), verdict='INCONCLUSIVE')
    radii = np.linalg.norm(points - np.asarray(center, dtype=float), axis=1)
    design = np.column_stack([np.ones_like(times), times - times[0]])
    (_, vx), (_, vy) = (np.linalg.lstsq(design, points[:, axis], rcond=None)[0] for axis in (0, 1))
    # Fraction of time inside the tolerance, weighting each sample by its interval.
    weights = np.diff(times, append=times[-1])
    inside = float(weights[radii <= tolerance].sum() / weights.sum()) if weights.sum() else 0.0
    return dict(samples=len(points), rms_radius=float(np.sqrt(np.mean(radii ** 2))),
                max_radius=float(radii.max()), drift_speed=float(np.hypot(vx, vy)),
                drift_heading=float(np.arctan2(vy, vx)), time_inside=inside,
                verdict='HELD' if radii.max() <= tolerance else 'LEFT TOLERANCE')
