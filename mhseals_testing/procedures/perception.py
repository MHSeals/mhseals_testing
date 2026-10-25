"""Perception checks, without moving: camera, LiDAR, detections, tracks and the costmap.

Rates come from the monitor over an observation window. Minimum rates are
what the tracker and costmaps need to stay current, not sensor maxima. A row
with a nominal ``hz`` (a 2 Hz preview) fails below half of it, as topic_rate does;
without one its kind's minimum applies.
"""

MIN_HZ = {'image': 5.0, 'compressed': 5.0, 'points': 5.0, 'detections': 1.0}
FAIL_FRACTION = 0.5   # of a row's nominal hz; topic_rate's fail band
PERCEPTION_COSTMAP = '/perception/costmap'   # the camera host's grid Nav2 layers in (v2)
LETHAL = 99  # costmap cell value of an obstacle (100) or its inscribed inflation (99)


def verdict(row, minimum):
    if row is None:
        return 'MISSING', 'no messages'
    if row['status'] == 'STALE':
        return 'FAIL', f"stale for {row['age']:.1f} s"
    if row['hz'] < minimum:
        return 'FAIL', f"{row['hz']:.1f} Hz < {minimum:g} Hz"
    if row['issue']:
        return 'WARN', row['issue']
    return 'PASS', f"{row['hz']:.1f} Hz " + row['values'].get('summary', '')


def minimum_hz(sensor):
    return sensor.hz * FAIL_FRACTION if sensor.hz else MIN_HZ[sensor.kind]


def occupied_cells(grid):
    return sum(1 for value in grid.data if value >= LETHAL)


def run(ctx):
    with ctx.observe('perception') as runner:
        runner.supervise(ctx.settings.observe, label='measuring perception rates')
    sensors = ctx.monitor.snapshot()['sensors']
    results = []
    for sensor in ctx.profile.sensors:
        if sensor.kind not in MIN_HZ:
            continue
        level, detail = verdict(sensors.get(sensor.name), minimum_hz(sensor))
        results.append(dict(procedure='perception', check=sensor.name, verdict=level,
                            detail=f'{sensor.topic}: {detail}'))
    nav = ctx.services['nav2']()
    # Perception's own grid when the camera host publishes one, else Nav2's local costmap.
    source, costmap = PERCEPTION_COSTMAP, nav.fetch_costmap(topic=PERCEPTION_COSTMAP)
    if costmap is None:
        source, costmap = f'local costmap (no {PERCEPTION_COSTMAP})', nav.fetch_costmap()
    if costmap is None:
        results.append(dict(procedure='perception', check='costmap', verdict='N/A',
                            detail=f'no {PERCEPTION_COSTMAP} and no local costmap '
                                   '(Nav2 not running)'))
    else:
        cells = occupied_cells(costmap)
        lidar = sensors.get('LiDAR', {}).get('status')
        results.append(dict(procedure='perception', check='costmap',
                            verdict='PASS' if cells else ('WARN' if lidar == 'LIVE' else 'N/A'),
                            detail=f'{source}: {cells} obstacle cells; LiDAR {lidar or "MISSING"}'
                                   + ('' if cells else ' (open water shows none)')))
    for result in results:
        ctx.record(result)
    return results
