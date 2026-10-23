"""Course truth and track builders shared by the referee tests."""
import math


def element(task, label, x, y, radius=0.2):
    return dict(task=task, label=label, template='', x=x, y=y, radius=radius)


TRUTH = dict(schema='crane-roboboat-course-truth-v1', elements=[
    element('nav_channel', 'red_pole_buoy', 7, 1.5), element('nav_channel', 'green_pole_buoy', 7, -1.5),
    element('nav_channel', 'red_pole_buoy', 13, 1.5),
    element('nav_channel', 'green_pole_buoy', 13, -1.5),
    element('speed_challenge', 'red_buoy', 3, 12.25), element('speed_challenge', 'green_buoy', 3, 9.75),
    element('speed_challenge', 'black_buoy', 10, 12.6),
    element('speed_challenge', 'black_buoy', 14, 9.4),
    element('speed_challenge', 'blue_buoy', 19, 11),
], bays=[
    dict(dock=0, index=0, x=12, y=-19, yaw=math.pi / 2, width=1.8, depth=1.8,
         banner='blue_square', occupied=True),
    dict(dock=0, index=2, x=16, y=-19, yaw=math.pi / 2, width=1.8, depth=1.8,
         banner='blue_square', occupied=False),
])


def line(a, b, t0=0.0, steps=50, dt=0.2):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    return [(t0 + i * dt, a[0] + (b[0] - a[0]) * i / steps, a[1] + (b[1] - a[1]) * i / steps, yaw)
            for i in range(steps + 1)]
