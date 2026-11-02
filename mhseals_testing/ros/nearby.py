"""The TUI's view of raw clouds and images without subscribing to them (owner decision D-20261101-26).

mhseals_nav's nearby node summarizes /points boat-side on /perception/nearby (std_msgs/String JSON, at most 5 Hz,
only for a new cloud): ``{sectors: [nearest range per sector from the bow clockwise, null when empty], nearest:
{range, bearing} or null, count, rate, latency, frame}``, ranges in metres from base_link, bearing in degrees
clockwise from the bow. ``observe`` turns one message into the LiDAR sensor row (rate and latency as the node
measured them) and the ``nearby`` facts (``perc.nearby.range``, the points-nearby health check). A raw image topic is
only looked for on the graph (``published``); views that need the pixels or points subscribe while open
(RosIO.tap). Pure: no rclpy.
"""

import json
import math

TOPIC = '/perception/nearby'
PROBE = 'nearby'


def parse(text):
    """The nearby dict of one message, or None when it is not one."""
    try:
        found = json.loads(text)
    except (TypeError, ValueError):
        return None
    return found if isinstance(found, dict) and isinstance(found.get('sectors'), list) else None


def number(value):
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def nearest(found):
    """``(range, bearing)`` of the nearest point, or None when the cloud kept none."""
    near = found.get('nearest') if isinstance(found, dict) else None
    if not isinstance(near, dict) or number(near.get('range')) is None:
        return None
    return number(near['range']), number(near.get('bearing'))


def summary(found):
    near = nearest(found)
    count = found.get('count')
    if near is None:
        return f'nothing near ({count} pts)'
    bearing = '' if near[1] is None else f' at {near[1]:.0f} deg'
    return f'nearest {near[0]:.1f} m{bearing} ({count} pts)'


def observe(monitor, sensors, found, publish=None, fact=None):
    """One nearby message into each points sensor's row (``sensors``: their names) and the ``nearby`` facts."""
    latency = number(found.get('latency'))
    for name in sensors:
        monitor.observe(name, {'summary': summary(found), 'count': found.get('count')},
                        '' if latency is None or latency >= 0 else 'cloud stamped in the future',
                        stamp_age=latency, rate=number(found.get('rate')) or 0.0)
    if publish is not None and fact is not None:
        near = nearest(found)
        label = '' if near is None or near[1] is None else f'at {near[1]:.0f} deg'
        publish(PROBE, {} if near is None else {'perc.nearby.range': fact(near[0], label)})


def published(monitor, name, publishers):
    """A raw image sensor's row from the graph alone: present while its topic has a publisher."""
    if publishers:
        monitor.observe(name, {'summary': 'published; the pixels only while a view is open', 'graph': True})
