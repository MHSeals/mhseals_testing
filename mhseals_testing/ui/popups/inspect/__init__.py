"""popup.inspect: Enter on a Status row that has no popup of its own (tui-visual.md section 4).

The popup picks a builder by the row: a sensor gets its 60 s arrival-rate strip, a TF chain its
frame diagram, Compass/Yaw a heading dial and field gauge, GPS the fix ladder and an uncertainty
circle drawn to scale, RC its rate and latency, any other advisory its number over time. Each body
ends with the row's Detail line.

Everything is sampled only while the popup is open (``Inspect.update`` from the popup refresh),
into rings that die with it, so a closed popup costs nothing. Builders are pure functions of the
``Inspect`` state, the snapshot and the inner width, which the tests drive directly.

The package: ``common`` (formatting, the rings, the strip), one module per section (``sensor``, ``tf``,
``compass``, ``gps``, ``advisory``) and this one, the popup; every name inspect.py had is re-exported here.
"""

# Every name inspect.py had stays inspect.X (unused here: noqa F401 throughout).
from bisect import bisect_left, bisect_right
from collections import deque
import math
import time

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.analysis import fixlog, magnetic
from mhseals_testing.profiles import MIN_DEADLINE
from mhseals_testing.runner import fold
from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.panels.status import short, status_rows
from mhseals_testing.ui.popups.inspect.common import (  # noqa: F401
    ARRIVALS, EVERY, FIX_STEPS, FIX_TOKENS, Inspect, LABEL, MOVING, POINTS, RING, SEP, WIDTH, WINDOW, bearing,
    cardinal, columns, duration, find, hue, kv, lead, length, mark, nice, number, recent, sensor, strip)
from mhseals_testing.ui.popups.inspect.sensor import (  # noqa: F401
    rate_columns, sensor_part)
from mhseals_testing.ui.popups.inspect.tf import (  # noqa: F401
    chain, tf_part)
from mhseals_testing.ui.popups.inspect.compass import (  # noqa: F401
    compass_heading, compass_part, dial)
from mhseals_testing.ui.popups.inspect.gps import (  # noqa: F401
    BANDS, fix_step, gps_part, uncertainty)
from mhseals_testing.ui.popups.inspect.advisory import (  # noqa: F401
    advisory_part, rc_part)


# --- the popup --------------------------------------------------------------------------------

def body(state, snapshot, inner=WIDTH - 4, edges=()):
    """The renderables for ``state.row`` (refreshed from ``snapshot``)."""
    row = find(status_rows(snapshot, state.specs), state.row)
    name, out = row['name'], []
    spec = state.watched()
    if row.get('source') == 'tf':
        source, _, target = name.partition(f' {theme.ARROW} ')
        out += tf_part(edges, source, target, inner)
    elif name in ('Compass', 'Yaw') or (state.spec and state.spec.kind == 'mag'):
        out += compass_part(snapshot, state.specs, inner, state.compact)
    elif name == 'GPS' or (state.spec and state.spec.kind == 'gpsraw'):
        out += gps_part(state, snapshot, inner)
    elif name == 'RC':
        out += rc_part(state, snapshot, inner)
    elif row.get('group') == 'System':
        out += advisory_part(state, snapshot, inner)
    elif state.spec and state.spec.kind in ('imu', 'odometry'):
        key, label, unit = ('wz', 'Gyro z', 'rad/s') if state.spec.kind == 'imu' else ('speed', 'Speed', 'm/s')
        out += strip(label, recent(state.series.get(key, ()), state.now), state.now, inner, unit)
    if spec is not None:
        if out:
            out.append(Text(''))
        out += sensor_part(state, spec, snapshot['sensors'].get(spec.name), inner)
    out.append(kv('Detail', theme.ascii_safe(str(row.get('detail', '')))))
    return Group(*out)


def title(row, spec):
    name = row.get('label') or row['name']
    return f"{name}{SEP}{spec.topic}" if spec else name


def open_inspect(app, row=None):
    """Open the inspect popup on ``row`` (default: the Status cursor row); any key but j/k closes."""
    specs = app.profile.sensors
    if row is None:
        rows = status_rows(app.snapshot(), specs)
        if not rows:
            return
        row = rows[min(app.cursors['status'], len(rows) - 1)]
    live = bool(getattr(app.source, 'live', True))
    probe = Inspect(row, specs, app.monitor, live, compact=app.console.size.height < 32 or app.console.size.width < 100)
    spec = probe.watched()
    if spec and live:
        try:   # one graph query at open, never per refresh
            probe.publishers = list(app.ros.publishers(spec.topic))
        except Exception:   # noqa: BLE001  no graph (review, tests): say unknown
            probe.publishers = None
    feeds = getattr(app.ros, 'map', None) if live else None
    probe.fix_changes = getattr(feeds, 'fix_changes', None)
    probe.accuracy = getattr(getattr(feeds, 'state', None), 'accuracy', None)   # the minimap's shared reading
    modal = dict(kind='message', title=title(row, spec), width=WIDTH, hint='Any key closes')   # j/k added when clipped
    last = [None]

    def refresh():
        now = app.monitor.clock()
        if last[0] is not None and now - last[0] < EVERY:
            return
        last[0] = now
        snapshot = app.snapshot()
        probe.update(snapshot, now)
        edges = app.monitor.tf_edges() if row.get('source') == 'tf' and live else ()
        modal['body'] = body(probe, snapshot, WIDTH - 4, edges)

    def handle(key):
        return not (modal.get('clipped') and app.scroll(modal, fold(key), ('k', 'j')))
    refresh()
    app.popup(modal, handle, refresh)
