"""Status panel: sensors, TF chains and advisories as one navigable list."""

from rich.console import Group
from rich.text import Text

from mhseals_testing.analysis import magnetic
from mhseals_testing.health.kinds.topic_rate import classify
from mhseals_testing.readiness import advisories
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES, fold_groups, window
from mhseals_testing.ui.theme import MUTED, SEPARATOR

MARKS = {'↓': theme.pick('↓', 'v'), '↑': theme.pick('↑', '^'), '~': '~',
         '⏱': theme.pick('⏱', 't'), '×': theme.pick('×', 'x')}
RATE_STYLE = {'warn': theme.WARN, 'fail': theme.BAD}  # Hz colour; a healthy rate stays muted


def status_rows(snapshot, specs):
    """Sensors, TF chains and advisories as one navigable list."""
    rows = []
    for spec in specs:
        row = snapshot['sensors'].get(spec.name)
        if row is None:
            state, value, detail = ('MISSING' if spec.required else 'OFF'), '', spec.topic
        else:
            state = row['status']
            # A silent sensor reads stale, not 'once': its rate window has simply emptied.
            value = ('stale' if row['status'] == 'STALE' else
                     f"{row['hz']:.0f} Hz" if row['hz'] else 'once')
            detail = row['issue'] or row['values'].get('summary', '') or spec.topic
        rate = ''
        if row is not None and spec.hz and state != 'STALE':
            rate, mark = classify(spec.hz, row['hz'], row['max_gap'], row['stamp_age'],
                                  limit=spec.deadline)
            rate = '' if rate == 'ok' else rate
            value += MARKS.get(mark, '')
        rows.append(dict(group='Sensors', state=state, name=spec.name, value=value, rate=rate,
                         detail=f'{spec.topic}{SEPARATOR}{detail}' if detail != spec.topic
                         else spec.topic, required=spec.required))
    rows.extend(localization_rows(snapshot))
    for chain in snapshot['tf']:
        rows.append(dict(group='Frames', state=chain['status'],
                         name=f"{chain['source']} {theme.ARROW} {chain['target']}", value='',
                         detail=chain['detail'], required=True))
    rows.extend(snapshot.get('components', ()))
    for name, level, detail in advisories(snapshot):
        detail = theme.ascii_safe(detail)
        rows.append(dict(group='System', state=level, name=name, value=short(detail),
                         detail=detail, required=False))
    return rows


def localization_rows(snapshot, site=magnetic.SITE):
    """GPS fix, compass field and yaw trust, from the optional gpsraw and mag sensors.

    gpsraw values: fix, sats, h_acc (m). mag values: x, y, z (tesla); optional
    compass_gyro (deg), magcal_residual (deg) and yaw_sigma (rad) from the monitor.
    A sensor that is not LIVE shows STALE rather than its last good value.
    A missing sensor gives no row, so a boat without them looks as before.
    """
    rows = []
    gps = snapshot['sensors'].get('gpsraw')
    if gps:
        v = gps['values']
        state = 'OK' if v.get('fix', 0) >= 3 and gps['status'] == 'LIVE' else 'WARN'
        fix = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'FLOAT', 6: 'FIXED'}.get(v.get('fix'), '?')
        value = f"{fix} {theme.SEPARATOR.strip()} {v.get('sats', 0)}"
        if v.get('h_acc'):
            value += f" {theme.SEPARATOR.strip()} {v['h_acc']:.1f}m"
        rows.append(dict(group='Localization', state=state, name='GPS', value=value,
                         detail='gpsraw fix, satellites, horizontal accuracy', required=False))
    mag = snapshot['sensors'].get('mag')
    if mag:
        v = mag['values']
        ratio = magnetic.field_ratio(v['x'], v['y'], v['z'], site)
        bad = not magnetic.FIELD_RATIO[0] <= ratio <= magnetic.FIELD_RATIO[1]
        field = magnetic.field_mg(v['x'], v['y'], v['z'])
        live = mag['status'] == 'LIVE'
        rows.append(dict(group='Localization', state=('STALE' if not live else 'ERROR' if bad else 'OK'), name='Compass',
                         value=f"{field:.0f}/{site['field_mg']:.0f} mG",
                         detail='|B| against the site WMM field', required=False))
        quality, reason = magnetic.yaw_quality(v.get('compass_gyro'), v.get('magcal_residual'),
                                               v.get('yaw_sigma'), ratio)
        rows.append(dict(group='Localization', state='STALE' if not live else 'OK' if quality == 'ok' else 'WARN',
                         name='Yaw', value=theme.ascii_safe(reason) or 'ok', detail='yaw_quality: compass, gyro, magcal, sigma',
                         required=False))
    return rows


def short(detail, limit=10):
    """The telling number of an advisory: '78°C' of 'load 7.8/20, 78°C', '+0.04s' of a clock."""
    summary, separator, _ = str(detail).partition(';')
    if separator and len(summary) <= limit:
        return summary
    for part in reversed(str(detail).split(',')):
        token = part.strip().split(' ')[-1] if part.strip() else ''
        if token and any(c.isdigit() for c in token) and '/' not in token and len(token) <= limit:
            return token
    return ''


def status_panel(snapshot, view, height):
    rows = status_rows(snapshot, view['specs'])
    focused = view['focus'] == 'status'
    cursor = min(view['cursors'].get('status', 0), max(0, len(rows) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, {'header': '', 'justify': 'right'},
                       padding=(0, 0))
    table.show_header = False
    budget = max(1, height - 4)
    groups = {}
    for row in rows:
        groups.setdefault(row['group'], []).append(row['state'])
    cursor_group = rows[cursor]['group'] if rows else None
    folded = fold_groups(groups, budget, cursor_group)
    # Entries are heading lines, folded-group lines and rows; the window keeps the cursor row in view.
    entries, at = [], 0
    for group, states in groups.items():
        if group in folded:
            entries.append(('fold', group, len(states)))
        else:
            entries.append(('head', group, 0))
            entries += [('row', index, 0) for index in range(at, at + len(states))]
        at += len(states)
    place = next((i for i, e in enumerate(entries) if e[0] == 'row' and e[1] == cursor), 0)
    start, visible = window(entries, place, budget)
    for kind, key, count in visible:
        if kind == 'head':
            table.add_row(Text(key, style=f'{MUTED} bold'), '')
        elif kind == 'fold':
            table.add_row(Text(f'{key}  {theme.FOCUS}', style=f'{MUTED} bold'),
                          Text(f'{count} ok', style=MUTED))
        else:
            row = rows[key]
            name = Text(row['name'], style='bold' if row['required'] else None)
            table.add_row(theme.row(row['state'], name, focused and key == cursor),
                          Text(row['value'], style=RATE_STYLE.get(row.get('rate'), MUTED)))
    detail = Text(rows[cursor]['detail'] if rows else 'No sensors in this profile',
                  style=MUTED, no_wrap=True, overflow=theme.OVERFLOW)
    live = sum(1 for row in rows if row['group'] == 'Sensors' and row['state'] == 'LIVE')
    sensors = sum(1 for row in rows if row['group'] == 'Sensors')
    return theme.block(Group(table, Text(''), detail), TITLES['status'], focused,
                       f'{live}/{sensors} live')
