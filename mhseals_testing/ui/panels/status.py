"""Status panel: sensors, TF chains and advisories as one navigable list."""

from rich.console import Group
from rich.text import Text

from mhseals_testing.analysis import magnetic
from mhseals_testing.health.kinds.topic_rate import classify
from mhseals_testing.readiness import advisories
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import GROUPS, HEADERLESS, TITLES, fold_groups, window
from mhseals_testing.ui.theme import MUTED, SEPARATOR

MARKS = {'↓': theme.pick('↓', 'v'), '↑': theme.pick('↑', '^'), '~': '~',
         '⏱': theme.pick('⏱', 't'), '×': theme.pick('×', 'x')}
RC_MIN_HZ = 10                      # below this the RC link reads '!' (authority 4.3)
RATE_STYLE = {'warn': theme.WARN, 'fail': theme.BAD}  # Hz colour; a healthy rate stays muted


def sensor_rows(snapshot, specs):
    """One Components row per profile sensor (``source='sensor'``, read by the health strip)."""
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
        rows.append(dict(group='Components', source='sensor', state=state, name=spec.name,
                         value=value, rate=rate, required=spec.required,
                         detail=f'{spec.topic}{SEPARATOR}{detail}' if detail != spec.topic
                         else spec.topic))
    return rows


def component_rows(snapshot, specs):
    """Sensors, then the managed components the app feeds into ``snapshot['components']``."""
    return sensor_rows(snapshot, specs) + list(snapshot.get('components', ()))


def frame_rows(snapshot, specs):
    """TF chains, as Localization detail (``source='tf'``, read by the health strip)."""
    return [dict(group='Localization', source='tf', state=chain['status'],
                 name=f"{chain['source']} {theme.ARROW} {chain['target']}", value='',
                 detail=chain['detail'], required=True) for chain in snapshot['tf']]


def config_value(detail):
    """The Config row's cell: ``✓ 9 files`` or ``rejected`` (the glyph is in col 3); the rest stays in the detail."""
    head = detail.split(theme.SEPARATOR)[0]
    return 'rejected' if head.startswith('!') else head


def system_rows(snapshot, specs):
    """Engine advisories (compute, clock, FCU, network, Battery, ...) except the Session ones."""
    rows = []
    for name, level, detail in advisories(snapshot):
        detail = theme.ascii_safe(detail)
        rows.append(dict(group='Session' if name in SESSION else 'System', state=level, name=name,
                         value=config_value(detail) if name == 'Config' else short(detail), detail=detail, required=False))
    return rows


def health_rows(snapshot, specs):
    """Row 0, ``Health  41 ok`` or ``! Health  1 warn`` (``snapshot['health']`` from ui.feeds.health)."""
    health = snapshot.get('health')
    if not health:
        return []
    bad, warn = health.get('bad', 0), health.get('warn', 0)
    parts = [f'{n} {word}' for n, word in ((bad, 'fail'), (warn, 'warn')) if n]
    state = 'FAIL' if bad else 'WARN' if warn else 'OK'
    value = ' · '.join(parts) or f"{health.get('ok', 0)} ok"
    return [dict(group='Health', state=state, name='Health', value=theme.ascii_safe(value), plain=state == 'OK',
                 detail=health.get('worst') or 'all checks ok', required=False)]


def control_rows(snapshot, specs):
    """Driving, RC, Hold, Agents from ``snapshot['control']`` (ui.feeds.authority.control); ui 6.1."""
    data = snapshot.get('control')
    if data is None:
        return []

    def row(name, value, detail, state='OK', style=None, **extra):
        return dict(group='Control', state=state, name=name, value=theme.ascii_safe(value), detail=detail,
                    required=False, plain=state == 'OK', style=style, **extra)

    kind, word = data.get('driving') or (None, 'idle')
    rows = [row('Driving', word, 'who drives; Enter opens the authority popup',
                style=theme.auth_style(kind) if kind else None)]
    rc = data.get('rc')
    if not rc:
        rows.append(row('RC', 'no receiver', 'no RC_CHANNELS seen', 'OFF'))
    elif not rc.get('valid', True):
        rows.append(row('RC', 'lost', 'receiver present but not valid', 'WARN'))
    else:
        hz, latency = rc.get('rate_hz') or 0, rc.get('latency_ms')
        value = f"{rc.get('mode', '')}{SEPARATOR}{hz:.0f} Hz" + (f' ~{latency:.0f}ms' if latency else '')
        slow = hz < RC_MIN_HZ
        rows.append(row('RC', value, f'RC_CHANNELS at {hz:.0f} Hz: N1 not applied or FCU rebooted' if slow
                        else 'rc mode, rate and estimated latency', 'WARN' if slow else 'OK'))
    holds = data.get('holds') or []
    if holds:
        hold = holds[0]
        rows.append(row('Hold', f"HOLD {hold.get('cause') or hold.get('by', '')} {hold.get('age', '')}".strip(),
                        'Enter releases the hold', style=theme.auth_style('hold')))
    else:
        rows.append(row('Hold', 'off', 'no hold on the command mux'))
    left, right = theme.pick('‹ ', '< '), theme.pick(' ›', ' >')
    rows.append(row('Agents', f"{left}{data.get('agents', 'allow')}{right}", 'left/right: allow or block new agent claims',
                    style='', setting='agents'))
    return rows


def recording_rows(snapshot, specs):
    """``Recording ‹ lean · full · off ›`` (System group; ui 6.1) when the app owns a recorder."""
    mode = snapshot.get('rec_mode')
    if mode is None:
        return []
    left, right = theme.pick('‹ ', '< '), theme.pick(' ›', ' >')
    return [dict(group='System', state='OK' if mode != 'off' else 'WARN', name='Recording', required=False,
                 value=f'{left}{mode}{right}', plain=mode != 'off', setting='recording',
                 detail='left/right: lean, full or off; each change ends the segment and starts the next')]


def session_rows(snapshot, specs):
    """Rows the app feeds into ``snapshot['session']`` (Command effort, Bench)."""
    return list(snapshot.get('session', ()))


def sensor(snapshot, specs, kind):
    """The snapshot row of the first profile sensor of ``kind``; None if absent or silent."""
    for spec in specs:
        if spec.kind == kind:
            return snapshot['sensors'].get(spec.name)
    return None


def localization_rows(snapshot, specs, site=magnetic.SITE):
    """GPS fix, compass field and yaw trust, from the optional GPS raw and compass sensors.

    The sensors are found in ``specs`` by their profile ``kind`` ('gpsraw', 'mag'), not by display
    name or topic, because a profile may call them anything or put them on other topics.

    gpsraw values: fix, sats, h_acc (m). mag values: x, y, z (tesla); optional
    compass_gyro (deg), magcal_residual (deg) and yaw_sigma (rad) from the monitor.
    A sensor that is not LIVE shows STALE rather than its last good value.
    A sensor the profile lacks, or that has not reported, gives no row, so a boat without them looks as before.
    """
    rows = []
    gps = sensor(snapshot, specs, 'gpsraw')
    if gps:
        v = gps['values']
        state = 'OK' if v.get('fix', 0) >= 3 and gps['status'] == 'LIVE' else 'WARN'
        fix = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'FLOAT', 6: 'FIXED'}.get(v.get('fix'), '?')
        value = f"{fix} {theme.SEPARATOR.strip()} {v.get('sats', 0)}"
        if v.get('h_acc'):
            value += f" {theme.SEPARATOR.strip()} {v['h_acc']:.1f}m"
        rows.append(dict(group='Localization', state=state, name='GPS', value=value,
                         detail='gpsraw fix, satellites, horizontal accuracy', required=False))
    mag = sensor(snapshot, specs, 'mag')
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


# A provider is ``f(snapshot, specs) -> rows``; each row names its own group. Later items append
# providers (Control, Recording, Config, RTK) without touching the grouping below.
PROVIDERS = [health_rows, control_rows, component_rows, localization_rows, frame_rows, system_rows, recording_rows, session_rows]
# Placeholder until the Command-effort and Bench items publish these advisories (ui.md 6.1, Session group);
# no producer exists yet, so only the test advises them.
SESSION = ('Command', 'Bench')   # advisory names that belong to Session, not System


def status_rows(snapshot, specs):
    """The ui 6.1 groups in order (``GROUPS``), each row a dict with a ``group`` key."""
    rows = [row for provider in PROVIDERS for row in provider(snapshot, specs)]
    return sorted(rows, key=lambda row: GROUPS.index(row['group']) if row['group'] in GROUPS
                  else len(GROUPS))


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


def row_label(row, active):
    """The glyph and label cell; a healthy summary row keeps the glyph cell blank so labels align."""
    name = Text(row['name'], style='bold' if row['required'] else None)
    plain = row.get('plain')
    return theme.row(None if plain else row['state'], name, active, prefix=' ' if plain else None)


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
            if group not in HEADERLESS:
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
            table.add_row(row_label(row, focused and key == cursor),
                          Text(row['value'], style=row['style'] if row.get('style') is not None
                               else RATE_STYLE.get(row.get('rate'), MUTED)))
    detail = Text(rows[cursor]['detail'] if rows else 'No sensors in this profile',
                  style=MUTED, no_wrap=True, overflow=theme.OVERFLOW)
    live = sum(1 for row in rows if row.get('source') == 'sensor' and row['state'] == 'LIVE')
    sensors = sum(1 for row in rows if row.get('source') == 'sensor')
    return theme.block(Group(table, Text(''), detail), TITLES['status'], focused,
                       f'{live}/{sensors} live')
