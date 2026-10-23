"""Pure Rich rendering of the single-page dashboard (no ROS or GPIO).

``render(snapshot, view, width, height)`` draws the monitor snapshot and the
app's view state: which panel has focus, cursors, thruster state, procedures
and any popup. Tests render it to text. Layout, after cx:

  header   name, profile, host; health strip of status icons
  body     Status (sensors, frames, system) | Motion, Thrusters | Procedures, Activity
  footer   armed state, focus, notice; key hints for the focused panel
"""

import math

from rich.console import Group
from rich.layout import Layout
from rich.segment import Segment
from rich.style import Style
from rich.table import Table
from rich.text import Text

from mhseals_hardware.thruster_mixer import POSITIONS, thruster_pulses, validate_decoupling
from mhseals_testing.readiness import advisories
from mhseals_testing.ui import theme
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR

PANELS = ('status', 'motion', 'thrusters', 'procedures', 'activity')
TITLES = {'status': 'Status', 'motion': 'Motion', 'thrusters': 'Thrusters',
          'procedures': 'Procedures', 'activity': 'Activity'}
WIDE = 100          # below this the Motion panel folds away
SIDEBAR = 36
TOP_ROW = 8         # Motion and Thrusters height
NEUTRAL = 1500

# --- view models: rows the app navigates and the renderer draws ------------------


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
        rows.append(dict(group='Sensors', state=state, name=spec.name, value=value,
                         detail=f'{spec.topic}{SEPARATOR}{detail}' if detail != spec.topic
                         else spec.topic, required=spec.required))
    for chain in snapshot['tf']:
        rows.append(dict(group='Frames', state=chain['status'],
                         name=f"{chain['source']} {theme.ARROW} {chain['target']}", value='',
                         detail=chain['detail'], required=True))
    for name, level, detail in advisories(snapshot):
        rows.append(dict(group='System', state=level, name=name, value=short(detail),
                         detail=detail, required=False))
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


def result_label(result):
    label = result.get('procedure', result.get('axis', '?'))
    if 'check' in result:
        label += f" {result['check']}"
    if 'direction' in result:
        label += ' +' if result['direction'] > 0 else ' -'
    return label


def result_detail(result):
    """The one number that matters for each kind of result."""
    for key, unit in (('response', ''), ('points', ' pts'), ('overshoot', ''),
                      ('cross_track_rms', ' m xtrack'), ('rms_radius', ' m radius')):
        value = result.get(key)
        if isinstance(value, (int, float)):
            if key == 'overshoot':
                return f'{value:.0%} overshoot'
            if key == 'points':
                return f"{value}/{result.get('possible', '?')}{unit}"
            return f'{value:+.3f}{unit}' if key == 'response' else f'{value:.2f}{unit}'
    return str(result.get('detail', result.get('status', '')))


# --- panels ----------------------------------------------------------------------


def window(items, cursor, size):
    """The slice of ``items`` that keeps ``cursor`` visible in ``size`` rows."""
    if size <= 0 or len(items) <= size:
        return 0, items
    start = min(max(0, cursor - size // 2), len(items) - size)
    return start, items[start:start + size]


def status_panel(snapshot, view, height):
    rows = status_rows(snapshot, view['specs'])
    focused = view['focus'] == 'status'
    cursor = min(view['cursors'].get('status', 0), max(0, len(rows) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, {'header': '', 'justify': 'right'},
                       padding=(0, 0))
    table.show_header = False
    budget = max(1, height - 4)
    start, visible = window(rows, cursor, budget - 3)  # room for group headings
    group = None
    for index, row in enumerate(visible, start):
        if row['group'] != group:
            group = row['group']
            table.add_row(Text(group, style=f'{MUTED} bold'), '')
        name = Text(row['name'], style='bold' if row['required'] else None)
        table.add_row(theme.row(row['state'], name, focused and index == cursor),
                      Text(row['value'], style=MUTED))
    detail = Text(rows[cursor]['detail'] if rows else 'No sensors in this profile',
                  style=MUTED, no_wrap=True, overflow='ellipsis')
    live = sum(1 for row in rows if row['group'] == 'Sensors' and row['state'] == 'LIVE')
    sensors = sum(1 for row in rows if row['group'] == 'Sensors')
    return theme.block(Group(table, Text(''), detail), TITLES['status'], focused,
                       f'{live}/{sensors} live')


def sparkline(values, width=10):
    if not values:
        return Text('·' * width, style=MUTED)
    values = values[-width:]
    low, high = min(values), max(values)
    bars = '▁▂▃▄▅▆▇█'
    if high - low < 1e-3:
        return Text('─' * len(values), style=MUTED)
    return Text(''.join(bars[min(7, int(7 * (v - low) / (high - low)))] for v in values),
                style=ACCENT)


# Body frame drawn bow-up: +x (forward) is ↑, +y (port) is ←. Index by 45° sector from +x.
ARROWS = '↑↖←↙↓↘→↗'
DRIFT_DEADBAND = 0.02      # m/s; slower than this reads as no drift
DRIFT_AGREE = (0.05, 0.15)  # m/s between measured and fitted drift: good, warn, else bad


def drift_arrow(vx, vy):
    """The arrow for a body-frame horizontal velocity, or · when it is negligible."""
    if math.hypot(vx, vy) < DRIFT_DEADBAND:
        return '·'
    return ARROWS[round(math.atan2(vy, vx) / (math.pi / 4)) % 8]


def drift_vector(vx, vy, wz=0.0):
    """``↖ 0.12 m/s ↺`` : direction, speed and the turn sense of any yaw drift."""
    text = Text(drift_arrow(vx, vy), style=f'bold {ACCENT}')
    text.append(f' {math.hypot(vx, vy):.2f} m/s')
    if abs(wz) >= DRIFT_DEADBAND:
        text.append(f" {'↺' if wz > 0 else '↻'}{abs(wz):.2f}", style=MUTED)
    return text


def bearing(vx, vy):
    """Direction relative to the bow in words: 'ahead', '61° port', '120° stbd', 'astern'."""
    degrees = math.degrees(math.atan2(vy, vx))
    if abs(degrees) < 5:
        return 'ahead'
    if abs(degrees) > 175:
        return 'astern'
    return f"{abs(degrees):.0f}° {'port' if degrees > 0 else 'stbd'}"


def drift_now(snapshot):
    """The boat's body-frame velocity and heading right now, from the latest odometry."""
    values = snapshot['sensors'].get('odometry', {}).get('values', {})
    if values.get('vx') is None or values.get('vy') is None:
        return None
    return values['vx'], values['vy'], values.get('wz') or 0.0, values.get('yaw')


def expected_drift(snapshot, view, yaw):
    """What the model says the boat should do at zero effort, in the body frame.

    The mechanical bias is body-fixed; the current and wind are world-fixed,
    so they are turned into the body by the live heading. The controller's
    live estimate wins over the characterization's. Returns (source, vx, vy, wz).
    """
    characterization = view['hardware'].get('characterization', {})
    bias = characterization.get('mechanical_bias')
    fitted = characterization.get('drift', {})
    world = snapshot.get('drift')
    source = 'live' if world is not None else 'fit'
    if world is None and fitted.get('separated'):
        world = (fitted['x'], fitted['y'])
    if bias is None and world is None:
        return None
    vx, vy, wz = (bias['surge'], bias['sway'], bias['yaw']) if bias else (0.0, 0.0, 0.0)
    if world is not None and yaw is not None:
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = vx + c * world[0] + s * world[1], vy - s * world[0] + c * world[1]
    return source, vx, vy, wz


def drift_line(snapshot, view):
    """Live drift direction next to the model's, and whether they agree.

    While nothing is commanded the measured velocity is the drift; while driving it
    is shown live but not compared, since thrust is part of it.
    """
    now = drift_now(snapshot)
    driving = any(abs(value) > 1e-6 for value in view['command'])
    line = Text('Drift  ' if not driving else 'Vel    ', style=MUTED)
    if now is None:
        line.append('no odometry', style=MUTED)
    else:
        line.append_text(drift_vector(*now[:3]))
        if math.hypot(now[0], now[1]) >= DRIFT_DEADBAND:
            line.append(f'  {bearing(now[0], now[1])}', style=MUTED)
    expected = expected_drift(snapshot, view, now[3] if now else None)
    if expected is None:
        line.append('\n       fit ', style=MUTED)
        line.append('not characterized', style=MUTED)
        return line
    source, *fit = expected
    line.append(f'\n       {source} ', style=MUTED)
    line.append_text(drift_vector(*fit))
    if now is not None and not driving:
        error = math.hypot(now[0] - fit[0], now[1] - fit[1])
        state = 'good' if error <= DRIFT_AGREE[0] else 'warn' if error <= DRIFT_AGREE[1] else 'bad'
        line.append('  ')
        line.append_text(theme.labelled(state, f'Δ {error:.2f}', MUTED))
    return line


def motion_panel(snapshot, view):
    effort = view['units'] == 'effort'
    measured = snapshot['sensors'].get('odometry', {})
    values = measured.get('values', {})
    table = theme.grid('', {'header': 'Cmd', 'justify': 'right'},
                       {'header': 'Meas', 'justify': 'right'}, {'header': '', 'style': MUTED},
                       'Trend', padding=(0, 1))
    for axis, key, unit in (('Surge', 'vx', 'm/s'), ('Sway', 'vy', 'm/s'), ('Yaw', 'wz', 'rad/s')):
        command = view['command'][('vx', 'vy', 'wz').index(key)]
        table.add_row(axis, f'{command:+.0%}' if effort else f'{command:+.2f}',
                      f'{values[key]:+.2f}' if values.get(key) is not None else '—', unit,
                      sparkline(snapshot.get('trends', {}).get(key, [])))
    subtitle = ('effort' if effort else 'm/s') + SEPARATOR + theme.labelled(
        measured.get('status', 'MISSING'), 'odom').plain
    return theme.block(Group(table, drift_line(snapshot, view)), TITLES['motion'],
                       view['focus'] == 'motion', subtitle)


def effort_bar(pulse, testing=False, width=9):
    """A centred bar: left of the tick is reverse, right is forward."""
    effort = max(-1.0, min(1.0, (pulse - NEUTRAL) / 400))
    half = width // 2
    cells = [' '] * width
    cells[half] = '│'
    for step in range(1, round(abs(effort) * half) + 1):
        cells[half + step if effort > 0 else half - step] = '█'
    return Text(''.join(cells), style=theme.WARN if testing else ACCENT)


def thruster_rows(view):
    """Per position: physical output, reversal and the pulse it gets now."""
    config = view['draft'] or view['hardware']
    channel_map, reversed_ = config['channel_map'], config.get('reversed', [False] * 4)
    if view['test_pulses'] is not None:
        by_output = view['test_pulses']
    elif view['units'] == 'effort' and view['armed']:
        by_output = thruster_pulses(view['command'], view['matrix'], channel_map, reversed_,
                                    validate_decoupling(view['hardware']['decoupling']))
    else:
        by_output = [NEUTRAL] * 4
    saved = view['hardware']
    return [dict(position=position.upper(), output=output, reversed=flip,
                 pulse=by_output[output - 1], readback=view['pwm_readback'][output - 1],
                 changed=(output != saved['channel_map'][index]
                          or flip != saved['reversed'][index]))
            for index, (position, output, flip) in enumerate(
                zip(POSITIONS, channel_map, reversed_))]


def thrusters_panel(view):
    focused = view['focus'] == 'thrusters'
    cursor = view['cursors'].get('thrusters', 0)
    testing = view['test_pulses'] is not None
    columns = ['    Thr', 'Out', 'Rev', {'header': 'Pulse', 'justify': 'right'}, 'Effort']
    if view['outputs'] == 'odroid':
        columns.append('Pin')
    table = theme.grid(*columns, padding=(0, 1))
    for index, row in enumerate(thruster_rows(view)):
        active = view['armed'] and row['pulse'] != NEUTRAL
        state = ('WARN' if testing else 'LIVE') if active else ('IDLE' if view['armed'] else 'OFF')
        grabbed = view['grabbed'] == index
        label = Text(row['position'] + (' ⇅' if grabbed else ''))
        cells = [theme.row(state, label, focused and index == cursor),
                 Text(str(row['output']), style=f'bold {theme.WARN}' if row['changed'] else None),
                 Text('↺' if row['reversed'] else '·', style=theme.WARN if row['reversed'] else MUTED),
                 Text(f"{row['pulse']} µs", style='bold' if active else MUTED),
                 effort_bar(row['pulse'], testing)]
        if view['outputs'] == 'odroid':
            cells.append(Text(row['readback'], style=MUTED))
        table.add_row(*cells)
    subtitle = ('testing' if testing else view['outputs']) + SEPARATOR + (
        'unsaved' if view['draft'] else view['config_note'])
    return theme.block(table, TITLES['thrusters'], focused, subtitle)


def procedures_panel(view, height):
    focused = view['focus'] == 'procedures'
    items = view['procedures']
    cursor = min(view['cursors'].get('procedures', 0), max(0, len(items) - 1))
    table = theme.grid({'header': '', 'ratio': 1}, padding=(0, 0))
    table.show_header = False
    start, visible = window(items, cursor, max(1, height - 4))
    for index, item in enumerate(visible, start):
        state = ('RUNNING' if item['title'] == view['running'] else
                 'READY' if not item['reason'] else 'OFF')
        table.add_row(theme.row(state, item['title'], focused and index == cursor,
                                Text(item['key'], style=f'bold {ACCENT}')))
    current = items[cursor] if items else None
    detail = Text((current['reason'] or current['description']) if current else '',
                  style=MUTED, no_wrap=True, overflow='ellipsis')
    ready = sum(1 for item in items if not item['reason'])
    return theme.block(Group(table, Text(''), detail), TITLES['procedures'], focused,
                       f'{ready} ready')


def activity_panel(snapshot, view, height):
    focused = view['focus'] == 'activity'
    cursor = view['cursors'].get('activity', 0)
    budget = max(1, height - 3)
    if view['activity'] == 'log':
        table = theme.grid({'header': '  Event', 'ratio': 1}, {'header': 'Age', 'justify': 'right'},
                           padding=(0, 1))
        events = list(reversed(snapshot['events']))
        start, visible = window(events, cursor, budget)
        for index, (stamp, message) in enumerate(visible, start):
            table.add_row(theme.row(None, message, focused and index == cursor),
                          Text(f'{view["now"] - stamp:.0f}s', style=MUTED))
        body = table if events else Text('No events yet', style=MUTED)
        return theme.block(body, TITLES['activity'], focused, 'log')
    table = theme.grid({'header': '    Procedure', 'ratio': 3},
                       {'header': 'Result', 'width': 11},
                       {'header': 'Detail', 'ratio': 3}, padding=(0, 1))
    results = list(reversed(snapshot['results']))
    start, visible = window(results, cursor, budget)
    for index, result in enumerate(visible, start):
        verdict = str(result.get('verdict', '—'))
        table.add_row(theme.row(verdict, result_label(result), focused and index == cursor),
                      Text(verdict.lower()), Text(result_detail(result), style=MUTED))
    body = table if results else Text(f'No results yet{SEPARATOR}Enter on a procedure runs it',
                                      style=MUTED)
    return theme.block(body, TITLES['activity'], focused, f'results{SEPARATOR}{len(results)}')


# --- header, footer, popups ------------------------------------------------------


def health_strip(snapshot, view):
    rows = status_rows(snapshot, view['specs'])
    sensors = [row for row in rows if row['group'] == 'Sensors' and row['required']]
    sensors_ok = all(row['state'] == 'LIVE' for row in sensors)
    frames_ok = all(row['state'] == 'LIVE' for row in rows if row['group'] == 'Frames')
    parts = [theme.labelled('LIVE' if view['recording'] else 'FAIL', 'rec'),
             theme.labelled('LIVE' if sensors_ok else 'FAIL',
                            f"sensors {sum(r['state'] == 'LIVE' for r in sensors)}/{len(sensors)}"),
             theme.labelled('LIVE' if frames_ok else 'FAIL', 'tf')]
    for row in rows:
        if row['group'] == 'System' and row['state'] != 'OK':
            parts.append(theme.labelled(row['state'], f"{row['name']} {row['value']}".strip()))
    if view['bench_override']:
        parts.append(theme.labelled('WARN', 'bench override'))
    return Text('   ').join(parts)


def arm_state(armed, detail=None):
    """Armed or disarmed, drawn like every other status: a colored dot and a label."""
    text = Text('● ', style=theme.BAD if armed else theme.GOOD)
    text.append('Armed' if armed else 'Disarmed', style='bold')
    if detail:
        text.append(SEPARATOR + detail, style=MUTED)
    return text


def header(snapshot, view):
    top = Table.grid(expand=True)
    top.add_column()
    top.add_column(justify='right')
    title = Text(' boat_test ', style=f'bold {ACCENT}')
    title.append(f" {view['profile']}", style='bold')
    right = Text(f"{view['host']}{SEPARATOR}ROS {view['domain']}  ", style=MUTED)
    right.append_text(arm_state(view['armed']))
    top.add_row(title, right)
    strip = Text(' ')
    strip.append_text(health_strip(snapshot, view))
    return Group(top, strip)


def footer(view, width):
    status = Table.grid(expand=True, padding=(0, 1))
    status.add_column(ratio=3, no_wrap=True)
    status.add_column(ratio=3, no_wrap=True)
    status.add_column(ratio=4, justify='right', no_wrap=True, overflow='ellipsis')
    position = view['position']
    focus = Text(f"Focus: {TITLES[view['focus']]}", style=f'bold {ACCENT}')
    if position:
        focus.append(f'{SEPARATOR}{position}', style=MUTED)
    status.add_row(arm_state(view['armed'], view['outputs'] if view['armed'] else None),
                   focus, Text(view['notice'], style=MUTED))
    hints = view['hints']
    columns = 4 if width >= WIDE else 3
    grid = Table.grid(expand=True)
    for _ in range(columns):
        grid.add_column(ratio=1, no_wrap=True, overflow='ellipsis')
    hints = hints[:columns * 2]
    for row in range(2):
        cells = [theme.hint(*hint) for hint in hints[row * columns:(row + 1) * columns]]
        grid.add_row(*(cells + [''] * (columns - len(cells))))
    return Group(status, grid)


def popup(modal, width):
    """A centred dialog: message, list of choices, or a text field."""
    body = []
    if modal.get('message'):
        body.append(Text(modal['message']))
    if modal.get('rows'):
        rows = theme.grid({'header': '', 'style': f'bold {ACCENT}'},
                          {'header': '', 'no_wrap': False, 'overflow': 'fold', 'ratio': 1},
                          padding=(0, 2))
        rows.show_header = False
        for label, value in modal['rows']:
            rows.add_row(str(label), str(value))
        body.append(rows)
    if modal['kind'] == 'choice':
        if body:
            body.append(Text(''))
        for index, option in enumerate(modal['options']):
            key = modal.get('keys', [None] * len(modal['options']))[index]
            line = Text(f'{key} ' if key else '', style=f'bold {ACCENT}')
            line.append_text(Text.from_markup(option) if '[' in option else Text(option))
            body.append(theme.selected(line, index == modal['selected']))
    elif modal['kind'] == 'input':
        body += [Text(''), Text('> ', style=f'bold {ACCENT}') + Text(modal['text'] + '█')]
    if modal.get('hint'):
        body += [Text(''), Text(modal['hint'], style=MUTED)]
    panel = theme.block(Group(*body), modal['title'], True)
    if modal.get('danger'):
        panel.border_style = theme.BAD
    return panel, min(width - 4, modal.get('width', 64))


class Overlay:
    """Draw ``top`` centred over ``base``, dimming the base like a modal backdrop."""

    def __init__(self, base, top, top_width, width, height):
        self.base, self.top, self.top_width = base, top, top_width
        self.width, self.height = width, height

    def __rich_console__(self, console, options):
        options = options.update(width=self.width, height=self.height)
        lines = console.render_lines(self.base, options, pad=True)
        top = console.render_lines(self.top, options.update(width=self.top_width, height=None))
        dim = Style(dim=True)
        lines = [list(Segment.apply_style(line, post_style=dim)) for line in lines]
        x = (self.width - self.top_width) // 2
        y = max(0, (self.height - len(top)) // 2)
        for offset, row in enumerate(top[:self.height]):
            line = lines[y + offset]
            left, _, right = Segment.divide(line, [x, x + self.top_width, self.width])
            lines[y + offset] = left + row + right
        for line in lines:
            yield from line
            yield Segment.line()


# --- the page --------------------------------------------------------------------


def render(snapshot, view, width=120, height=34):
    body_height = max(5, height - 5)
    layout = Layout()
    layout.split_column(Layout(header(snapshot, view), size=2), Layout(name='body'),
                        Layout(footer(view, width), size=3))
    body = layout['body']
    sidebar = SIDEBAR if width >= WIDE else 30
    body.split_row(Layout(status_panel(snapshot, view, body_height), size=sidebar),
                   Layout(name='main'))
    main = body['main']
    lower = body_height - TOP_ROW
    if width >= WIDE:
        main.split_column(Layout(name='top', size=TOP_ROW), Layout(name='bottom'))
        main['top'].split_row(Layout(motion_panel(snapshot, view)),
                              Layout(thrusters_panel(view)))
        main['bottom'].split_row(Layout(procedures_panel(view, lower), size=SIDEBAR),
                                 Layout(activity_panel(snapshot, view, lower)))
    else:
        half = max(4, (body_height - TOP_ROW) // 2)
        main.split_column(Layout(thrusters_panel(view), size=TOP_ROW),
                          Layout(procedures_panel(view, half), size=half),
                          Layout(activity_panel(snapshot, view, body_height - TOP_ROW - half)))
    if view.get('modal'):
        panel, panel_width = popup(view['modal'], width)
        return Overlay(layout, panel, panel_width, width, height)
    return layout
