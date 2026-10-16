"""Pure Rich rendering of the test-suite dashboard (no ROS or GPIO).

``render_dashboard(snapshot, context)`` draws a monitor snapshot plus UI
context (profile, view, arming state, command); tests render it to text.
"""

from pathlib import PurePath
from rich.console import Group
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from mhseals_hardware.thruster_mixer import POSITIONS, mix_thrusters
from mhseals_testing.readiness import advisories

VIEWS = ('overview', 'sensors', 'tf', 'health', 'mapping', 'results', 'logs')
COLORS = {'LIVE': 'green', 'OK': 'green', 'WARN': 'yellow', 'STALE': 'yellow',
          'MISSING': 'red', 'FAIL': 'red', 'N/A': 'dim'}


MANUAL_KEYS = 'WASD move  Arrows surge/yaw  +/- scale\nSpace STOP  M/X/Esc return to dashboard'
DASHBOARD_KEYS = 'Tab views  H arm/disarm  M manual  P procedures  +/- scale\nE export  Space STOP  Q quit'


def badge(status):
    return Text(status, style=COLORS.get(status, 'cyan'))


def sensor_table(snapshot, specs, compact=False):
    table = Table(expand=True, title='Received sensor data', padding=(0, 1))
    for column in (('Sensor', 'State', 'Hz') if compact else
                   ('Sensor / topic', 'State', 'Hz', 'Age', 'Details')):
        table.add_column(column, no_wrap=True, overflow='ellipsis')
    for spec in specs:
        value = snapshot['sensors'].get(spec.name, {})
        status = value.get('status', 'MISSING')
        name_text = spec.name + (' *' if spec.required else '')
        row = [Text(name_text if compact else name_text + '\n' + spec.topic),
               badge(status), f"{value.get('hz', 0):.1f}"]
        if not compact:
            values = value.get('values', {})
            detail = value.get('issue') or values.get('summary', '')
            row += [f"{value['age']:.2f}s" if 'age' in value else '--', Text(detail)]
        table.add_row(*row)
    return table


def motion_panel(snapshot, context):
    command = context.get('command', (0, 0, 0))
    odom = snapshot['sensors'].get('odometry', {})
    measured = odom.get('values', {})
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_row('', 'SURGE +forward', 'SWAY +port', 'YAW +CCW')
    if context.get('units', 'effort') == 'effort':
        table.add_row('Effort command', *(f'{v:+.0%}' for v in command))
    else:
        table.add_row('Velocity command', *(f'{v:+.2f}' for v in command))
    table.add_row('Measured', *(f'{measured[k]:+.3f}' if measured.get(k) is not None else '--'
                               for k in ('vx', 'vy', 'wz')))
    table.add_row('Units', 'm/s', 'm/s', 'rad/s')

    def trend(key):
        values = snapshot.get('trends', {}).get(key, [])
        if not values:
            return '--'
        values = values[::max(1, len(values) // 12)][-12:]
        low, high = min(values), max(values)
        return ('─' * len(values) if high - low < .001 else
                ''.join('▁▂▃▄▅▆▇█'[min(7, int(7 * (v-low)/(high-low)))] for v in values))
    table.add_row('Recent trend*', *(trend(key) for key in ('vx', 'vy', 'wz')))
    frame = measured.get('child', 'unknown')
    title = ('Motion — effort is NOT velocity' if context.get('units', 'effort') == 'effort'
             else 'Motion — velocity command')
    return Panel(Group(table, Text(f"Odometry: {odom.get('status', 'MISSING')} | frame: {frame}"
                                   ' | *auto-scaled')),
                 title=title, border_style='cyan')


def mapping_panel(context, compact=False):
    mapping = context.get('channel_map', (1, 2, 3, 4))
    command = context.get('command', (0, 0, 0))
    # The mixer maps effort; a velocity command has no direct thruster prediction.
    outputs = (mix_thrusters(*command, mixer=context['matrix'])
               if context.get('units', 'effort') == 'effort' else [None] * 4)
    labels = {position: f'{position.upper()} → #{channel}  '
                        + (f'{output:+.0%}' if output is not None else ' -- ')
              for position, channel, output in zip(POSITIONS, mapping, outputs)}
    hull = Text(f"           BOW / +surge\n{labels['fl']}     {labels['fr']}\n"
                f"     │                   │\n{labels['rl']}     {labels['rr']}\n"
                'Predicted mixer effort; not motor feedback', style='cyan')
    if compact:
        return Panel(hull, title='Thruster map', border_style='blue')
    table = Table(expand=True)
    for name in ('Position', 'Output', 'PWM chip', 'Channel', 'Readback'):
        table.add_column(name, no_wrap=True, overflow='ellipsis')
    chips, channels = context.get('chips'), context.get('channels')
    for position, channel in zip((p.upper() for p in POSITIONS), mapping):
        readback = context.get('pwm_readback', ['not sampled'] * 4)[channel - 1]
        if chips:
            path = PurePath(chips[channel - 1])
            chip = next((part for part in path.parts if part.endswith('.pwm')), path.name)
            pin = str(channels[channel - 1])
        else:  # simulator or attach-only: no pins to show
            chip, pin = context.get('outputs', 'none'), '--'
        table.add_row(position, str(channel), Text(chip), pin, Text(readback))
    return Group(Panel(hull, title='Thruster mapping'), table,
                 Text('Mapping comes from the hardware config; '
                      'run thruster_identify to change it.', style='dim'))


def tf_table(snapshot):
    table = Table(expand=True, title='TF chain integrity')
    for column in ('Required chain', 'State', 'Details'):
        table.add_column(column)
    for edge in snapshot['tf']:
        table.add_row(Text(edge['source'] + ' → ' + edge['target']),
                      badge(edge['status']), Text(edge['detail']))
    return Group(table, Text('Checks freshness, reachability, cycles and competing publishers.\n'
                             'A green chain does not verify physical mounting, axes or calibration.',
                             style='dim'))


def advisory_table(snapshot, compact=False):
    table = Table(expand=True, title='Advisories (never block motion)', padding=(0, 1))
    for column in (('Check', 'State') if compact else ('Check', 'State', 'Details')):
        table.add_column(column, no_wrap=True, overflow='ellipsis')
    for name, level, detail in advisories(snapshot):
        table.add_row(name, badge(level), *(() if compact else (Text(detail),)))
    return table


def result_table(snapshot):
    table = Table(expand=True, title='Results (newest first)')
    for column in ('Procedure', 'Verdict', 'Samples', 'Baseline', 'Mean', 'Response'):
        table.add_column(column)
    for result in reversed(snapshot['results'][-10:]):
        label = result.get('procedure', result.get('axis', '?'))
        if 'check' in result:
            label += ': ' + result['check']
        if 'direction' in result:
            label += ' +' if result['direction'] > 0 else ' -'
        table.add_row(label, Text(result.get('verdict', '--')), str(result.get('samples', '--')),
                      *(f'{result[k]:+.3f}' if k in result else '--'
                        for k in ('baseline', 'mean', 'response')))
    return Group(table, Text('Response = mean velocity minus pre-test drift. SIGN MATCH is not '
                             'calibration.\nSurge/sway: m/s. Yaw: rad/s. The session report has '
                             'every detail.'))


def render_dashboard(snapshot, context, width=100, height=32):
    view = context.get('view', 'overview')
    armed = context.get('armed', False)
    header = Text(f" {context.get('profile', 'boat').upper()}  {context.get('host', '')}  "
                  f"ROS {context.get('domain', '?')}   ")
    header.append('ARMED' if armed else 'DISARMED / MONITOR', style='bold red' if armed else 'bold green')
    header.append(f"   BAG: {context.get('bag', 'OFF')}\n ")
    for name in VIEWS:
        header.append(f' {name.upper()} ', style='bold black on cyan' if name == view else 'dim')
    specs = context['specs']
    if view == 'sensors':
        body = sensor_table(snapshot, specs)
    elif view == 'tf':
        body = tf_table(snapshot)
    elif view == 'health':
        body = advisory_table(snapshot)
    elif view == 'mapping':
        body = mapping_panel(context)
    elif view == 'results':
        body = result_table(snapshot)
    elif view == 'logs':
        body = Group(Text('Recent events', style='bold'),
                     *(Text(message) for _, message in snapshot['events']),
                     Text('\nProcesses / logs\n' + context.get('process_summary', 'None')))
    else:
        right = Group(motion_panel(snapshot, context), mapping_panel(context, compact=True))
        if width >= 100:
            left = Group(sensor_table(snapshot, specs, compact=True),
                         advisory_table(snapshot, compact=True))
            body = Layout()
            body.split_row(Layout(left, ratio=1),
                           Layout(Text(''), size=1),
                           Layout(right, ratio=2))
        else:
            states = '  '.join(f"{spec.name}: "
                               f"{snapshot['sensors'].get(spec.name, {}).get('status', 'MISSING')}"
                               for spec in specs if spec.required)
            body = Group(Text(states), right)
    tf_ok = all(row['status'] == 'LIVE' for row in snapshot['tf'])
    override = 'BENCH OVERRIDE | ' if context.get('bench_override') else ''
    footer = Text(f"TF: {'LIVE' if tf_ok else 'CHECK'} | {override}{context.get('notice', 'Ready')}\n",
                  no_wrap=True, overflow='ellipsis')
    footer.append(MANUAL_KEYS if context.get('manual') else context.get('keys', DASHBOARD_KEYS),
                  style='bold')
    layout = Layout()
    layout.split_column(Layout(header, size=3), Layout(body), Layout(footer, size=4))
    return layout
