"""Motion panel: commanded and measured velocity, trends and the drift line."""

import math

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR


def sparkline(values, width=10):
    if not values:
        return Text(theme.MIDDOT * width, style=MUTED)
    values = values[-width:]
    low, high = min(values), max(values)
    bars = theme.SPARK
    if high - low < 1e-3:
        return Text(theme.RULE * len(values), style=MUTED)
    return Text(''.join(bars[min(7, int(7 * (v - low) / (high - low)))] for v in values),
                style=ACCENT)


# Body frame drawn bow-up: +x (forward) is ↑, +y (port) is ←. Index by 45° sector from +x.
ARROWS = theme.pick('↑↖←↙↓↘→↗', '^\\</v\\>/')
DRIFT_DEADBAND = 0.02      # m/s; slower than this reads as no drift
DRIFT_AGREE = (0.05, 0.15)  # m/s between measured and fitted drift: good, warn, else bad


def drift_arrow(vx, vy):
    """The arrow for a body-frame horizontal velocity, or · when it is negligible."""
    if math.hypot(vx, vy) < DRIFT_DEADBAND:
        return theme.MIDDOT
    return ARROWS[round(math.atan2(vy, vx) / (math.pi / 4)) % 8]


def drift_vector(vx, vy, wz=0.0):
    """``↖ 0.12 m/s ↩`` : direction, speed and the turn sense of any yaw drift."""
    text = Text(drift_arrow(vx, vy), style=f'bold {ACCENT}')
    text.append(f' {math.hypot(vx, vy):.2f} m/s')
    if abs(wz) >= DRIFT_DEADBAND:
        text.append(f" {theme.pick('↩', '<') if wz > 0 else theme.pick('↪', '>')}{abs(wz):.2f}", style=MUTED)
    return text


def bearing(vx, vy):
    """Direction relative to the bow in words: 'ahead', '61° port', '120° stbd', 'astern'."""
    degrees = math.degrees(math.atan2(vy, vx))
    if abs(degrees) < 5:
        return 'ahead'
    if abs(degrees) > 175:
        return 'astern'
    return f"{abs(degrees):.0f}{theme.pick('°', ' deg')} {'port' if degrees > 0 else 'stbd'}"


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
                       {'header': 'Trend', 'no_wrap': False, 'overflow': 'crop'},  # shrinks first
                       padding=(0, 1))
    for axis, key, unit in (('Surge', 'vx', 'm/s'), ('Sway', 'vy', 'm/s'), ('Yaw', 'wz', 'rad/s')):
        command = view['command'][('vx', 'vy', 'wz').index(key)]
        table.add_row(axis, f'{command:+.0%}' if effort else f'{command:+.2f}',
                      f'{values[key]:+.2f}' if values.get(key) is not None else theme.DASH, unit,
                      sparkline(snapshot.get('trends', {}).get(key, [])))
    subtitle = ('effort' if effort else 'm/s') + SEPARATOR + theme.labelled(
        measured.get('status', 'MISSING'), 'odom').plain
    return theme.block(Group(table, drift_line(snapshot, view)), TITLES['motion'],
                       view['focus'] == 'motion', subtitle)
