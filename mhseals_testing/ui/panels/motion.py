"""Motion panel: braille plots of commanded and measured velocity, and the drift line."""

import math

from rich.console import Group
from rich.text import Text

from mhseals_testing.profiles import PLOT_SCALE
from mhseals_testing.ui import braille, theme
from mhseals_testing.ui.panels import TITLES
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR


AXES = (('Surge', 'm/s', 'vx'), ('Sway', 'm/s', 'vy'), ('Yaw', 'rad/s', 'wz'))
TALL = 20            # panel height at which each axis gets 4 braille rows (else 1)
SHORT = 40           # inner width below which the axis header is `Surge  +60% -> +0.37`
STALE = 2.0          # s; a measurement older than this reads as a dash


def _latest(series, now):
    return series[-1][1] if series and now - series[-1][0] <= STALE else None


def _axis(index, snapshot, view, width, rows):
    """One axis: a header row, then ``rows`` braille rows (with ±scale labels when 4)."""
    plots = snapshot.get('plots') or dict(now=0.0, window=10.0, meas={}, cmd=[])
    name, unit, key = AXES[index]
    scale = view.get('plot_scale', PLOT_SCALE)[index]
    effort = view['units'] == 'effort'
    now, window = plots['now'], plots['window']
    top, bottom = f'{scale:+g}', f'{-scale:+g}'
    gutter = max(len(top), len(bottom)) + 1 if rows > 1 else 0
    plot = braille.canvas(max(1, width - gutter), rows)
    if rows > 1:
        plot.hline(0.0, -scale, scale)
    series = plots['meas'].get(key, [])
    pred = [(t, c[index] * (scale if effort else 1.0)) for t, c in plots['cmd']]
    plot.line(pred, now - window, now, -scale, scale, 'pred')
    clipped = plot.line(series, now - window, now, -scale, scale, 'meas')
    command = view['command'][index]
    cmd = f'{command:+.0%}' if effort else f'{command:+.2f}'
    meas = _latest(series, now)
    meas = f'{meas:+.2f}' if meas is not None else theme.DASH
    marks = ''.join(sorted(clipped))
    if width < SHORT:
        head = f'{name}  {cmd} {theme.pick("→", "->")} {meas}'
    else:
        head = f'{name} {unit}{" gyro" if key == "wz" else ""}  cmd {cmd}  meas {meas}'
    header = Text(head + (' ' + marks if marks else ''), style='bold', no_wrap=True, overflow='crop')
    lines = plot.rows()
    if rows > 1:
        labels = {0: top, braille._row(0.0, -scale, scale, rows * 4) // 4: '0', rows - 1: bottom}
        lines = [Text(f'{labels.get(i, ""):>{gutter - 1}} ', style=MUTED) + line
                 for i, line in enumerate(lines)]
    return [header, *lines]


class Plots:
    """The three axes at whatever width the panel is given; ``rows`` braille rows each."""

    def __init__(self, snapshot, view, rows):
        self.snapshot, self.view, self.rows = snapshot, view, rows

    def __rich_console__(self, console, options):
        for index in range(3):
            yield from _axis(index, self.snapshot, self.view, options.max_width, self.rows)
        if self.rows > 1:
            yield Text(f"{theme.pick('━', '=')} meas  {theme.pick('╌', '-')} pred"
                       f"  -{self.snapshot.get('plots', {}).get('window', 10):g}s"
                       + ('  cmd (unscaled)' if self.view['units'] == 'effort' else ''),
                       style=MUTED, no_wrap=True, overflow='crop')


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


def motion_panel(snapshot, view, height=0):
    """Three braille axes over the drift lines; 4 rows an axis once ``height`` reaches TALL."""
    measured = snapshot['sensors'].get('odometry', {})
    subtitle = ('effort' if view['units'] == 'effort' else 'm/s') + SEPARATOR + theme.labelled(
        measured.get('status', 'MISSING'), 'odom').plain
    subtitle = Text(subtitle, style=MUTED)
    if view.get('authority'):                       # who drives, in their hue (ui U4)
        subtitle.append(SEPARATOR, style=MUTED)
        subtitle.append(view['authority'][1], style=theme.auth_style(view['authority'][0]))
    rows = 4 if height >= TALL else 1
    return theme.block(Group(Plots(snapshot, view, rows), drift_line(snapshot, view)),
                       TITLES['motion'], view['focus'] == 'motion', subtitle)
