"""Motion panel: braille plots of commanded and measured velocity (drift lives in the Conditions panel)."""

from rich.text import Text

from mhseals_testing.profiles import PLOT_SCALE
from mhseals_testing.ui import braille, theme
from mhseals_testing.ui.panels import TITLES


AXES = (('Surge', 'm/s', 'vx'), ('Sway', 'm/s', 'vy'), ('Yaw', 'rad/s', 'wz'))
TALL = 20            # panel height at which each axis gets 4 braille rows (else 1)
SHORT = 40           # inner width below which the axis header is `Surge  +60% -> +0.37`
_HUE = {'green': 'GOOD', 'yellow': 'WARN', 'red': 'BAD', None: 'MUTED'}   # track score hues
STALE = 2.0          # s; a measurement older than this reads as a dash


def _latest(series, now):
    return series[-1][1] if series and now - series[-1][0] <= STALE else None


def _traces(index, snapshot, view, width, rows):
    """The braille canvas of one axis in ``width`` columns, and what it clipped, scale and latest measurement."""
    plots = snapshot.get('plots') or dict(now=0.0, window=10.0, meas={}, cmd=[])
    key = AXES[index][2]
    scale = view.get('plot_scale', PLOT_SCALE)[index]
    effort = view['units'] == 'effort'
    now, window = plots['now'], plots['window']
    gutter = max(len(f'{scale:+g}'), len(f'{-scale:+g}')) + 1 if rows > 1 else 0
    plot = braille.canvas(max(1, width - gutter), rows, name=f'motion.{key}.{rows}')
    if rows > 1:
        plot.hline(0.0, -scale, scale)
    series = plots['meas'].get(key, [])
    tracking = view.get('tracking')
    pred = tracking['pred'][index] if tracking and tracking['pred'][index] else [
        (t, c[index] * (scale if effort else 1.0)) for t, c in plots['cmd']]   # the command until a model runs
    plot.line(pred, now - window, now, -scale, scale, 'pred')
    clipped = plot.line(series, now - window, now, -scale, scale, 'meas')
    return plot, clipped, scale, gutter, _latest(series, now)


def _axis(index, snapshot, view, width, rows):
    """One axis: a header row, then ``rows`` braille rows (with ±scale labels when 4)."""
    name, unit, key = AXES[index]
    plot, clipped, scale, gutter, meas = _traces(index, snapshot, view, width, rows)
    top, bottom = f'{scale:+g}', f'{-scale:+g}'
    effort = view['units'] == 'effort'
    tracking = view.get('tracking')
    command = view['command'][index]
    cmd = f'{command:+.0%}' if effort else f'{command:+.2f}'
    meas = f'{meas:+.2f}' if meas is not None else theme.DASH
    marks = ''.join(sorted(clipped))
    arrow = theme.pick('→', '->')
    percent = theme.DASH if not tracking or tracking['score'][index] is None else f"{tracking['score'][index]:.0%}"
    header = Text(f'{name}', style='bold', no_wrap=True, overflow='crop')
    if width < SHORT:      # U14: shrink in order: drop the score, then the arrow and cmd; meas always shows whole
        hue = getattr(theme, _HUE[tracking['colour'][index]]) if tracking else ''
        tiers = ([[(f' {percent}   ', hue), (f'{cmd} {arrow} {meas}', 'bold')]] if tracking else []) + [
            [('  ', ''), (f'{cmd} {arrow} {meas}', 'bold')], [(' ', ''), (f'{cmd} {arrow} {meas}', 'bold')],
            [(' ', ''), (meas, 'bold')]]
        for tier in tiers:
            if len(name) + sum(len(t) for t, _ in tier) <= width or tier is tiers[-1]:
                break
        for part, style in tier:
            header.append(part, style=style)
    else:
        parts = [(f'{unit}{"" if tracking or key != "wz" else " gyro"}', 'bold')]    # gyro yields to the score
        if tracking:
            parts.append((f'track {percent}', getattr(theme, _HUE[tracking['colour'][index]])))
        parts += [(f'cmd {cmd}', 'bold'), (f'meas {meas}', 'bold')]
        gap = '  ' if len(name) + 1 + sum(len(t) for t, _ in parts) + 2 * (len(parts) - 1) <= width else ' '   # tighten before cropping
        for n, (part, style) in enumerate(parts):
            header.append(' ' if n == 0 else gap)
            header.append(part, style=style)
    header.append(' ' + marks if marks else '', style='bold')
    lines = plot.rows()
    if rows > 1:
        labels = {0: top, braille._row(0.0, -scale, scale, rows * 4) // 4: '0', rows - 1: bottom}
        lines = [Text(f'{labels.get(i, ""):>{gutter - 1}} ', style=theme.MUTED) + line
                 for i, line in enumerate(lines)]
    return [header, *lines]


LABEL = 18           # columns of a compact row's label: name, command, measurement


class Compact:
    """The three axes as one row each."""

    def __init__(self, snapshot, view):
        self.snapshot, self.view = snapshot, view

    def __rich_console__(self, console, options):
        for index in range(3):
            yield compact_axis(index, self.snapshot, self.view, options.max_width)


class Plots:
    """The three axes at whatever width the panel is given; ``rows`` braille rows each."""

    def __init__(self, snapshot, view, rows):
        self.snapshot, self.view, self.rows = snapshot, view, rows

    def __rich_console__(self, console, options):
        for index in range(3):
            yield from _axis(index, self.snapshot, self.view, options.max_width, self.rows)
        if self.rows > 1:
            tracking = self.view.get('tracking')
            model = tracking and any(tracking['pred'])
            yield Text(f"{theme.pick('━', '=')} meas  {theme.pick('╌', '-')} {'model' if model else 'pred'}"
                       f"  -{self.snapshot.get('plots', {}).get('window', 10):g}s"
                       + ('  cmd (unscaled)' if not model and self.view['units'] == 'effort' else ''),
                       style=theme.MUTED, no_wrap=True, overflow='crop')


def compact_axis(index, snapshot, view, width):
    """One row: ``Surge +60% +0.37`` and the braille trace in what is left (the 70-89 column form)."""
    plot, _, _, _, meas = _traces(index, snapshot, view, max(1, width - LABEL), 1)
    command = view['command'][index]
    cmd = f'{command:+.0%}' if view['units'] == 'effort' else f'{command:+.2f}'
    meas = f'{meas:+.2f}' if meas is not None else theme.DASH
    label = Text(f'{AXES[index][0]:<5} {cmd:>5} {meas:>5} ', style='bold', no_wrap=True, overflow='crop')
    return label + plot.rows()[0]


def motion_panel(snapshot, view, height=0, compact=False):
    """Three braille axes; 4 rows an axis once ``height`` reaches TALL, or ``compact``: one row an axis."""
    measured = snapshot['sensors'].get('odometry', {})
    units = 'effort' if view['units'] == 'effort' else 'm/s'
    subtitle = units + theme.SEPARATOR + theme.labelled(
        measured.get('status', 'MISSING'), 'odom').plain
    subtitle = Text(subtitle, style=theme.MUTED)
    if view.get('authority'):                       # who drives, in their hue (ui U4)
        subtitle.append(theme.SEPARATOR, style=theme.MUTED)
        subtitle.append(view['authority'][1], style=theme.auth_style(view['authority'][0]))
    rows = 4 if height >= TALL else 1
    body = Compact(snapshot, view) if compact else Plots(snapshot, view, rows)
    return theme.block(body,
                       TITLES['motion'], view['focus'] == 'motion', subtitle)
