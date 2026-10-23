"""Conditions · Command panel (tui.md 10-11, ui.md 6.3): the north-up rose and the command glyph.

Pure and ROS-free: both return Rich ``Text`` rows. The rose is a world-frame picture, so the
drift arrow never turns with the boat; only the heading tick moves. The command glyph is the
body frame (bow up), its arrow in the hue of whoever is commanding.
"""

import math
import os

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.panels import TITLES

ROSE_COLS, ROSE_ROWS = 21, 9
RADIUS = 17                 # dots; the canvas is 42 x 36 dots and dots are square
FULL_SPEED = 0.3            # m/s at the full radius; faster is clipped to the ring
STALE_S = 60.0              # an older drift estimate is drawn dim
HEADING_TICK = (0.5, 0.7)   # radial span of the heading tick, as a fraction of RADIUS
SECTORS = '↑↖←↙↓↘→↗'         # 45 degree steps counter-clockwise from ahead (ROS: +sway is left)
SECTORS_ASCII = '^<<vvv>>'


def _plain(char):
    """A braille cell as ``' '``, ``'``, ``.`` or ``:`` (upper, lower, both dots) in ASCII mode."""
    if not theme.ASCII or not '⠀' <= char <= '⣿':
        return char
    bits = ord(char) - 0x2800
    return " '.:"[bool(bits & 0x1B) + 2 * bool(bits & 0xE4)]


def _sector(surge, sway):
    """The 45 degree sector index of a (surge, sway) pair, or None when both are zero."""
    if not (surge or sway):
        return None
    return round(math.atan2(sway, surge) / (math.pi / 4)) % 8


def _polar(bearing_deg, radius):
    """Dot offset (right, down) of a compass bearing (0 = north, clockwise) at ``radius``."""
    rad = math.radians(bearing_deg)
    return radius * math.sin(rad), -radius * math.cos(rad)


def rose(drift=None, heading=None, yaw_ok=True, age=0.0):
    """The 9 x 21 rose as a list of Text rows, north up.

    ``drift`` is the world velocity (east, north) in m/s or None, ``heading`` the boat's
    compass bearing in degrees or None. The arrow is plain fg, dim when the estimate is
    older than STALE_S; the heading tick is accent, or warn while yaw is unreliable.
    """
    canvas = Canvas(ROSE_COLS, ROSE_ROWS)
    cx, cy = ROSE_COLS, ROSE_ROWS * 2  # dot centre: 21, 18

    def put(bearing, radius, layer):
        dx, dy = _polar(bearing, radius)
        canvas.dot(cx + round(dx), cy + round(dy), layer)

    for step in range(16):
        put(step * 22.5, RADIUS, 'ring')
    if heading is not None:
        for step in range(5):
            put(heading, RADIUS * (HEADING_TICK[0] + (HEADING_TICK[1] - HEADING_TICK[0]) * step / 4), 'cursor')
    if drift is not None:
        # A two dot wide line; a drift past FULL_SPEED is clipped, shown by the line meeting the ring.
        length = RADIUS * min(math.hypot(*drift) / FULL_SPEED, 1.0)
        bearing = math.degrees(math.atan2(drift[0], drift[1]))
        for step in range(int(length) + 1):
            dx, dy = _polar(bearing, step)
            x, y = cx + round(dx), cy + round(dy)
            canvas.dot(x, y, 'boat')
            canvas.dot(x + (abs(dy) >= abs(dx)), y + (abs(dy) < abs(dx)), 'boat')
    for bearing, letter in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        dx, dy = _polar(bearing, RADIUS)
        canvas.glyph((cx + round(dx)) // 2, (cy + round(dy)) // 4, letter, 'label')
    canvas.glyph(cx // 2, cy // 4, theme.pick('┼', '+'), 'ring')
    styles = {'ring': theme.MUTED, 'label': 'bold', 'boat': theme.MUTED if age > STALE_S else '',
              'cursor': theme.ACCENT if yaw_ok else theme.WARN}
    return [Text.assemble(*[(_plain(cell.text), styles.get(cell.fg, ''))
                            for cell in row]) for row in canvas.cells()]


def mode_label(chords, tmux=None, mode=None):
    """The line under the glyph: ``mode`` (``proc · C=I``, ``rc · mux``) wins outside manual."""
    if mode:
        return mode
    if chords:
        return 'chords'
    tmux = bool(os.environ.get('TMUX')) if tmux is None else tmux
    return 'single key (tmux)' if tmux else 'single key'


def command_view(state):
    """``(command, chords)`` from a ManualState; older ones only have ``event_mode``."""
    return (tuple(getattr(state, 'command', (0.0, 0.0, 0.0))),
            bool(getattr(state, 'chords', getattr(state, 'event_mode', False))))


def command_glyph(command, amplitude, chords=False, owner='keyboard', tmux=None, mode=None):
    """The body-frame hull with the command arrow and the figures below, as Text rows.

    The arrow is doubled above 40 % of ``amplitude`` and wears the hue of ``owner``; a
    yaw turn adds ``↩``/``↪`` beside the hull, bold above 50 %.
    """
    surge, sway, yaw = command
    hue = theme.AUTHORITY_HUES.get(owner, '')
    edge = theme.pick('─', '-')
    grid = [[(' ', '')] * 13 for _ in range(7)]   # col 0 and 12 are the yaw margins
    for row in range(7):
        for col in range(1, 12):
            top = row in (0, 6)
            char = (theme.pick('╭╮╰╯', '++++')[(row == 6) * 2 + (col == 11)] if top and col in (1, 11)
                    else edge if top else theme.BAR if col in (1, 11) else ' ')
            grid[row][col] = (char, theme.MUTED)
    index = _sector(surge, sway)
    if index is not None:
        arrow = theme.pick(SECTORS, SECTORS_ASCII)[index]
        grid[3][6] = (arrow, hue)
        if math.hypot(surge, sway) > 0.4 * amplitude:
            angle = index * math.pi / 4   # the quantised sector, so the copy sits behind the glyph
            grid[3 - round(math.cos(angle))][6 - round(math.sin(angle))] = (arrow, hue)
    if yaw:
        turn = theme.pick('↩' if yaw > 0 else '↪', '<' if yaw > 0 else '>')
        bold = abs(yaw) > 0.5 * amplitude
        grid[3][0 if yaw > 0 else 12] = (turn, f'{hue} bold' if bold else hue)
    lines = [Text.assemble(*cells) for cells in grid]
    figures = ' '.join(f'{value / amplitude:+.0%}' if value else '0' for value in command) if amplitude else '0 0 0'
    lines.append(Text(figures, style=theme.MUTED))
    lines.append(Text(mode_label(chords, tmux, mode), style=theme.MUTED))
    return lines


# --- the panel (ui.md 6.3) -------------------------------------------------------------

COMMAND_COLS = 14            # inner width 38 less the rose and the ' │ ' separator
NOTABLE_STATES = ('trending', 'changed')   # drift states worth a word in the subtitle
OWNERS = {'human': 'keyboard', 'ours': 'procedure', 'nav': 'nav', 'agent': 'agent', 'hold': 'hold'}
VERDICT_HUES = {'marginal': theme.WARN, 'poor': theme.BAD}   # good and unknown stay plain


def _short(value, digits=2):
    """``.12`` for 0.12: the panel drops the leading zero to fit 36 columns."""
    return f'{value:.{digits}f}'.replace('0.', '.', 1) if abs(value) < 1 else f'{value:.{digits}f}'


def _deg(rad_per_s):
    return math.degrees(rad_per_s)


def conditions_title(info, full=True):
    """The subtitle: ``good 94% · 4 min`` (plus ``· trending`` or ``· changed``), or the measuring reason."""
    verdict = info.get('verdict', 'unknown')
    if verdict == 'unknown':
        return Text(info.get('reason') or 'measuring', style=theme.MUTED)
    trust = info.get('ceiling')
    trust = info.get('drift_trust') if trust is None else trust
    title = Text.assemble((verdict, VERDICT_HUES.get(verdict, '')))
    title.append(f' {trust:.0%}' if trust is not None else '', style=theme.MUTED)
    if not full:      # the compact Cond line has no room for the minutes
        return title
    title.append(f"{theme.SEPARATOR}{info.get('neutral_s', 0) / 60:.0f} min", style=theme.MUTED)
    if info.get('state') in NOTABLE_STATES:
        title.append(f"{theme.SEPARATOR}{info['state']}", style=theme.WARN)
    return title


def _drift_figures(info):
    """``(speed m/s, bearing degrees toward)`` of the world drift, or None before one exists."""
    drift = info.get('drift')
    return None if drift is None else (math.hypot(*drift), math.degrees(math.atan2(drift[0], drift[1])) % 360)


def drift_row(info):
    """``drift .12 m/s →045° ±.02 · trust 88%``; a dash until a drift exists."""
    figures = _drift_figures(info)
    if figures is None:
        return Text(f'drift {theme.DASH}', style=theme.MUTED)
    return Text(theme.ascii_safe(f"drift {_short(figures[0])} m/s {theme.ARROW}{figures[1]:03.0f}° "
                                 f"±{_short(info['se'])}{theme.SEPARATOR}trust {info['drift_trust']:.0%}"))


def sea_row(info):
    """``floor .14°/s · sea .9°/s`` (no heave figure until an a_z source exists), or ``! yaw 4°`` in warn while yaw is unreliable."""
    if not info.get('yaw_ok', True):
        return Text(f"! yaw {info.get('yaw_note', '')}".rstrip(), style=theme.WARN)
    if info.get('floor') is None or info.get('sea') is None:
        return Text(f'floor {theme.DASH}', style=theme.MUTED)
    text = f"floor {_short(_deg(info['floor']))}°/s{theme.SEPARATOR}sea {_short(_deg(info['sea']), 1)}°/s"
    return Text(theme.ascii_safe(text))


def _command(view):
    owner = OWNERS.get((view.get('authority') or ('', ''))[0], 'none')
    lines = command_glyph(view['command'], view.get('amplitude', 1.0), view.get('chords', False), owner,
                          mode=view.get('command_mode'))
    if len(lines[-1]) > COMMAND_COLS:    # `single key (tmux)` is too wide: lose the parenthesis
        lines[-1] = Text(lines[-1].plain.split(' (')[0], style=theme.MUTED)
    # Make room for the `Command` label: interior rows 1 and 5 of the hull are never drawn on.
    return [Text('Command')] + [line for i, line in enumerate(lines) if i not in (1, 5)]


def compact_lines(view, info):
    """The 70-89 column form: ``Cond good 94% · drift .12→045° ±.02`` over ``Cmd ↑ +60% 0 0 · chords``."""
    cond = Text('Cond ', style=theme.MUTED)
    cond.append_text(conditions_title(info, full=False))
    figures = _drift_figures(info)
    if figures:
        cond.append(theme.ascii_safe(f"{theme.SEPARATOR}drift {_short(figures[0])}{theme.ARROW}"
                                     f"{figures[1]:03.0f}° ±{_short(info['se'])}"))
    if not info.get('yaw_ok', True):
        cond.append(f'{theme.SEPARATOR}! yaw', style=theme.WARN)
    glyph = command_glyph(view['command'], view.get('amplitude', 1.0), view.get('chords', False),
                          mode=view.get('command_mode'))
    index = _sector(*view['command'][:2])
    arrow = theme.pick(SECTORS, SECTORS_ASCII)[index] if index is not None else theme.MIDDOT
    return [cond, Text.assemble(('Cmd ', theme.MUTED), arrow, ' ', glyph[7], (theme.SEPARATOR, theme.MUTED), glyph[8])]


def conditions_panel(view, compact=False):
    """The rose beside the command glyph over the drift and sea rows (13 rows), or two lines when ``compact``.

    ``view['conditions']`` is ``Conditions.summary()`` plus ``heading`` (compass degrees), ``yaw_ok`` and
    ``yaw_note`` (the health ``rose`` anchor); absent means nothing measured yet.
    """
    info = view.get('conditions') or {}
    if compact:
        return theme.block(Group(*compact_lines(view, info)), TITLES['conditions'])
    rose_rows = rose(info.get('drift'), info.get('heading'), info.get('yaw_ok', True), info.get('age', 0.0))
    rows = []
    command = _command(view)
    command += [Text()] * (len(rose_rows) - len(command))
    for left, right in zip(rose_rows, command):
        right.truncate(COMMAND_COLS, pad=True)
        rows.append(Text.assemble(left, ' ', (theme.BAR, theme.MUTED), ' ', right))
    return theme.block(Group(*rows, drift_row(info), sea_row(info)), TITLES['conditions'],
                       subtitle=conditions_title(info))
