"""Conditions panel pieces (tui.md 10-11, ui.md 6.3): the north-up rose and the command glyph.

Pure and ROS-free: both return Rich ``Text`` rows. The rose is a world-frame picture, so the
drift arrow never turns with the boat; only the heading tick moves. The command glyph is the
body frame (bow up), its arrow in the hue of whoever is commanding.
"""

import math
import os

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap.canvas import Canvas

ROSE_COLS, ROSE_ROWS = 21, 9
RADIUS = 17                 # dots; the canvas is 42 x 36 dots and dots are square
FULL_SPEED = 0.3            # m/s at the full radius; faster is clipped to the ring
STALE_S = 60.0              # an older drift estimate is drawn dim
HEADING_TICK = (0.5, 0.7)   # radial span of the heading tick, as a fraction of RADIUS
SECTORS = '↑↖←↙↓↘→↗'         # 45 degree steps counter-clockwise from ahead (ROS: +sway is left)
SECTORS_ASCII = '^<<vvv>>'


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
    return [Text.assemble(*[(cell.text, styles.get(cell.fg, ''))
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
