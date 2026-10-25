"""Conditions · Command panel (tui-visual 3): one north-up rose, one command arrow, labelled bars.

Pure and ROS-free. The rose is a world-frame picture: the hull (the map's pentagon) turns to the
heading, the command is one arrow icon at its bearing in the hue of whoever drives (16 bearings,
hand-drawn in braille), yaw is ↺ or ↻ in the top-right corner and the world drift a dashed muted line. The
right column names the driver, then surge, sway and yaw as bipolar bars, then the heading.
"""

import math
import os
from functools import lru_cache

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap import hull
from mhseals_testing.ui.panels import TITLES

ROSE_COLS, ROSE_ROWS = 21, 9
RADIUS = 17                 # dots; the canvas is 42 x 36 dots and dots are square
CENTRE = (ROSE_COLS, ROSE_ROWS * 2)
HULL_DOTS = (9, 6)          # length, beam of the rose's hull
REACH = (HULL_DOTS[0] / 2 + 1, HULL_DOTS[1] / 2 + 1)   # the command line starts this far out
ICON_REACH = RADIUS - 3     # dots; no command-icon dot lies farther out, clear of the ticks and letters
FULL_SPEED = 0.3            # m/s at the full radius; faster is clipped to the ring
STALE_S = 60.0              # an older drift estimate is drawn dim
ARROWS = '↑↗→↘↓↙←↖'         # screen directions, clockwise from up
ARROWS_ASCII = '^/>\\v/<\\'
# The command icon: a filled head on a short stem, in braille dots (x right, y down) about its middle,
# hand-drawn for 0, 22.5 and 45 deg; the other 13 bearings are exact flips and transposes of these
# (dots are square), so all 16 have one weight. ICON_SYM maps a master's offsets onto a bearing.
ICONS = {
    0: ((0, -3), (-1, -2), (0, -2), (1, -2), (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1),
        (0, 0), (0, 1), (0, 2), (0, 3)),
    1: ((1, -3), (-1, -2), (0, -2), (1, -2), (0, -1), (1, -1), (2, -1), (0, 0), (0, 1), (-1, 2), (-1, 3)),
    2: ((3, -3), (1, -3), (2, -3), (3, -2), (3, -1), (2, -2), (1, -1), (0, 0), (-1, 1), (-2, 2)),
}
ICON_SYM = ((1, 0, 0, 1), (-1, 0, 0, 1), (1, 0, 0, -1), (-1, 0, 0, -1),
            (0, -1, -1, 0), (0, 1, -1, 0), (0, -1, 1, 0), (0, 1, 1, 0))   # (x, y) -> (ax + by, cx + dy)
SPIN = ('↺', '↻')           # +yaw turns counter-clockwise (REP 103); drawn in the top-right corner
SPIN_ASCII = ('ccw', 'cw')
POINTS = ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW')
EIGHTHS = ' ▏▎▍▌▋▊▉█'       # left-aligned fills; a bar growing left shows them in reverse video
BITS = {(0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (0, 3): 0x40,
        (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (1, 3): 0x80}
# Rose layers, highest first: a cell takes the hue of its strongest dot.
BOW, BOAT, COMMAND, YAW, DRIFT, TICK = range(6)
ASCII_DOTS = '##.(:.'       # the ASCII cell per layer


def _polar(bearing_deg, radius):
    """Dot offset (right, down) of a screen bearing (0 = up, clockwise) at ``radius``."""
    rad = math.radians(bearing_deg)
    return radius * math.sin(rad), -radius * math.cos(rad)


def _arrow(bearing):
    return theme.pick(ARROWS, ARROWS_ASCII)[round(bearing / 45) % 8]


def _shape(k):
    a, b, c, d = next(m for m in ICON_SYM if _maps(m, k))
    return tuple((a * x + b * y, c * x + d * y) for x, y in ICONS[k % 4 if k % 4 < 3 else 1])


def _fits(k, tenths):
    """The icon's (shape, anchor) at ``tenths`` / 10 dots out, or None if a dot passes ICON_REACH or
    lands in a letter cell."""
    letters = {((CENTRE[0] + round(dx)) // 2, (CENTRE[1] + round(dy)) // 4)
               for dx, dy in (_polar(t, RADIUS) for t in (0, 90, 180, 270))}
    shape = _shape(k)
    ox, oy = (round(v) for v in _polar(k * 22.5, tenths / 10))
    spots = [(ox + x, oy + y) for x, y in shape]
    if all(math.hypot(x + 0.5, y + 0.5) <= ICON_REACH for x, y in spots) and not any(
            ((CENTRE[0] + x) // 2, (CENTRE[1] + y) // 4) in letters for x, y in spots):
        return shape, (ox, oy)
    return None


@lru_cache(maxsize=16)
def command_icon(k):
    """(dot offsets, anchor) of the icon for screen bearing ``k`` * 22.5 deg. The anchor is the
    farthest out that fits every bearing of the same kind (cardinal, diagonal, in-between), so
    mirrored bearings sit at one radius."""
    kin = [j for j in range(16) if (j % 4 == 0) == (k % 4 == 0) and (j % 4 == 2) == (k % 4 == 2)]
    for tenths in range(120, 40, -5):
        if all(_fits(j, tenths) for j in kin):
            return _fits(k, tenths)
    raise AssertionError(k)


def _maps(m, k):
    """Whether symmetry ``m`` turns the master's bearing (0, 22.5 or 45) into bearing ``k``."""
    base = {0: 0, 1: 1, 2: 2, 3: 1}[k % 4]
    bx, by = _polar(base * 22.5, 1.0)
    wx, wy = _polar(k * 22.5, 1.0)
    return abs(m[0] * bx + m[1] * by - wx) < 1e-6 and abs(m[2] * bx + m[3] * by - wy) < 1e-6


def cardinal(heading):
    return POINTS[round(heading / 22.5) % 16]


def _reach(rel):
    """Dots from the centre to just outside the hull, ``rel`` radians off the bow."""
    return math.ceil(1 / math.hypot(math.cos(rel) / REACH[0], math.sin(rel) / REACH[1]))


def _token(name, fallback):
    """A palette token's style, or ``fallback`` under a theme overlay that lacks it."""
    try:
        return theme.PALETTE.style(name)
    except KeyError:
        return fallback


def rose(drift=None, heading=None, yaw_ok=True, age=0.0, command=(0.0, 0.0, 0.0), amplitude=1.0, hue=''):
    """The 9 x 21 rose as Text rows: north up, or bow up (``↑bow``) while ``heading`` is None.

    ``drift`` is the world velocity (east, north) in m/s, ``heading`` the compass bearing in degrees
    and ``command`` the body (surge, sway, yaw) against ``amplitude``. Without a heading the command
    is drawn in the body frame and the world drift is hidden, never mixed in.
    """
    dots, glyphs = {}, {}
    cx, cy = CENTRE
    up = heading if heading is not None else 0.0     # the bow's screen bearing

    def put(x, y, layer):
        pos = (x // 2, y // 4)
        if 0 <= pos[0] < ROSE_COLS and 0 <= pos[1] < ROSE_ROWS:
            bits, best = dots.get(pos, (0, layer))
            dots[pos] = (bits | BITS[(x % 2, y % 4)], min(best, layer))

    def ray(bearing, start, stop, layer, step=1):
        for radius in range(start, stop + 1, step):
            dx, dy = _polar(bearing, radius)
            put(cx + round(dx), cy + round(dy), layer)

    for step in range(16):
        ray(step * 22.5, RADIUS - (step % 4 == 2), RADIUS, TICK)   # a longer tick on the intercardinals
    if heading is None:
        for col, char in enumerate(theme.pick('↑bow', '^bow')):
            glyphs[(ROSE_COLS // 2 - 1 + col, 0)] = (char, 'bold')
    else:
        for bearing, letter in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
            dx, dy = _polar(bearing, RADIUS)
            glyphs[((cx + round(dx)) // 2, (cy + round(dy)) // 4)] = (letter, 'bold')
        if drift is not None and any(drift):
            bearing = math.degrees(math.atan2(drift[0], drift[1]))
            start = _reach(math.radians(bearing - up))
            ray(bearing, start, start + round(min(math.hypot(*drift) / FULL_SPEED, 1.0) * (RADIUS - 1 - start)),
                DRIFT, 2)
    for dx, dy, part in hull.mask(90.0 - up, minimum=HULL_DOTS):
        put(cx + dx, cy + dy, BOW if part == 'bow' else BOAT)

    surge, sway, yaw = command
    scale = amplitude or 1.0
    if surge or sway:                                # one icon at the commanded bearing, no shaft
        bearing = up + math.degrees(-math.atan2(sway, surge))   # +sway is to port: counter-clockwise
        k = round(bearing / 22.5) % 16
        dots_, (ox, oy) = command_icon(k)
        if theme.ASCII:
            glyphs[((cx + ox) // 2, (cy + oy) // 4)] = (ARROWS_ASCII[round(k / 2) % 8], hue)
        else:
            for dx, dy in dots_:
                put(cx + ox + dx, cy + oy + dy, COMMAND)
    if yaw:                                          # the turn: ↺ / ↻ in the corner, dim below half
        word = theme.pick(SPIN, SPIN_ASCII)[yaw < 0]
        strong = abs(yaw) / scale >= 0.5
        for i, char in enumerate(word):
            glyphs[(ROSE_COLS - 1 - len(word) + i, 0)] = (char, f'{"bold" if strong else "dim"} {hue}'.strip())

    boat = _token('boat', 'bold') if yaw_ok else theme.WARN
    styles = (_token('bow', theme.ACCENT) if yaw_ok else theme.WARN, boat, hue, hue,
              f'dim {theme.MUTED}' if age > STALE_S else theme.MUTED, theme.MUTED)
    ascii_dots = ASCII_DOTS
    rows = []
    for row in range(ROSE_ROWS):
        line = Text()
        for col in range(ROSE_COLS):
            if (col, row) in glyphs:
                line.append(*glyphs[(col, row)])
            elif (col, row) in dots:
                bits, layer = dots[(col, row)]
                line.append(ascii_dots[layer] if theme.ASCII else chr(0x2800 + bits), styles[layer])
            else:
                line.append(' ' if theme.ASCII else '⠀')
        rows.append(line)
    return rows


# --- the command column -----------------------------------------------------------------------

def mode_label(chords, tmux=None, mode=None):
    """How the keyboard drives: ``mode`` (``proc · C=I``, ``rc · mux``) wins outside manual."""
    if mode:
        return mode
    if chords:
        return 'chords'
    tmux = bool(os.environ.get('TMUX')) if tmux is None else tmux
    return 'single key (tmux)' if tmux else 'single key'


def _figure(value, amplitude):
    """``+60%``, or a muted ``0``."""
    if not value or not amplitude:
        return Text('0', style=theme.MUTED)
    return Text(f'{value / amplitude:+.0%}')


def bar(value, amplitude, cells, hue, grows_left=False):
    """A bipolar bar of ``cells`` per side about a centre ``│``, at 1/8-cell resolution.

    A positive value grows right, or left when ``grows_left`` (sway and yaw: port is left, as on
    the water). Past ``amplitude`` the end cell shows ``▶``/``◀``. ASCII: ``#`` per whole cell.
    """
    frac = value / amplitude if amplitude else 0.0
    left = (frac > 0) == grows_left
    eighths = round(min(abs(frac), 1.0) * cells * 8)
    if theme.ASCII:
        side = Text.assemble(('#' * round(eighths / 8), hue))
    else:
        side = Text.assemble(('█' * (eighths // 8), hue))
        if eighths % 8:
            part = eighths % 8
            # Unicode has right-aligned blocks only for 1/8 and 1/2; the rest is a left fill reversed.
            side.append(*(('▕', hue) if part == 1 and left else ('▐', hue) if part == 4 and left
                          else (EIGHTHS[8 - part], f'reverse {hue}') if left else (EIGHTHS[part], hue)))
    if abs(frac) > 1:
        side = Text.assemble((side.plain[:-1], hue))
        side.append(theme.pick('◀' if left else '▶', '<' if left else '>'), style=hue)
    pad = Text(' ' * (cells - len(side)))
    empty = Text(' ' * cells)
    lhs, rhs = (Text.assemble(pad, _flip(side)), empty) if left else (empty, side + pad)
    if not frac:
        lhs, rhs = empty, empty
    return Text.assemble(lhs, (theme.BAR, theme.MUTED), rhs)


def _flip(side):
    """The cells of ``side`` in reverse order, styles kept: a left bar runs outward from the centre."""
    out = Text()
    for i in range(len(side) - 1, -1, -1):
        out.append_text(side[i])
    return out


def _labelled(label, value, amplitude, width):
    figure = _figure(value, amplitude)
    return Text.assemble(label, ' ' * max(width - len(label) - len(figure), 1), figure)


def _yaw_label(yaw):
    if not yaw:
        return 'yaw'
    return 'yaw ' + theme.pick('↺' if yaw > 0 else '↻', 'ccw' if yaw > 0 else 'cw')


def heading_row(info):
    """``hdg 247° WSW`` (``247° WSW ±4°`` with a sigma), warn with ``!`` while yaw is unreliable."""
    heading = info.get('heading')
    if heading is None:
        return Text(f'hdg {theme.DASH}', style=theme.MUTED)
    deg = theme.pick('°', '')      # ascii_safe's `deg` would not fit the column
    text = f'hdg {heading % 360:03.0f}{deg} {cardinal(heading)}'
    if info.get('heading_sd') is not None:
        text = f"{text[4:]} {theme.pick('±', '+-')}{info['heading_sd']:.0f}{deg}"
    return Text(f'! {text}', style=theme.WARN) if not info.get('yaw_ok', True) else Text(text)


# --- the panel (ui.md 6.3) --------------------------------------------------------------------

NOTABLE_STATES = ('trending', 'changed')   # drift states worth a word in the subtitle
OWNERS = {'human': 'keyboard', 'ours': 'procedure', 'nav': 'nav', 'agent': 'agent', 'hold': 'hold'}
VERDICT_HUES = {'marginal': 'WARN', 'poor': 'BAD'}   # good and unknown stay plain
BAR_CELLS = 6               # per side in the full panel; the 100 column panel fits 5
MICRO_CELLS = 2             # per side in the compact line


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
    title = Text.assemble((verdict, getattr(theme, VERDICT_HUES.get(verdict, ''), '')))
    title.append(f' {trust:.0%}' if trust is not None else '', style=theme.MUTED)
    if not full:      # the compact line has no room for the minutes
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


def driver(view):
    """``(label, hue)`` of who drives: ``keyboard·chords``, ``nav``, ``proc · C=I``, ``rc · mux``."""
    kind = (view.get('authority') or ('', ''))[0]
    owner = OWNERS.get(kind)
    hue = theme.auth_style(kind) if owner else theme.MUTED
    mode = view.get('command_mode')
    label = mode_label(view.get('chords', False), mode=mode).split(' (')[0]
    if owner == 'keyboard' and not mode:
        label = f'keyboard{theme.MIDDOT}{label}'
    elif owner and not mode:
        label = owner
    return label, hue


GAUGE_CELLS = 10
THROTTLE_BANDS = 5       # throttle.0 .. throttle.4 across the gauge, calm to hot


def _throttle(fraction):
    return theme.PALETTE.style('throttle.%d' % min(THROTTLE_BANDS - 1, int(fraction * THROTTLE_BANDS)))


def speed_gauge(speed, cells=GAUGE_CELLS, label=True):
    """``≫ ▰▰▰▰▰▰▰▱▱▱ 75%``: the manual speed as a gauge whose lit cells run calm to hot.

    Each cell wears the band of its own position, so a higher speed reaches hotter cells; the colour
    never reads as severity or any hue family (a neutral grey ramp up to white). ASCII: ``>> #######--- 75%``;
    NO_COLOR keeps the count of lit cells and the percentage.
    """
    speed = min(1.0, max(0.0, float(speed)))
    lit = int(speed * cells + 0.5)
    on, off = theme.pick('▰', '#'), theme.pick('▱', '-')
    out = Text.assemble((theme.PALETTE.glyph('speed'), _throttle(speed)), ' ')
    for i in range(cells):
        out.append(on, _throttle((i + 0.5) / cells)) if i < lit else out.append(off, theme.MUTED)
    if label:
        out.append(f' {speed:.0%}', _throttle(speed))
    return out


def command_column(view, info, width):
    """The 9 right-hand rows: driver, speed gauge, surge/sway/yaw figures over their bars, heading."""
    label, hue = driver(view)
    surge, sway, yaw = view['command']
    amplitude = view.get('amplitude', 1.0)
    cells = min(BAR_CELLS, (width - 3) // 2)
    speed = view.get('speed')
    gauge = Text() if speed is None else speed_gauge(speed, min(GAUGE_CELLS, max(3, width - 9)))
    rows = [Text(label if len(label) <= width else label.replace('keyboard', 'keys'), style=hue), gauge]
    for name, value, left in (('surge', surge, False), ('sway', sway, True), (_yaw_label(yaw), yaw, True)):
        rows.append(_labelled(name, value, amplitude, 3 + 2 * cells))
        rows.append(Text.assemble((theme.pick('▕', '['), theme.MUTED), bar(value, amplitude, cells, hue, left),
                                  (theme.pick('▏', ']'), theme.MUTED)))
    rows.append(heading_row(info))
    return rows


def _fit(segments, width, sep=None):
    """Join ``segments`` (most important first) with separators while they fit in ``width``."""
    out = Text()
    for i, segment in enumerate(segments):
        piece = Text.assemble((theme.SEPARATOR, theme.MUTED), segment) if i and sep is None else segment
        if i and len(out) + len(piece) > width:
            break
        out.append_text(piece)
    out.truncate(width, overflow='ellipsis')
    return out


def compact_lines(view, info, width=46):
    """The 60-89 column form: ``good 94% · hdg 030° NNE · drift .12→045° ±.02`` over
    ``Cmd ↑ surge  │█▊+60 · yaw ▐│  +20 · keyboard·chords``; segments that do not fit are dropped."""
    cond = [conditions_title(info, full=False), heading_row(info)]
    figures = _drift_figures(info)
    if figures:
        cond.append(Text(theme.ascii_safe(f"drift {_short(figures[0])}{theme.ARROW}{figures[1]:03.0f}° "
                                          f"±{_short(info['se'])}")))
    label, hue = driver(view)
    surge, sway, yaw = view['command']
    amplitude = view.get('amplitude', 1.0)
    rel = math.degrees(-math.atan2(sway, surge))
    head = Text.assemble(('Cmd ', theme.MUTED), (_arrow(rel) if surge or sway else theme.MIDDOT, hue))
    axes = [Text.assemble(name, ' ', bar(value, amplitude, MICRO_CELLS, hue, left), _figure(value, amplitude))
            for name, value, left in (('surge', surge, False), ('sway', sway, True), (_yaw_label(yaw), yaw, True))
            if value] or [Text('0', style=theme.MUTED)]
    speed = view.get('speed')
    tail = [] if speed is None else [speed_gauge(speed, 5)]
    command = [Text.assemble(head, ' ', axes[0]), *axes[1:], Text(label, style=hue), *tail]
    return [_fit(cond, width), _fit(command, width)]


class _Body:
    """The panel body, laid out for the width Rich gives it (38 at 100 columns, 39 at 120)."""

    def __init__(self, view, info, compact):
        self.view, self.info, self.compact = view, info, compact

    def __rich_console__(self, console, options):
        width = options.max_width
        if self.compact:
            lines = compact_lines(self.view, self.info, width)
        else:
            info, command = self.info, self.view['command']
            _, hue = driver(self.view)
            left = rose(info.get('drift'), info.get('heading'), info.get('yaw_ok', True), info.get('age', 0.0),
                        command, self.view.get('amplitude', 1.0), hue)
            right = command_column(self.view, info, width - ROSE_COLS - 3)
            lines = [Text.assemble(a, ' ', (theme.BAR, theme.MUTED), ' ', b) for a, b in zip(left, right)]
            lines += [drift_row(info), sea_row(info)]
        for line in lines:
            line.truncate(width, overflow='ellipsis')
            yield line


def conditions_panel(view, compact=False):
    """The rose beside the command column over the drift and sea rows (13 rows), or two lines when ``compact``.

    ``view['conditions']`` is ``Conditions.summary()`` plus ``heading`` (compass degrees), ``yaw_ok`` and
    ``yaw_note`` (the health ``rose`` anchor); absent means nothing measured yet.
    """
    info = view.get('conditions') or {}
    subtitle = None if compact else conditions_title(info)
    return theme.block(_Body(view, info, compact), TITLES['conditions'], view['focus'] == 'conditions',
                       subtitle=subtitle)
