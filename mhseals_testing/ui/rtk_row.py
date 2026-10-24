"""One-row RTK status: link, fix rung, fix word and the most urgent number (gps.md 7).

``rtk_row(view, width, ascii)`` is pure: it takes a ``RtkView`` and the row's inner
width and returns Rich Text. Grammar at every tier: ``glyph RTK LINK RUNG WORD TAIL``.
The link slot carries the base phase and the correction age; the tail shows the
most urgent numbers that fit. Glyphs follow ui.md U3 (check, !, cross, busy, dot).
"""

from dataclasses import dataclass
import math

from rich.text import Text

STALE, DEAD = 2.0, 5.0          # s, rtk.age warn and fail (health.md)
GUTTER = 3                      # 1 pad and a 2-col cursor gutter before the glyph
OVERHEAD = 10                   # gutter, glyph, space, "RTK", space, right pad
LINK_MAX, WORD = 12, 5
TIERS = ((40, 11), (30, 6), (26, 4))        # (min inner width, tail width at the tier)
FIXES = {'none': ('▁', '_'), '2D': ('▂', '.'), '3D': ('▃', '-'),
         'DGPS': ('▄', '='), 'FLOAT': ('▆', '+'), 'FIXED': ('█', '#')}
SPINNER = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'
PARTIAL = ' ▏▎▍▌▋▊▉'
# (unicode, ascii, style) per row state
GLYPHS = {'good': ('✓', '+', 'green'), 'warn': ('!', '!', 'yellow'), 'bad': ('✗', 'x', 'red'),
          'busy': ('◌', '~', 'cyan'), 'off': ('·', 'o', 'dim')}


@dataclass
class RtkView:
    """Everything the row shows; ``None`` means unknown or not applicable."""
    base: str = 'none'              # none | identifying | surveying | streaming | error
    fix: str = 'none'               # rover fix word, a key of FIXES
    age: float = None               # s since the last RTCM frame; None with base streaming = no frame yet
    drops: int = 0                  # stale drops in the last 60 s
    pubs: int = 1                   # RTCM publishers seen
    survey: float = 0.0             # 0..1, see survey_fraction
    survey_limit: str = 'time'      # time | accuracy: the term limiting survey
    survey_left: float = None       # s left of the minimum survey time
    survey_acc: float = None        # m, mean accuracy now
    survey_target: float = None     # m
    baseline: float = None          # m, ECEF base to rover
    converge_warn: bool = False     # rtk.converge warns
    tick: int = 0                   # spinner frame


def survey_fraction(t, t_min, acc, acc0, target, valid):
    """min(time, accuracy) progress; never 1 before NAV-SVIN ``valid``, and it may go backwards."""
    f_time = t / t_min if t_min else 1.0
    f_acc = 1.0
    if acc and acc0 > target > 0:
        f_acc = math.log(acc0 / acc) / math.log(acc0 / target) if acc > target else 1.0
    fraction = max(0.0, min(1.0, f_time, f_acc))
    return fraction if valid else min(fraction, 0.99)


def _state(view):
    """(link, glyph state): link is one of off/error/identifying/surveying/fresh/stale/dead."""
    if view.base == 'error':
        return 'error', 'bad'
    if view.base != 'streaming':
        link = {'identifying': 'identifying', 'surveying': 'surveying'}.get(view.base, 'off')   # unknown = no base
        if view.age is not None and view.age > STALE:
            link = 'stale' if view.age <= DEAD else 'dead'      # it streamed once: corrections are dying while it reconnects
    elif view.age is None or view.age > DEAD:
        link = 'dead'
    else:
        link = 'stale' if view.age > STALE else 'fresh'
    streaming = view.base == 'streaming'
    if view.pubs > 1 or link == 'dead' or (streaming and view.fix == 'none'):
        return link, 'bad'
    if link == 'stale' or view.drops or view.converge_warn:
        return link, 'warn'
    if link == 'off':
        return link, 'off'
    if link == 'fresh' and view.fix == 'FIXED':
        return link, 'good'
    return link, 'busy'


def _bar(fraction, cells, ascii):
    if ascii:
        full = round(fraction * cells)
        return Text('#' * full) + Text('.' * (cells - full), style='dim')
    eighths = int(fraction * cells * 8 + 1e-9)
    full, part = divmod(min(eighths, cells * 8), 8)
    text = Text('█' * full + PARTIAL[part].strip())
    return text + Text('░' * (cells - full - bool(part)), style='dim')


def _link(view, link, cells, ascii):
    """The link slot, exactly ``cells`` wide; labels shorten to fit the slot, not the row."""
    short = cells < 7           # 'no base' is 7 cells
    if link == 'off':
        return Text(('off' if short else 'no base').ljust(cells), style='dim')
    if link == 'error':
        return Text(('err' if short else 'error').ljust(cells), style='red')
    if link == 'identifying':
        frame = '|/-\\'[view.tick % 4] if ascii else SPINNER[view.tick % len(SPINNER)]
        return Text((frame + ('' if short else ' base')).ljust(cells), style='dim')
    if link == 'surveying':
        return _bar(view.survey, cells, ascii)
    if link == 'fresh':
        return Text(('=' if ascii else '━') * (cells - 1) + ('>' if ascii else '▸'))
    style = 'yellow' if link == 'stale' else 'red'
    pattern = ''.join('- '[i % 2] for i in range(cells)) if ascii else '╌' * cells
    if link == 'dead':
        cross = 'x' if ascii else '×'
        pattern = pattern[:cells // 2] + cross + pattern[cells // 2 + 1:]
    return Text(pattern, style=style)


def _age(age, narrow):
    if narrow or age >= 10:
        return '>99s' if age > 99 else f'{age:.0f}s'
    return f'{age:.1f}s'


def _distance(metres, narrow):
    if metres >= 1000:
        return f'{metres / 1000:.0f}km' if narrow else f'{metres / 1000:.1f}km'
    return f'{metres:.0f}m' if narrow else f'{metres:.1f}m'


def _tail_items(view, link, ascii, narrow, long):
    """(text, style) candidates in priority order; the caller keeps what fits."""
    if view.pubs > 1:
        yield (f'{view.pubs}pub' if narrow else f'{view.pubs} pubs'), 'red'
    if link in ('stale', 'dead') and view.age is not None:
        yield _age(view.age, narrow), 'yellow' if link == 'stale' else 'red'
    if view.drops:
        yield ('-' if ascii else '−') + str(view.drops), 'yellow'
    if link == 'surveying':
        if view.survey_limit == 'time' and view.survey_left is not None:
            yield (f'{view.survey_left:.0f}s left' if long else f'{view.survey_left:.0f}s'), ''
        elif view.survey_acc is not None:
            goal = f'{">" if ascii else "→"}{view.survey_target:.1f}' if long and view.survey_target else ''
            yield f'{view.survey_acc:.1f}{goal}m', ''
    if view.fix in ('FLOAT', 'FIXED') and view.baseline is not None:
        yield _distance(view.baseline, narrow), ''


def rtk_row(view, width, ascii=False, cursor=None):
    """The RTK row for an inner ``width``; empty Text below 26 columns, where the header strip stands in.

    ``cursor`` is the Status cursor glyph (2 cols), drawn in the gutter when the cursor is on this row.
    """
    tier = next((tail for floor, tail in TIERS if width >= floor), None)
    if tier is None:
        return Text()
    narrow, long = width < 30, width >= 40
    value = width - OVERHEAD
    cells = min(LINK_MAX, value - WORD - 4 - tier)
    tail_width = value - cells - WORD - 4
    link, state = _state(view)
    glyph, plain, style = GLYPHS[state]
    rung, rung_ascii = FIXES.get(view.fix, FIXES['none'])
    fix_style = {'none': 'red', '2D': 'yellow', 'FIXED': 'bold green'}.get(view.fix, '')
    if view.fix == 'FLOAT' and view.converge_warn:
        fix_style = 'yellow'

    tail, used = [], 0
    for text, item_style in _tail_items(view, link, ascii, narrow, long):
        if used + len(text) + bool(tail) <= tail_width:
            used += len(text) + bool(tail)
            tail.append((text, item_style))
    row = Text(' ')
    row.append(cursor if cursor else '  ', style='bold cyan')
    row.append(plain if ascii else glyph, style=style)
    row.append(' RTK ')
    row.append_text(_link(view, link, cells, ascii))
    row.append(' ')
    row.append(rung_ascii if ascii else rung, style=fix_style)
    row.append(' ' + view.fix.ljust(WORD) + ' ', style=fix_style)
    row.append(' ' * (tail_width - used))
    for i, (text, item_style) in enumerate(tail):
        row.append(('' if i == 0 else ' ') + text, style=item_style)
    row.append(' ')
    return row


# Status row state and strip state per glyph state (Status vocabulary: OK, WARN, ERROR)
ROW_STATES = {'good': 'OK', 'busy': 'CHECKING', 'off': 'OK', 'warn': 'WARN', 'bad': 'ERROR'}
ROW_STATES_STRIP = {'good': 'LIVE', 'busy': 'CHECKING', 'off': 'LIVE', 'warn': 'WARN', 'bad': 'FAIL'}
BASE_WORDS = {'off': 'off', 'error': 'error', 'identifying': 'base', 'surveying': 'svin'}


def strip_item(view):
    """(state, label) for the header strip: ``rtk FLOAT``, ``rtk stale`` (warn), ``rtk 14s`` (fail)."""
    link, state = _state(view)
    if view.pubs > 1:
        return 'FAIL', f'rtk {view.pubs} pubs'
    if link == 'error':
        return 'FAIL', 'rtk error'
    if link == 'dead':
        return 'FAIL', 'rtk ' + ('none' if view.age is None else _age(view.age, True))
    if state == 'bad':
        return 'FAIL', f'rtk {view.fix}'
    if link == 'stale':
        return 'WARN', 'rtk stale'
    word = BASE_WORDS.get(link, view.fix)
    return ROW_STATES_STRIP[state], f'rtk {word}'


class RtkCell:
    """Rich renderable: the row at whatever width the Status column gives it."""

    def __init__(self, view, ascii=False, cursor=None):
        self.view, self.ascii, self.cursor = view, ascii, cursor

    def __rich_console__(self, console, options):
        yield rtk_row(self.view, options.max_width, self.ascii, self.cursor)
