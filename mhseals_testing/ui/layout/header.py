"""The page header (name, profile, host; health strip) and footer (state, focus, key hints)."""

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.harness import recwatch
from mhseals_testing.ui import chrome, theme
from mhseals_testing.ui.panels import TITLES, scrubber
from mhseals_testing.ui.panels.status import reconcile, status_rows
from mhseals_testing.ui.rtk import row as rtk_row
from mhseals_testing.ui.layout.density import PINNED_HINTS, STRIP_FULL, STRIP_SHORT, WIDE, WIDER


def strip_item(state, label):
    """A healthy item is plain muted text; anything else gets its status glyph and hue."""
    if theme.kind(state) == 'good':
        return Text(label, style=theme.MUTED)
    return theme.labelled(state, label)


def rec_chip(view):
    """The recorder item: plain muted text when fine, a glyph only when it needs attention."""
    sample = view.get('rec')
    if sample is None:                           # no watcher (tests, headless): running or not
        return strip_item('LIVE' if view['recording'] else 'FAIL', 'rec')
    state, text = recwatch.strip_text(sample, view['armed'], theme.SEPARATOR, theme.pick('≈', '~'))
    if state in ('warn', 'fail'):
        return theme.labelled('WARN' if state == 'warn' else 'FAIL', text)
    return Text(text, style=theme.MUTED if state == 'ok' else 'dim')


def issue_chip(issues):
    """The issue glyph and count (ui/feeds/issues.py), first in the strip so it is never cut."""
    word = 'issue' if len(issues) == 1 else 'issues'
    return Text(theme.PALETTE.glyph('issue'), style=f'bold {theme.WARN}') + Text(f' {len(issues)} {word}',
                                                                              style=theme.WARN)


def health_strip(snapshot, view, width=None):
    rows = status_rows(snapshot, view['specs'])
    sensors = [row for row in rows if row.get('source') == 'sensor' and row['required']]
    sensors_ok = all(row['state'] == 'LIVE' for row in sensors)
    frames_ok = all(row['state'] == 'LIVE' for row in rows if row.get('source') == 'tf')
    issues = view.get('issues') or {}
    parts = [*([issue_chip(issues)] if issues else []), rec_chip(view),
             strip_item('LIVE' if sensors_ok else 'FAIL',
                        f"sensors {sum(r['state'] == 'LIVE' for r in sensors)}/{len(sensors)}"),
             strip_item('LIVE' if frames_ok else 'FAIL', 'tf')]
    for row in rows:
        if row['group'] == 'System' and row['state'] != 'OK':
            parts.append(strip_item(row['state'], f"{row['name']} {row['value']}".strip()))
    if snapshot.get('rtk_view') is not None:
        parts.append(strip_item(*rtk_row.strip_item(snapshot['rtk_view'])))
    if view['bench_override']:
        parts.append(theme.labelled('WARN', 'bench override'))
    if width is None:
        return Text(theme.SEPARATOR).join(parts)
    cut = 0
    if width < STRIP_FULL:  # only the first five items show below 100 columns; the rest count in +N
        parts, cut = parts[:STRIP_SHORT], max(len(parts) - STRIP_SHORT, 0)
    return fit_strip(parts, width, cut)


def health_total(health):
    """The right end of the strip: ``health 41 ok`` or the fail and warn counts and the worst check."""
    if not health:
        return Text('')
    if not health.get('bad') and not health.get('warn'):
        return Text(f"health {health.get('ok', 0)} ok", style=theme.MUTED)
    total = Text()
    for state, count in (('bad', health.get('bad')), ('warn', health.get('warn'))):
        if count:
            total.append_text(theme.icon(state)).append(f' {count}  ')
    if health.get('worst'):
        total.rstrip()
        total.append(theme.SEPARATOR + health['worst'], style=theme.MUTED)
    return total


def fit_strip(chips, width, extra=0):
    """Join the chips, replacing those that do not fit (and ``extra`` already cut) with a ``+N`` chip."""
    gap = Text(theme.SEPARATOR)
    for shown in range(len(chips), 0, -1):
        hidden = len(chips) - shown + extra
        parts = chips[:shown] + ([Text(f'+{hidden}', style=theme.MUTED)] if hidden else [])
        line = gap.join(parts)
        if len(line) + 1 <= width:
            return line
    return chips[0]


def header(snapshot, view, width=120):
    """Row 1: title, profile, deploy, host and the authority and arm chips; row 2: the health strip or agent banner."""
    top = chrome.top_row(view, width)
    stopped = (view.get('authority') or (None,))[0] == 'stop' and not view.get('review')
    if stopped:      # a stop wins the row; an agent claim on it is named there, never hidden
        return Group(top, chrome.stop_banner(view.get('armed'), width, agent=view.get('agent')))
    if (view.get('operator') or {}).get('state', 'normal') != 'normal' and not view.get('review'):
        return Group(top, chrome.operator_banner(view['operator'], width, view.get('agent')))   # names the claim
    if view.get('agent'):
        return Group(top, chrome.banner(view['agent'], width))
    total = health_total(reconcile(view.get('health'), status_rows(snapshot, view['specs'])))
    row = Table.grid(expand=True)
    row.add_column(no_wrap=True, overflow=theme.OVERFLOW)
    row.add_column(justify='right', no_wrap=True)
    row.add_row(Text(' ') + health_strip(snapshot, view, width - len(total) - 3), total + Text(' '))
    return Group(top, row)


def fit_hints(hints, capacity):
    """Cut the hints to ``capacity`` but never the PINNED keys, which keep the last cells."""
    if len(hints) <= capacity:
        return hints
    pinned = [hint for hint in hints if hint[0] in PINNED_HINTS]
    rest = [hint for hint in hints if hint[0] not in PINNED_HINTS]
    return rest[:max(capacity - len(pinned), 0)] + pinned[-capacity:]


def playing(view):
    """The ui.review.Review being played, or None (``review: True`` only asks for the chip)."""
    review = view.get('review')
    return review if review and not isinstance(review, bool) else None


KEY_TOKENS = {'Space': 'stop', 'E': 'auth.agent'}   # footer keys with a hue of their own, always


def key_style(key):
    token = KEY_TOKENS.get(key)
    return theme.PALETTE.style(token) if token else None


def footer(view, width):
    status = Table.grid(expand=True, padding=(0, 1))
    status.add_column(ratio=4, no_wrap=True, overflow=theme.OVERFLOW)
    status.add_column(ratio=6, justify='right', no_wrap=True, overflow=theme.OVERFLOW)
    position = view['position']
    title, position = view.get('focus_title') or (TITLES.get(view['focus'], view['focus'].title()), position)
    focus = Text(f' Focus {title}', style=f'bold {theme.ACCENT}')
    if position:
        focus.append(f'{theme.SEPARATOR}{position}', style=theme.MUTED)
    notice = Text(view['notice'], style=theme.MUTED)
    if view.get('flash'):      # a mark's confirmation: bold, before the notice
        notice = Text(view['flash'], style=f'bold {theme.ACCENT}') + Text(theme.SEPARATOR, style=theme.MUTED) + notice
    status.add_row(focus, notice)
    review = playing(view)
    if review:  # the strip footer takes the status line's place; the notice rides on its status line
        status = Group(*scrubber.review_footer(review, width, view['notice']))
    hints = view['hints']
    columns = 5 if width >= WIDER else 4 if width >= WIDE else 3 if width >= 60 else 2
    grid = Table.grid(expand=True)
    for _ in range(columns):
        grid.add_column(ratio=1, no_wrap=True, overflow=theme.OVERFLOW)
    hints = fit_hints(hints, columns * 2)
    for row in range(2):
        cells = [theme.hint(*hint, style=key_style(hint[0])) for hint in hints[row * columns:(row + 1) * columns]]
        grid.add_row(*(cells + [''] * (columns - len(cells))))
    return Group(status, grid)
