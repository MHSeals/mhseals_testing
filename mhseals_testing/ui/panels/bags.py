"""The Bags tab: disk gauge, one row per session with flags, and the selected session's summary.

Pure rendering of what ``harness.bags`` returns; key handling and the popups belong to the app.
Rows follow ui.md 6.9: lead glyph ``◌`` recording, ``!`` unfinalized, ``·`` empty; flags are
``pin``, ``▾N`` markers, ``✓ sent``, ``unfin.``, ``health✗N``, ``sim``.
"""

from datetime import datetime

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme

GAUGE_WIDTH = 24
GB = 1e9


def gauge(fraction, width=GAUGE_WIDTH):
    """``━━━──`` filled to ``fraction`` (clamped to 0..1); ``#`` on ``-`` in ASCII mode."""
    filled = round(max(0.0, min(1.0, fraction)) * width)
    on, off = theme.pick('━', '#'), theme.pick('─', '-')
    return Text(on * filled, style=theme.ACCENT) + Text(off * (width - filled), style=theme.MUTED)


def size_text(size):
    """``412 MB`` / ``1.1 GB``."""
    return f'{size / GB:.1f} GB' if size >= GB else f'{size / 1e6:.0f} MB'


def duration_text(seconds):
    """``38m`` / ``1.2h``; empty when unknown."""
    if seconds <= 0:
        return ''
    return f'{seconds / 60:.0f}m' if seconds < 3600 else f'{seconds / 3600:.1f}h'


def lead(bag):
    """The one-cell state glyph of a row."""
    if bag.recording:
        return theme.icon('busy')
    if bag.unfinalized:
        return theme.icon('warn')
    return Text(theme.MIDDOT, style=theme.MUTED) if bag.empty else Text(' ')


def flags(bag):
    """The Flags cell, in the fixed order pin, markers, sent, unfinalized, health, sim."""
    parts = []
    if bag.pinned:
        parts.append('pin')
    if bag.markers:
        parts.append(f'{theme.pick("▾", "v")}{bag.markers}')
    if bag.sent:
        parts.append(f'{theme.pick("✓", "+")} sent')
    if bag.unfinalized:
        parts.append('unfin.')
    if bag.health_fails:
        parts.append(f'health{theme.pick("✗", "x")}{bag.health_fails}')
    if bag.sim:
        parts.append('sim')
    return ' '.join(parts)


def title_text(bag):
    """The title, else ``(empty)`` for an empty session, else the outcome as the automatic title."""
    return bag.title or ('(empty)' if bag.empty else bag.outcome or '(untitled)')


def gauge_line(usage, bag_bytes, budget, rate=None):
    """``Disk ━━━── 21.4/58 GB · bags 12.3 GB · budget 20 GB`` plus ``rec ≈0.2 MB/s · 41 h left`` when a rate is known."""
    used, total = usage
    text = Text('Disk ', style=theme.MUTED) + gauge(used / total if total else 0)
    text.append(f' {used / GB:.1f}/{total / GB:.0f} GB{theme.SEPARATOR}bags {bag_bytes / GB:.1f} GB'
                f'{theme.SEPARATOR}budget {budget / GB:.0f} GB')
    if rate:
        hours = (total - used) / rate / 3600
        text.append(f'{theme.SEPARATOR}rec ≈{rate / 1e6:.1f} MB/s{theme.SEPARATOR}{hours:.0f} h left')
    return text


def render(bags, usage, cursor=0, budget=20 * GB, root='', rate=None, focused=True):
    """The list panel. ``usage`` is ``(used, total, bags_bytes)`` from ``harness.bags.disk``."""
    used, total, bag_bytes = usage
    table = theme.grid('', 'Session', 'Title', dict(header='Dur', justify='right'),
                       dict(header='Size', justify='right'), 'Outcome', 'Flags')
    for index, bag in enumerate(bags):
        active = index == cursor
        marker = Text(theme.CURSOR if active else '  ', style=f'bold {theme.ACCENT}') + lead(bag)
        table.add_row(marker, bag.id, title_text(bag), duration_text(bag.duration), size_text(bag.size),
                      bag.outcome or theme.DASH, flags(bag), style=f'bold {theme.ACCENT}' if active else None)
    body = Group(gauge_line((used, total), bag_bytes, budget, rate), Text(''),
                 table if bags else Text('no sessions yet', style=theme.MUTED))
    return theme.block(body, 'Bags', focused, subtitle=f'{root} · {len(bags)} sessions' if root else f'{len(bags)} sessions')


def summary(bag):
    """The bottom panel for the selected session: when, where, what ran, and the note."""
    if bag is None:
        return theme.block(Text('nothing selected', style=theme.MUTED), 'summary')
    when = datetime.fromtimestamp(bag.start).strftime('%Y-%m-%d %H:%M')
    lines = [f'{when}{theme.SEPARATOR}{duration_text(bag.duration) or "?"}{theme.SEPARATOR}{size_text(bag.size)}',
             f'outcome: {bag.outcome or theme.DASH}{theme.SEPARATOR}markers {bag.markers}'
             f'{theme.SEPARATOR}health fails {bag.health_fails}']
    if bag.note:
        lines.append(f'note: {bag.note}')
    return theme.block(Text('\n'.join(lines)), f'{bag.id}{theme.SEPARATOR}{title_text(bag)}')
