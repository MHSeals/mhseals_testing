"""Inspect section 1: a sensor's 60 s arrival-rate strip."""

import math
from bisect import bisect_left, bisect_right
from rich.text import Text

from mhseals_testing.profiles import MIN_DEADLINE
from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.popups.inspect.common import SEP, WINDOW, kv, number


# --- 1. sensor rate strip ---------------------------------------------------------------------

def rate_columns(times, now, cells, hz, gap):
    """(Hz per dot column, gap flag per column) over the last WINDOW seconds of arrival ``times``."""
    n = cells * 2
    win = max(1.0, 5 / hz) if hz else 2.0
    rates, gaps = [], []
    for x in range(n):
        tc = now - WINDOW + (x + 1) * WINDOW / n
        if not times or tc < times[0]:
            rates.append(None)
            gaps.append(False)
            continue
        count = bisect_right(times, tc) - bisect_right(times, tc - win)
        rates.append(count / win if tc - win >= times[0] else None)
        after = bisect_right(times, tc)
        prev, nxt = times[after - 1], times[after] if after < len(times) else now
        gaps.append(nxt - prev > gap)
    return rates, gaps


def sensor_part(state, spec, row, inner):
    """Rate strip, rate figures, topic and publishers, then the latest values."""
    times, now = list(state.times), state.now
    times = times[bisect_left(times, now - WINDOW):]
    hz = spec.hz
    gap = max(3 / hz, MIN_DEADLINE) if hz else (spec.deadline or 2.0)
    cells = inner - 5
    out = []
    if not state.live:
        out.append(kv('Rate', Text('review: no arrivals are kept for a past session', style=theme.MUTED)))
    elif len(times) < 2:
        out.append(kv('Rate', Text(f'no arrivals in the last {WINDOW:.0f} s', style=theme.BAD)))
    else:
        rates, gaps = rate_columns(times, now, cells, hz, gap)
        seen = [r for r in rates if r is not None]
        dts = [b - a for a, b in zip(times, times[1:])]
        median = sorted(dts)[len(dts) // 2]
        jitter = sorted(abs(d - median) for d in dts)[int(0.95 * (len(dts) - 1))]
        hi = math.ceil(max([(hz or 0) * 1.3] + [r * 1.1 for r in seen] + [1.0]))
        stamp = (row or {}).get('stamp_age')
        figures = [f"{(row or {}).get('hz', 0):.0f} Hz"]
        if seen:
            figures.append(f'{min(seen):.0f}–{max(seen):.0f}')
        figures.append(f'max gap {max(dts):.2f} s')
        if stamp is not None:
            figures.append(f'stamp {stamp * 1000:.0f} ms')
        figures.append(f"{theme.pick('±', '+/-')}{jitter * 1000:.0f} ms p95")
        out.append(kv('Rate', theme.ascii_safe(SEP.join(figures))))
        band = (0.85 * hz,) if hz else None      # the warn floor of topic_rate.classify
        lines = sparks.spark(rates, cells, 0, hi, band=band, gaps=gaps, height=3)
        axis = theme.pick('┤', '|')
        for value, line in zip((f'{hi:>3.0f} {axis}', f'    {theme.BAR}', f'{0:>3.0f} {axis}'), lines):
            out.append(Text(value, style=theme.MUTED) + line)
        tail = (f"expected {hz:g} Hz{SEP}{theme.pick('┈', '-')} {0.85 * hz:.3g} warn floor" if hz
                else f'typical {1 / median:.3g} Hz (no nominal rate)')
        axis_line = Text(f'    -{WINDOW:.0f} s', style=theme.MUTED)
        axis_line.append(' ' * max(1, cells - len(tail) - 10) + tail + '  now', style=theme.MUTED)
        out.append(axis_line)
        if any(gaps):
            note = f"{theme.pick('┄', '.')} silence over {gap:.1f} s (the stale deadline)"
            out.append(kv('', Text(note, style=theme.BAD)))
    pubs = state.publishers
    who = (f'{len(pubs)} publisher{"s" * (len(pubs) != 1)}' + (f' ({", ".join(pubs[:3])})' if pubs else '')
           if pubs is not None else 'publishers unknown')
    out.append(kv('Topic', f'{spec.topic}{SEP}{who}'))
    values = {k: v for k, v in ((row or {}).get('values') or {}).items() if k != 'summary'}
    if values:
        out.append(kv('Values', SEP.join(f'{k} {number(v)}' for k, v in values.items())))
    return out
