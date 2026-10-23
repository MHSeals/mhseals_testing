"""The perception popup (V): one source's pipeline, latency, drops and per-class rates.

A pure view over the ``perc.<source>.*`` facts of perception.md section 4 (a mapping of
fact name to ``health.model.Fact``). Absent facts leave their row out, so the popup works
with whatever the probe publishes. Nothing here reads ROS or the clock.
"""

import re

from mhseals_testing.ui import theme

STAGES = ('camera', 'detector', 'sdk', 'bridge', 'converter')
LOSS_WARN = 0.15            # a drop above this between two stages marks the arrow
LATENCY_COLUMNS = 14        # seconds of latency history drawn
CLASS_ROWS = 8              # classes listed before the "not seen" line
SOURCE_FACT = re.compile(r'perc\.([^.]+)\.(?:hz|latency|stages|classes)$')
BARS = theme.SPARK
ELLIPSIS = theme.pick('…', '...')


def sources(facts):
    """Source names that have at least one headline perception fact, in first-seen order."""
    found = {}
    for name in facts:
        match = SOURCE_FACT.fullmatch(name)
        if match and match.group(1) != 'tracks':
            found[match.group(1)] = True
    return list(found)


def spark(values, high=None, width=LATENCY_COLUMNS):
    """Bars for the last ``width`` values scaled to 0..``high`` (default their max)."""
    values = [v for v in values if v is not None][-width:]
    top = high if high is not None else max(values, default=0)
    if not values or top <= 0:
        return theme.MIDDOT * width
    return ''.join(BARS[min(len(BARS) - 1, int(len(BARS) * v / top))] for v in values)


def value(facts, name, default=None):
    fact = facts.get(name)
    return default if fact is None else fact.value


def pipeline(stages):
    """``camera 4.8 -> detector 4.7 ...`` for the stages present; losses over 15 % are starred."""
    parts, last = [], None
    for name in STAGES:
        hz = (stages.get(name) or {}).get('hz')
        if hz is None:
            continue
        if parts:
            lost = last and 1 - hz / last > LOSS_WARN
            parts.append(f' {theme.ARROW}{"!" if lost else ""} ')
        parts.append(f'{name} {hz:.1f}')
        last = hz
    return ''.join(parts)


def latency_row(facts, source):
    fact = facts.get(f'perc.{source}.latency')
    if fact is None:
        return None
    text = f'p50 {fact.value * 1000:.0f}'
    if 'p95' in fact.data:
        text += f' {theme.MIDDOT} p95 {fact.data["p95"] * 1000:.0f}'
    text += ' ms'
    history = fact.data.get('history')
    if history:
        text += f'  {spark(history)}'
    extra = []
    offset = next((f for n, f in facts.items() if n.startswith('clock.offset.')), None)
    if offset is not None:
        extra.append(f'clock {offset.value * 1000:+.0f} ms')
    infer = value(facts, f'perc.{source}.inference_ms')
    if infer is not None:
        extra.append(f'infer {infer:.0f} ms')
    return text + (f'  ({f" {theme.MIDDOT} ".join(extra)})' if extra else '')


def drops_row(facts, source):
    drops = value(facts, f'perc.{source}.drops')
    if not isinstance(drops, dict):
        return None
    text = f' {theme.MIDDOT} '.join(f'{reason} {count:g}' for reason, count in drops.items())
    nodes = facts.get(f'perc.{source}.remote_nodes')
    if nodes is not None:
        text += f'    nodes {nodes.value}'
    return text or 'none'


def class_rows(facts, source, width=CLASS_ROWS):
    """One row per class seen, busiest first, then a line naming those not seen."""
    fact = facts.get(f'perc.{source}.classes')
    if fact is None:
        return []
    seen = sorted(((n, d) for n, d in fact.data.items() if d.get('rate', 0) > 0),
                  key=lambda item: -item[1]['rate'])
    rows = []
    for name, d in seen[:width]:
        age = d.get('last_age')
        rows.append((name, f'{d["rate"]:.1f}/s {theme.MIDDOT} {d.get("presence", 0):.0%} '
                           f'{theme.MIDDOT} p50 {d.get("p50", 0):.2f} {theme.MIDDOT} '
                           f'{"-" if age is None else f"{age:.1f}s"}  {spark(d.get("hist", []), width=10)}'))
    more = seen[width:]
    if more:
        rows.append(('', f'+{len(more)} more'))
    quiet = sorted(n for n, d in fact.data.items() if d.get('rate', 0) <= 0)
    if quiet:
        rows.append(('', f'{len(quiet)} not seen: {", ".join(quiet[:3])}'
                         f'{ELLIPSIS if len(quiet) > 3 else ""}'))
    return rows


def rows(facts, source):
    """The popup body as (label, text) pairs; empty sections are left out."""
    out = []
    stages = value(facts, f'perc.{source}.stages')
    if isinstance(stages, dict) and pipeline(stages):
        out.append(('Pipeline', pipeline(stages)))
    for label, row in (('Latency', latency_row(facts, source)), ('Drops /min', drops_row(facts, source))):
        if row:
            out.append((label, row))
    out += class_rows(facts, source)
    return out


def modal(facts, source=None, width=100):
    """A popup modal for ``source`` (default: the first one with facts)."""
    names = sources(facts)
    source = source or (names[0] if names else None)
    body = rows(facts, source) if source else []
    return dict(kind='message', title=f'Perception{theme.SEPARATOR}{source or "no source"}',
                rows=body or [('', 'No perception facts yet')], width=width,
                hint=f'v view{theme.SEPARATOR}Esc close{theme.SEPARATOR}j/k scroll')
