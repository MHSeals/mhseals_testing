"""The perception popup (V): one source's pipeline, latency, drops, tracks, heading and classes.

A pure view over the ``perc.<source>.*`` facts of perception-v2.md section 7.2 (a mapping of
fact name to ``health.model.Fact``). Absent facts leave their row out, so the popup works
with whatever the probe publishes. Nothing here reads ROS or the clock.
"""

import re

from mhseals_testing.ui import theme

STAGES = ('camera', 'sdk', 'backend', 'tracker', 'heading')
FRAME_CHAIN = ('sdk', 'backend')   # stages that should keep the camera's frame rate
LOSS_WARN = 0.15            # a drop above this between two stages marks the arrow
LATENCY_COLUMNS = 14        # samples of latency history drawn
CLASS_ROWS = 8              # classes listed before the "not seen" line
SOURCE_FACT = re.compile(r'perc\.([^.]+)\.(?:hz|latency|stages|classes)$')
BARS = theme.SPARK
ELLIPSIS = theme.pick('…', '...')
RHO = theme.pick('ρ', 'rho')


def sources(facts):
    """Source names that have at least one headline perception fact, in first-seen order."""
    found = {}
    for name in facts:
        match = SOURCE_FACT.fullmatch(name)
        if match:
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
    """``camera 14.9 -> sdk 14.6 ...`` for the stages present; frame losses over 15 % are starred.

    The tracker runs at its own rate, so only the frame chain camera, sdk, backend is judged.
    """
    parts, last = [], None
    for name in STAGES:
        hz = (stages.get(name) or {}).get('hz')
        if hz is None:
            continue
        if parts:
            lost = last and name in FRAME_CHAIN and 1 - hz / last > LOSS_WARN
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
    offset = value(facts, f'perc.{source}.clock_offset')
    if offset is not None:
        extra.append(f'clock {offset * 1000:+.0f} ms')
    infer = value(facts, f'perc.{source}.inference_ms')
    if infer is not None:
        extra.append(f'infer {infer:.0f} ms')
    tracker = value(facts, f'perc.{source}.tracker_latency')
    if tracker is not None:
        extra.append(f'tracker {tracker * 1000:.0f} ms')
    return text + (f'  ({f" {theme.MIDDOT} ".join(extra)})' if extra else '')


def drops_row(facts, source):
    """``loss 12% · stale 10 · reorder 0`` from the backend's cumulative drop counters."""
    fact = facts.get(f'perc.{source}.funnel_loss')
    if fact is None:
        return None
    drops = fact.data.get('drops', {})
    return f' {theme.MIDDOT} '.join([f'loss {fact.value:.0%}'] + [f'{k} {n:g}' for k, n in drops.items()])


def tracks_row(facts, source):
    """``S12 M1 U2 P8 · ids 93%``: the static/moving/unknown/prior split and id association."""
    parts = []
    split = value(facts, f'perc.{source}.split')
    if split:
        parts.append(split)
    ids = facts.get(f'perc.{source}.ids')
    if ids is not None:
        share = ids.data.get('id_assoc_share')
        parts.append(f'ids {share:.0%}' if share is not None and ids.value == 'true' else f'ids {ids.value}')
    age = value(facts, f'perc.{source}.costmap_age')
    if age is not None:
        parts.append(f'costmap {age:.1f} s')
    return f' {theme.MIDDOT} '.join(parts) or None


def heading_row(facts, source):
    """``lag 0.03 s ρ 0.21 ok``, or ``not excited`` while the boat has not turned enough."""
    fact = facts.get(f'perc.{source}.heading')
    if fact is None:
        return None
    if fact.data.get('excited') == 'false':
        return f'not excited {theme.MIDDOT} {fact.value}'
    try:
        return f'lag {float(fact.data["lag_s"]):.2f} s {RHO} {float(fact.data["rho"]):.2f} {fact.value}'
    except (KeyError, ValueError):
        return fact.value


def class_rows(facts, source, width=CLASS_ROWS):
    """One row per class seen (mean confidence, observations/s, confidence spark), busiest first."""
    fact = facts.get(f'perc.{source}.classes')
    if fact is None:
        return []
    seen = sorted(((n, d) for n, d in fact.data.items() if d.get('rate', 0) > 0),
                  key=lambda item: -item[1]['rate'])
    rows = [(name, f'conf {d.get("conf", 0):.2f} {theme.MIDDOT} {d["rate"]:.1f}/s  '
                   f'{spark(d.get("history", []), high=1.0, width=10)}') for name, d in seen[:width]]
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
    for label, row in (('Latency', latency_row(facts, source)), ('Drops', drops_row(facts, source)),
                       ('Tracks', tracks_row(facts, source)), ('Heading', heading_row(facts, source))):
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
