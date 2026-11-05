"""Inspect section 2: a TF chain's frame diagram."""

from rich.text import Text

from mhseals_testing.ui import theme


# --- 2. TF chain ------------------------------------------------------------------------------

def chain(edges, source, target):
    """The hops source -> target over every edge ever seen: ([(parent, child, [edge dicts])], found).

    Without a full chain the hops reach the deepest frame source does reach (the partial chain)."""
    by_parent = {}
    for edge in edges:
        by_parent.setdefault(edge['parent'], {}).setdefault(edge['child'], []).append(edge)
    pending, seen, deepest = [(source, [])], {source}, []
    while pending:
        frame, path = pending.pop(0)
        if frame == target:
            return path, True
        deepest = path if len(path) > len(deepest) else deepest
        for child, group in sorted(by_parent.get(frame, {}).items()):
            if child not in seen:
                seen.add(child)
                pending.append((child, path + [(frame, child, group)]))
    return deepest, False


def tf_part(edges, source, target, inner):
    """``map ━━▶ odom ━━▶ base_link`` with each hop's publisher and age; a missing hop is ``╌✗╌`` with the
    publisher once seen sending the target, if any; every edge into the chain below."""
    hops, found = chain(edges, source, target)
    line, pubs, ages = Text(source), Text(' ' * len(source)), Text(' ' * len(source))

    def hop(arrow, child, who, age, style, who_style, age_style, child_style=None):
        width = max(len(arrow) + len(child), len(who) + 2, len(age) + 2)
        line.append(arrow, style=style)
        line.append(child.ljust(width - len(arrow)), style=child_style)
        pubs.append(('  ' + who).ljust(width), style=who_style)
        ages.append(('  ' + age).ljust(width), style=age_style)

    for parent, child, group in hops:
        fresh = [e for e in group if e['fresh']]
        names = sorted({e['authority'] or '?' for e in fresh or group})
        style = theme.BAD if not fresh else theme.WARN if len(names) > 1 else theme.GOOD
        best = min(group, key=lambda e: e['seen'])
        age = ('static' if best['static'] else f"{best['stamp_age']:.2f} s dyn") if fresh \
            else f"last seen {best['seen']:.0f} s ago"
        hop(f" {theme.pick('━━▶', '-->')} ", child, f'! {len(names)} publishers' if len(names) > 1 else names[0],
            age, style, style if len(names) > 1 else theme.MUTED, theme.MUTED if fresh else theme.BAD)
    if not found:
        into = [e for e in edges if e['child'] == target]
        known = sorted({e['authority'] or '?' for e in into})
        who = f"expected: {', '.join(known)}" if known else f'no publisher of {target} seen'
        last = f"last seen {min(e['seen'] for e in into):.0f} s ago" if into else 'never seen'
        hop(f" {theme.pick('╌✗╌', '-x-')} ", target, who, last, theme.BAD, theme.MUTED, theme.BAD, theme.BAD)
    out = [line, pubs, ages]
    frames = {source, target} | {child for _, child, _ in hops}
    table = theme.grid({'header': 'edge'}, {'header': 'publisher'}, {'header': 'age', 'justify': 'right'},
                       {'header': ''})
    for e in sorted(edges, key=lambda e: (e['child'], e['parent'], e['authority'])):
        if e['child'] in frames:
            state = 'OK' if e['fresh'] else 'STALE'
            table.add_row(f"{e['parent']} {theme.ARROW} {e['child']}", e['authority'] or '?',
                          'static' if e['static'] else f"{e['stamp_age']:.2f} s", theme.icon(state))
    return out + [Text(''), table]
