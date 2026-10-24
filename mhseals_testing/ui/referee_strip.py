"""The referee strip under the minimap (referee.md section 6, owner decisions: no points).

``2025 · ▸speed 41.2 s (best 39.8) · gates 2/2 ● · contacts 0``. Segments are plain
`(text, state)` pairs; ``fit`` drops the lowest-priority ones from the right until the row
fits, then crops, so the active challenge always survives a narrow terminal.
"""

from rich.text import Text

from mhseals_testing.ui import theme

COLORS = {'PASS': 'GOOD', 'FAIL': 'BAD', 'active': 'ACCENT'}


def segments(pack, challenges, contacts=0):
    """Build segments from ``challenges``: dicts with name, verdict, text, active, best."""
    out = [(str(pack), None)]
    for c in challenges:
        label = c['name'] + (' ' + c['text'] if c.get('text') else '')
        if c.get('best') is not None and c.get('active'):
            label += f" (best {c['best']:.1f})"
        mark = theme.FOCUS if c.get('active') else ''
        if c.get('verdict') in ('PASS', 'FAIL'):
            label += ' ' + theme.pick('●', '*')
        out.append((mark + label, 'active' if c.get('active') else c.get('verdict')))
    out.append((f'contacts {contacts}', 'FAIL' if contacts else None))
    return out


def fit(parts, width):
    """Rich Text of ``parts`` no wider than ``width`` cells."""
    parts = list(parts)

    def wide(items):
        return sum(len(t) for t, _ in items) + len(theme.SEPARATOR) * max(len(items) - 1, 0)

    # Drop from the right, but keep the pack, the active challenge and the contact count.
    keep = {0, len(parts) - 1} | {i for i, (_, s) in enumerate(parts) if s == 'active'}
    for i in range(len(parts) - 2, 0, -1):
        if wide(parts) <= width:
            break
        if i not in keep:
            parts.pop(i)
            keep = {j - (j > i) for j in keep}
    text = Text(no_wrap=True)
    for i, (part, state) in enumerate(parts):
        if i:
            text.append(theme.SEPARATOR, style=theme.MUTED)
        text.append(part, style=getattr(theme, COLORS.get(state, ''), None))
    text.truncate(max(width, 0), overflow='crop')
    return text
