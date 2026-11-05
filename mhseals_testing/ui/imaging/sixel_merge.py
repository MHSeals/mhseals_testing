"""Sixel parts painted as few images as possible: neighbouring tile parts become one composited image.

A frame paints every tile part that changed as its own sixel image, so a moving minimap sends about a hundred
images a frame. tmux keeps only its last 19 sixel images server-wide and redraws only those, so the rest of the
picture blanks on tmux's next redraw. ``compose`` joins the changed cells into the fewest rectangles of whole
rows (``rects``) and composites each from its tiles' pixels under one palette; when even that passes
``cap`` images, every visible cell is painted again in its rectangles, which are whole regions and fewer.
"""

import numpy as np

from mhseals_testing.ui.imaging.sixel import sixel

PALETTE_MAX = 255          # sixel colour registers a rectangle may use (index 0 is transparent)


def cap():
    """config/perf.yaml ``image_cap``: the most sixel images a frame paints (0 paints each tile part alone)."""
    from mhseals_testing.tunables import tune
    return tune('perf')['image_cap']


def paint(changed, visible, by_id, rects):
    """The escapes that clear and paint the ``changed`` parts ``[(row, col, id, crop)]``: joined rectangles
    (``compose``), else each part alone (``cap`` 0, or a rectangle past PALETTE_MAX colours)."""
    out, limit = [], cap()
    merged = compose(changed, visible, by_id, rects, limit) if limit and changed else None
    jobs = [(box, px, palette) for box, px, palette in merged] if merged is not None else []
    if merged is None:
        for row, col, n, (r0, c0, r1, c1) in changed:
            tile = by_id[n]
            ch, cw = tile.px.shape[0] // tile.rows, tile.px.shape[1] // tile.cols
            jobs.append(((row, col, row + r1 - r0, col + c1 - c0),
                         tile.px[r0 * ch:r1 * ch, c0 * cw:c1 * cw], tile.palette))
    for (r0, c0, r1, c1), px, palette in jobs:
        out.append(''.join(f'\x1b[{r + 1};{c0 + 1}H' + ' ' * (c1 - c0) for r in range(r0, r1)))
        if px.any():               # an empty part only clears what was under it, once
            out.append(f'\x1b[{r0 + 1};{c0 + 1}H' + sixel(px, palette))
    return out


def cells(parts, by_id):
    """The screen cells ``{(row, col)}`` of ``[(row, col, id, (r0, c0, r1, c1))]`` parts whose tile is pooled."""
    return {(row + r, col + c) for row, col, n, (r0, c0, r1, c1) in parts if n in by_id
            for r in range(r1 - r0) for c in range(c1 - c0)}


def spans(screen):
    """``(row, c0, c1)`` runs of consecutive cells per screen row."""
    out, by_row = [], {}
    for r, c in screen:
        by_row.setdefault(r, []).append(c)
    for row in sorted(by_row):
        cols = sorted(by_row[row])
        start = prev = cols[0]
        for c in cols[1:]:
            if c != prev + 1:
                out.append((row, start, prev + 1))
                start = c
            prev = c
        out.append((row, start, prev + 1))
    return out


def composite(parts, by_id, rect):
    """``(px, palette)`` of rectangle ``(r0, c0, r1, c1)`` from the parts inside it, or None past PALETTE_MAX."""
    r0, c0, r1, c1 = rect
    px = palette = None
    index = {}
    for row, col, n, (pr0, pc0, pr1, pc1) in parts:
        tile = by_id.get(n)
        top, left = max(row, r0), max(col, c0)                       # the part's cells inside the rectangle
        bottom, right = min(row + pr1 - pr0, r1), min(col + pc1 - pc0, c1)
        if tile is None or top >= bottom or left >= right:
            continue
        ch, cw = tile.px.shape[0] // tile.rows, tile.px.shape[1] // tile.cols
        if px is None:
            px, palette = np.zeros(((r1 - r0) * ch, (c1 - c0) * cw), np.uint8), [tile.palette[0]]
        lut = np.zeros(256, np.uint8)
        for i, rgb in enumerate(tile.palette[1:], 1):
            if rgb not in index:
                if len(palette) > PALETTE_MAX:
                    return None
                index[rgb] = len(palette)
                palette.append(rgb)
            lut[i] = index[rgb]
        tr, tc = pr0 + top - row, pc0 + left - col                   # the same cells in the tile
        px[(top - r0) * ch:(bottom - r0) * ch, (left - c0) * cw:(right - c0) * cw] = \
            lut[tile.px[tr * ch:(tr + bottom - top) * ch, tc * cw:(tc + right - left) * cw]]
    return None if px is None else (px, palette)


def compose(changed, visible, by_id, rects, cap):
    """``[((r0, c0, r1, c1), px, palette)]`` to paint: the changed parts' rectangles, or all visible parts' when
    those are fewer and the changed ones pass ``cap``. None when a rectangle needs more colours than sixel holds."""
    parts, boxes = changed, _boxes(changed, by_id, rects)
    if len(boxes) > cap:
        every = _boxes(visible, by_id, rects)
        if len(every) < len(boxes):
            parts, boxes = visible, every
    out = []
    for box in boxes:
        made = composite(parts, by_id, box)
        if made is None:
            return None
        out.append((box, *made))
    return out


def _boxes(parts, by_id, rects):
    screen = cells(parts, by_id)
    return rects(spans(screen)) if screen else []
