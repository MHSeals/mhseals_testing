"""The Camera panel: the scan projected into the image over the grey frame, the boxes and their range labels,
and its subtitle (the range legend, the image note)."""

import numpy as np
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels.fusion.common import (
    GREYS, RAMP, STALE, STAMP_MATCH_S, TEXT_CELL, class_token, draw, draw_rate, ink_style, label_cells,
    link_block, link_stride, range_label, scan_stale)


# --- Camera ---------------------------------------------------------------------------------------

def camera_layers(dets):
    """Boxes under the points in braille (a cell takes its highest layer), so a cluster beside its box stays in
    range colours; in ASCII over them, since a box there is a few '#' and points on its corners hide it. Images
    draw boxes over the points either way."""
    boxes = [(f'box.{token}', token) for token in sorted({class_token(d['cls']) for d in dets})]
    ramp = [('stale', STALE)] + [(f'ramp{i}', token) for i, token in enumerate(reversed(RAMP))]
    return ramp + boxes if theme.ASCII else boxes + ramp


def frame_shown(model):
    """Whether the Camera tile draws the frame: the image on, an image tier for the tile, a frame with camera_info's
    aspect (the points and boxes are normalised by that size, so another crop would not line up). Over a link budget
    it joins after the tile's first repaint, so the link does not take both at once."""
    return (model.get('frame') is not None and model.get('frame_aspect') is not False
            and pixels.active() and pixels.images('fusion.camera')
            and (not pixels.BUDGET or not model.get('image') or _STARTED.get('fusion.camera') == pixels.GENERATION))


def camera_model(model):
    """The model the Camera tile draws: with the frame, its own stamp's boxes and scan (the feed's ``at_frame``)."""
    view = model.get('at_frame') if frame_shown(model) else None
    return {**model, **view} if view else model


def block_mean(frame, h, w, fy, fx):
    """``frame`` as h x w means, the block (i, j) covering rows i*fy.. of the frame's height and columns j*fx.. of its width
    (fy, fx: the block's share of the raster)."""
    edges = [np.minimum((np.arange(n) * f * size).astype(int), size - 1)
             for n, f, size in ((h, fy, frame.shape[0]), (w, fx, frame.shape[1]))]
    total = np.add.reduceat(np.add.reduceat(frame.astype(np.float32), edges[0], axis=0), edges[1], axis=1)
    return total / (np.maximum(np.diff(edges[0], append=frame.shape[0]), 1)[:, None]
                    * np.maximum(np.diff(edges[1], append=frame.shape[1]), 1)[None, :])


def paint_frame(r, frame, share=1.0):
    """The frame in GREYS levels from bg towards muted, stretched to the raster (the camera_info aspect); over a link
    budget in blocks of link_block() px; ``share`` of its height from the top (the rest is left clear)."""
    bg, mid = np.asarray(pixels.token('bg'), float), np.asarray(pixels.token('muted'), float)
    greys = [r.ink(tuple(int(v) for v in bg + (mid - bg) * (i / (GREYS - 1)) * 0.8)) for i in range(GREYS)]
    block = link_block()
    h, w = -(-r.h // block), -(-r.w // block)
    if block > 1:       # each block takes its mean, so sensor noise does not flip a block's level (a repaint resends it)
        level = (block_mean(frame, h, w, block / r.h, block / r.w).astype(np.uint16) * GREYS // 256).astype(np.uint8)
        level = np.repeat(np.repeat(level, block, axis=0), block, axis=1)[:r.h, :r.w]
    else:
        ys = np.minimum(((np.arange(r.h) + 0.5) * frame.shape[0] / r.h).astype(int), frame.shape[0] - 1)
        xs = np.minimum(((np.arange(r.w) + 0.5) * frame.shape[1] / r.w).astype(int), frame.shape[1] - 1)
        level = (frame[ys][:, xs].astype(np.uint16) * GREYS // 256).astype(np.uint8)
    rows = -(-int(r.h * share) // block) * block if share < 1 else r.h
    r.px[:rows] = np.asarray(greys, np.uint8)[level][:rows]


_STARTED = {}    # drawing name -> pixels.GENERATION it first painted in (an image tile, over a link budget)
JOIN_STEPS = 4   # over a link budget a frame at a new block size comes in this many repaints, top rows first
_JOINED = {}     # drawing name -> (block size of its frame, repaints of it so far)


def paint_camera(model, cfg):
    breaks = np.asarray(cfg['ramp'], float)

    def paint(r, inks):
        if r.cw > TEXT_CELL[0] and frame_shown(model):
            share = 1.0
            if pixels.BUDGET:       # all of a frame's tiles are new at once (a repaint of 1-1.5 x the budget)
                block, steps = _JOINED.get('fusion.camera', (None, 0))
                steps = steps + 1 if block == link_block() else 1
                _JOINED['fusion.camera'] = (link_block(), steps)
                share = min(1.0, steps / JOIN_STEPS)
            paint_frame(r, model['frame'], share)
        if (pixels.BUDGET and model.get('image') and pixels.active() and pixels.images('fusion.camera')
                and _STARTED.get('fusion.camera') != pixels.GENERATION):
            _STARTED['fusion.camera'] = pixels.GENERATION
            return          # the tile's first repaint over a link is empty: its neighbours' first ones are all new tiles too
        scan = model.get('points')
        if scan is not None and len(scan[1]):
            step = link_stride(len(scan[1]), cfg, frame_shown(model))
            uv, rng = scan[1][::step], scan[2][::step]
            size = max(1, r.cw // 4)
            if scan_stale(model, cfg):
                r.squares(uv[:, 0] * r.w - size // 2, uv[:, 1] * r.h - size // 2, size, inks['stale'])
            else:
                bucket = np.searchsorted(breaks, rng, side='right')     # 0: nearer than the first break
                for b in reversed(range(len(RAMP))):                     # far first, so near points draw on top
                    sel = bucket == b
                    if sel.any():
                        r.squares(uv[sel, 0] * r.w - size // 2, uv[sel, 1] * r.h - size // 2, size,
                                  inks[f'ramp{len(RAMP) - 1 - b}'])
        width = max(1, r.cw // 5)
        for det in model['dets']:
            if det['box'] is None:
                continue
            u0, v0, u1, v1 = det['box']
            x0, y0, x1, y1 = (min(u0, u1) * r.w, min(v0, v1) * r.h, max(u0, u1) * r.w - 1, max(v0, v1) * r.h - 1)
            # ray mode dashed in images; text keeps a solid outline (a dash leaves a braille box a few dots)
            ray = det['source'] == 'lidar_ray' and r.cw > TEXT_CELL[0]
            ink, dash = inks[f"box.{class_token(det['cls'])}"], (3 * width if ray else 0)
            for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
                r.line(*a, *b, ink, width, dash)
    return paint


def camera_rows(model, cfg, cols, rows):
    model = camera_model(model)
    labels = {}
    for det in sorted((d for d in model['dets'] if d['box'] is not None), key=lambda d: d['range'] or 1e9):
        u0, v0, u1, v1 = det['box']
        style = theme.PALETTE.style(class_token(det['cls']))
        text, col = range_label(det), int(min(u0, u1) * cols)
        # over the box, else under it; a label that would touch another drops the stereo range, then itself
        for row, label in [(r, t) for t in (text, text.split('/')[0]) for r in
                           (int(min(v0, v1) * rows) - 1, int(max(v0, v1) * rows) + 1)]:
            cells = label_cells(label + ' ', col, row, cols, rows, style)
            if cells and not any((c + dc, rr) in labels for c, rr in cells for dc in (-1, 0)):
                labels.update(label_cells(label, col, row, cols, rows, style))
                break
    layers = camera_layers(model['dets'])
    hz = draw_rate(cfg, 'fusion.camera')
    block = link_block()
    return draw('fusion.camera', paint_camera(model, cfg), layers, cols, rows,
                (model['version'], frame_shown(model), scan_stale(model, cfg), block), labels, hz)


def camera_subtitle(model, cfg):
    """The range legend (in the points' own styles), the point count, the scan's offset from the boxes and ``no frame``
    in an image tier without one. A stale scan leads with ``TURNING``/``STALE`` and its offset (the title row cuts
    the tail at 80 columns; the word carries it without colour)."""
    model = camera_model(model)
    dt, stale = model.get('scan_dt'), scan_stale(model, cfg)
    offset = f'scan {dt:+.2f}s' if dt is not None else ''
    sub = Text()
    if stale:
        sub.append('TURNING' if model.get('motion') else 'STALE', style=f'bold {ink_style(STALE)}'.strip())
        sub.append(f' {offset}{theme.SEPARATOR}' if offset else theme.SEPARATOR, style=theme.MUTED)
    note = image_note(model, cfg)
    if note:
        sub.append(note[0], style=note[1])
        sub.append(theme.SEPARATOR, style=theme.MUTED)
    ramp = Text()
    for i, token in enumerate(RAMP[:-1]):
        ramp.append(theme.pick('⣿', '#'), style=ink_style(token))
        ramp.append(f"{cfg['ramp'][i]:g} ", style=theme.MUTED)
    ramp.append('m', style=theme.MUTED)
    sub.append_text(Text(' ').join([ramp, Text(f"{len(model['points'][1]) if model.get('points') else 0} pts",
                                               style=theme.MUTED)]))
    if offset and not stale:
        sub.append(f'{theme.SEPARATOR}{offset}', style=theme.MUTED)
    return sub


def image_note(model, cfg):
    """``(text, style)`` of the image toggle's state for the Camera title, or None while the frame is drawn in time
    with its boxes: off, needs graphics (the text tiers), no frame yet, loading (over a link), another aspect than camera_info, or the
    frame's boxes ``frame +0.40s`` from it (no array of its own stamp)."""
    if not model.get('image'):
        return 'image off', theme.MUTED
    if not (pixels.active() and pixels.images('fusion.camera')):
        return 'image needs graphics', theme.MUTED
    if model.get('frame') is None:
        return 'no frame', theme.MUTED
    if model.get('frame_aspect') is False:
        return 'frame aspect != camera_info', theme.WARN
    if pixels.BUDGET:       # over a link the frame starts after an empty repaint and comes in by rows
        block, steps = _JOINED.get('fusion.camera', (None, 0))
        if _STARTED.get('fusion.camera') != pixels.GENERATION or block != link_block() or steps < JOIN_STEPS:
            return 'image loading', theme.MUTED
    dt = model.get('frame_dt')
    if dt is not None and abs(dt) > STAMP_MATCH_S:
        return f'frame {dt:+.2f}s', theme.WARN if abs(dt) > cfg['feed']['scan_dt_max'] else theme.MUTED
    return None
