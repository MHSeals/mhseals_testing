"""The Around panel: top-down in camera axes, FOV, range rings, the scan and each box with its range bar."""

import math

import numpy as np

from mhseals_testing.ui import theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels.fusion.common import (
    PM, TEXT_CELL, class_token, draw, draw_rate, label_cells, link_stride, optical_xz)


# --- Around (top-down) ----------------------------------------------------------------------------

def reach(model, rings):
    """The outer ring drawn: the first ring past every box (and 20 m without boxes)."""
    far = max((d['range'] for d in model['dets'] if d['range']), default=0.0)
    return next((r for r in rings if r >= far * 1.1), rings[-1]) if far else min(rings[-1], 20.0)


FAN_MARGIN = math.radians(10)    # rings drawn this far past the camera's half FOV


def half_fov(model):
    K = model.get('K')
    return math.atan2(K[2], K[0]) if K else math.radians(55)


def fan(w, h, span, half):
    """``at(x, z)`` -> raster (px, py) of a top-down point, the boat at the bottom centre; the rings' arc
    (half FOV plus FAN_MARGIN) fills the width or the height, whichever binds."""
    arc = min(math.pi / 2, half + FAN_MARGIN)
    scale = min((h - 2) / span, w / 2 * 0.98 / (span * math.sin(arc)))
    cx, base = w / 2, h - 1 - max(0.0, (h - 2 - span * scale) / 2)

    def at(x, z):
        return cx + x * scale, base - z * scale
    return at, scale, arc


def paint_around(model, cfg, span):
    def paint(r, inks):
        half = half_fov(model)
        at, scale, arc = fan(r.w, r.h, span, half)
        cx, base = at(0, 0)
        for sign in (-1, 1):
            r.line(cx, base, *at(sign * span * math.sin(half), span * math.cos(half)), inks['fov'], 1, 2)
        for ring in [x for x in cfg['rings'] if x <= span]:
            a = np.linspace(-arc, arc, max(16, int(ring * scale * 3 * arc)))
            r.squares(cx + ring * scale * np.sin(a), base - ring * scale * np.cos(a), 1, inks['ring'])
        scan = model.get('points')
        if scan is not None and len(scan[3]):
            xz = scan[3][::max(1, len(scan[3]) // 1500, link_stride(len(scan[3]), cfg))]
            xz = xz[np.hypot(xz[:, 0], xz[:, 1]) <= span]
            r.squares(cx + xz[:, 0] * scale, base - xz[:, 1] * scale, 1, inks['point'])
        dot = max(2, r.cw // 2)
        for det in model['dets']:
            if det['range'] is None:
                continue
            x, z = optical_xz(det, det['pos'])
            px, pz = at(x, z)
            ink = inks[f"det.{class_token(det['cls'])}"]
            if det['stereo_pos'] is not None and det['source'] not in ('stereo', 'pass'):
                sx, sz = at(*optical_xz(det, det['stereo_pos']))
                r.line(px, pz, sx, sz, inks['stereo'])
                r.squares([sx - 1], [sz - 1], max(1, dot // 2), inks['stereo'])
            if det['sigma']:
                ux, uz = x / det['range'], z / det['range']
                k = 2 * det['sigma']
                r.line(*at(x - ux * k, z - uz * k), *at(x + ux * k, z + uz * k), inks['bar'], max(1, dot // 3))
            if det['source'] == 'lidar_ray':
                h = dot
                r.polygon([(px, pz - h), (px + h, pz), (px, pz + h), (px - h, pz)], ink)
            else:
                r.rect(px - dot / 2, pz - dot / 2, px + dot / 2 + 1, pz + dot / 2 + 1, ink)
        b = max(2, r.cw // 2)
        r.polygon([(cx, base - b), (cx + b / 2, base), (cx - b / 2, base)], inks['boat'])
    return paint


def around_labels(model, span, cols, rows, cell):
    """Hole labels: the outer ring's range at the top right, each box's ``7.0+-0.2`` beside its mark."""
    labels = label_cells(f'{span:g}m', cols - len(f'{span:g}m'), 0, cols, rows, theme.MUTED)
    at, _, _ = fan(cols * cell[0], rows * cell[1], span, half_fov(model))
    for det in sorted((d for d in model['dets'] if d['range']), key=lambda d: d['range']):
        px, pz = at(*optical_xz(det, det['pos']))
        text = f"{det['range']:.1f}" + (f"{PM}{2 * det['sigma']:.1f}" if det['sigma'] else '')
        col, row = int(px / cell[0]) + 2, int(pz / cell[1])
        if theme.ASCII:     # the mark spans to the cell of px + 1; one blank cell after it, as braille leaves
            col, text = int((px + 1) / cell[0]) + 1, ' ' + text
        if col + len(text) > cols:
            col = int(px / cell[0]) - 1 - len(text)
        cells = label_cells(text, col, row, cols, rows, theme.PALETTE.style(class_token(det['cls'])))
        if cells and not any((c + dc, rr) in labels for c, rr in cells for dc in (-1, 0, 1)):
            labels.update(cells)
    return labels


def around_rows(model, cfg, cols, rows):
    span = reach(model, cfg['rings'])
    tokens = sorted({class_token(d['cls']) for d in model['dets']})
    layers = ([('ring', 'grid'), ('fov', 'fov'), ('point', 'muted'), ('stereo', 'muted'), ('bar', 'slate')]
              + [(f'det.{t}', t) for t in tokens] + [('boat', 'boat')])
    cell = pixels.CELL if pixels.active() and pixels.images('fusion.around') else TEXT_CELL
    return draw('fusion.around', paint_around(model, cfg, span), layers, cols, rows,
                (model['version'], span), around_labels(model, span, cols, rows, cell), draw_rate(cfg, 'fusion.around', 0.5))
