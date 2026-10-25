"""Map frame furniture (tui-visual 2.3): range rings, scale bar, north arrow and the GPS accuracy circle.

Pure and ROS-free. The view is boat-centred, so the rings are static on screen and cached per
``(cols, rows, half, up)``; the scale bar and north arrow are glyphs on the top-priority
``chrome`` layer so nothing covers them.
"""

import math

from mhseals_testing.ui.minimap.marks import arrow

STEPS = (1, 2, 5, 10, 20, 50)   # m, ring steps
RING_DOT = 7                    # a ring dot every 7 dots of circumference
LABEL_DEG = 20.0                # ring labels sit at this bearing
ACC_DOT, ACC_MIN = 3, 2.0       # accuracy circle: every 3rd dot; hidden under a cell (2 dots) radius


def ring_step(dots_high, scale):
    """The smallest 1/2/5/10/20/50 m step at least the canvas half-height over 2.2."""
    half_height = dots_high / 2 / scale if scale > 0 else 0.0
    return next((s for s in STEPS if s >= half_height / 2.2), STEPS[-1])


def draw_rings(canvas, scale, count=2, labels=False):
    """``count`` rings about the canvas centre on ``grid``, labelled at 20 deg when ``labels``."""
    step = ring_step(canvas.dots_high, scale)
    cx, cy = canvas.dots_wide / 2, canvas.dots_high / 2
    for k in range(1, count + 1):
        r = step * k * scale
        n = max(8, int(2 * math.pi * r / RING_DOT))
        for i in range(n):
            a = 2 * math.pi * i / n
            canvas.dot(math.floor(cx + r * math.cos(a)), math.floor(cy - r * math.sin(a)), 'grid')
        if labels:
            a = math.radians(LABEL_DEG)
            col, row = int((cx + r * math.cos(a)) // 2) + 1, int((cy - r * math.sin(a)) // 4)
            for i, ch in enumerate(f'{step * k:g}'):
                canvas.glyph(col + i, row, ch, 'grid')
    return step


def accuracy_text(h_acc):
    """``±2 cm`` under a metre (whole cm below 10 cm), ``±0.4 m`` / ``±3 m`` above."""
    if h_acc < 0.1:
        return f'±{max(1, round(h_acc * 100))} cm'
    return f'±{h_acc:.1f} m' if h_acc < 10 else f'±{h_acc:.0f} m'


def draw_chrome(canvas, step, scale, up_yaw=None, ascii_=False, accuracy=None):
    """Scale bar (one ring step) bottom-left with the accuracy label when it fits; north top-right.

    ``up_yaw`` is the boat yaw (map degrees) in heading-up, None in north-up; ``accuracy`` is h_acc m.
    """
    cells = max(2, round(step * scale / 2))
    ends, dash = ('|', '-') if ascii_ else ('├┤', '─')
    bar = ends[0] + dash * (cells - 2) + ends[-1] + f' {step:g} m'
    room = canvas.cols - 4    # leave the north arrow its corner
    if accuracy is not None:
        acc = accuracy_text(accuracy).replace('±', '+-' if ascii_ else '±')
        if len(bar) + 3 + len(acc) <= room:
            bar += '   ' + acc
    if len(bar) <= room:
        for i, ch in enumerate(bar):
            canvas.glyph(i, canvas.rows - 1, ch, 'chrome')
    north = 'N' + arrow(90.0 if up_yaw is None else 180.0 - up_yaw, ascii_)
    for i, ch in enumerate(north):
        canvas.glyph(canvas.cols - len(north) + i, 0, ch, 'chrome')


def draw_accuracy(proj, pos, accuracy):
    """The fix's horizontal accuracy as a faint dotted circle to scale, coloured by fix band.

    ``accuracy`` is ``(h_acc_m, band)``; nothing under a one-cell radius or for a bad value.
    """
    if not accuracy:
        return
    h_acc, band = accuracy
    r = h_acc * proj.scale if h_acc and math.isfinite(h_acc) else 0.0
    if r < ACC_MIN or r > proj.canvas.dots_wide:
        return
    x, y = proj.dot(pos)
    n = max(8, int(2 * math.pi * r / ACC_DOT))
    for i in range(n):
        a = 2 * math.pi * i / n
        proj.canvas.dot(math.floor(x + r * math.cos(a)), math.floor(y - r * math.sin(a)), f'acc:gps.{band}@50')
