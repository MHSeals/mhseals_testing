"""Map view maths (map.md section 6): zoom ladder, fit, heading-up rotation and scale bar.

Pure and ROS-free. World points are (x, y) in the navsat ENU map frame, metres,
yaw is degrees counter-clockwise from east. A view is the (centre, half-width) of the
canvas in metres; half-width is horizontal, and canvas dots are square.
"""

import math

FIT = 'fit'
PRESETS = (5, 10, 20, 40, 80)  # half-width presets in m; `z` zooms in, `Z` out, `0` back to FIT
MARGIN = 0.10  # per side, around the fitted box
MIN_HALF = 5.0  # m, so a one-point course does not fit to nothing
SCALE_CELLS = (4, 10)  # the scale bar spans this many cells
NICE = (1, 2, 5)  # leading digits of a scale-bar length


def zoom_step(zoom, direction, fit_half=MIN_HALF):
    """The preset half-width one step in (``direction`` > 0) or out of ``zoom``.

    ``zoom`` is FIT, using ``fit_half`` as its half-width, or any half-width in metres.
    In picks the largest preset below it and stops at the smallest; out picks the next
    larger and stops at the largest. Returning to FIT is the caller's `0`.
    """
    half = fit_half if zoom == FIT else zoom
    if direction > 0:
        return max((p for p in PRESETS if p < half), default=PRESETS[0])
    return min((p for p in PRESETS if p > half), default=PRESETS[-1])


def fit(points, dots_wide, dots_high):
    """Centre and half-width that hold every point with a margin on both axes.

    Non-finite points are ignored; with none left, or a canvas under one dot, the view is
    the origin at MIN_HALF.
    """
    points = [p for p in points if math.isfinite(p[0]) and math.isfinite(p[1])]
    if not points or dots_wide < 1 or dots_high < 1:
        return (0.0, 0.0), MIN_HALF
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
    half = max((max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2 * dots_wide / dots_high)
    return centre, max(half * (1 + 2 * MARGIN), MIN_HALF)


def rotate(point, centre, yaw=None):
    """Screen-frame offset of ``point`` from ``centre``; heading-up turns yaw to straight up."""
    dx, dy = point[0] - centre[0], point[1] - centre[1]
    if yaw is None:
        return dx, dy
    a = math.radians(90 - yaw)
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def scale_bar(half_width, cols):
    """The longest 1/2/5 length in metres whose bar fits the cell range, with its cell count."""
    lengths = [m * 10 ** e for e in range(-2, 9) for m in NICE]
    if not math.isfinite(half_width) or half_width <= 0 or cols < 1:
        return lengths[0], 1  # degenerate canvas or view: the smallest bar, one cell
    per_cell = 2 * half_width / cols
    best = max((m for m in lengths if m / per_cell <= SCALE_CELLS[1]), default=lengths[0])
    return best, max(round(best / per_cell), 1)
