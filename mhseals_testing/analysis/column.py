"""Find the correction column of one direction from a few holds. ROS-free.

Driving a main axis at effort ``a`` with cross efforts ``c*a`` gives a cross response ``y`` (per
unit of main response) that is affine in ``c`` over the small range involved: ``y = y0 + J c``.
Three holds fix ``y0`` and the 2x2 ``J`` without any model of the hull; the root ``c = -J^-1 y0``
is then tried, and every further hold refits the same affine map (docs/math.pdf, "One column per
axis and sign"). Entries inside tolerance stop the search; if the holds run out, the best
column actually measured is kept, never an extrapolation.
"""
import numpy


def worst(response, tolerance):
    """The largest cross response in units of its tolerance; <= 1 means every entry passes."""
    return max(abs(y) / t for y, t in zip(response, tolerance))


def _within(column, limit):
    """``column`` scaled down so sum|c| <= limit (a(1 + sum|c|) must stay in range); None if no room."""
    total = float(numpy.abs(column).sum())
    if limit <= 0:
        return None
    scale = min(1.0, limit / total) if total else 1.0
    return tuple(float(value * scale) for value in column)


def _next(points, probe, limit):
    """The next column to try, or None when the holds so far cannot determine one."""
    first = numpy.array(points[0][0], dtype=float)
    if len(points) == 1:
        return _within((first[0] + probe, first[1]), limit)
    if len(points) == 2:
        return _within((first[0], first[1] + probe), limit)
    design = numpy.array([[1.0, *column] for column, _ in points])
    fit = numpy.linalg.lstsq(design, numpy.array([y for _, y in points], dtype=float), rcond=None)[0]
    jacobian = fit[1:].T
    if abs(numpy.linalg.det(jacobian)) < 1e-9:
        return None
    column = -numpy.linalg.solve(jacobian, fit[0])
    return _within(column, limit)


def refine(measure, start, tolerance, probe=0.15, limit=1.0, max_holds=4, points=()):
    """Search for the column that zeroes the cross response.

    ``measure(column) -> (y1, y2)`` runs one hold. ``points`` are holds already measured
    ((column, response) pairs, e.g. the survey's at the zero column). At most ``max_holds``
    new holds run. Returns the best measured column, its response, and whether it passes.
    """
    points, column = list(points), tuple(start)
    if points:  # the start was already measured: go on from it, unless it already passes
        if worst(points[0][1], tolerance) <= 1.0:
            return dict(column=points[0][0], response=points[0][1], holds=1, converged=True)
        column = _next(points, probe, limit)
    for _ in range(max_holds):
        if column is None:
            break
        points.append((column, tuple(measure(column))))
        if worst(points[-1][1], tolerance) <= 1.0:
            break
        column = _next(points, probe, limit)
    best = min(points, key=lambda point: worst(point[1], tolerance))
    return dict(column=best[0], response=best[1], holds=len(points),
                converged=bool(worst(best[1], tolerance) <= 1.0))
