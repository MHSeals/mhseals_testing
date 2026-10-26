"""Fits for the physics characterization steps (procedures/physics.py). ROS-free.

Every fit returns its numbers with the counts, residuals and covariance behind them, so the crane
gate can judge it; none of them drops a term or guesses an attribution (crane-physics-design 4).
"""
from __future__ import annotations

import math

import numpy

G = 9.80665


def bifilar_iz(mass_kg, period_s, separation_m, length_m):
    """Yaw inertia from a bifilar swing: I = m g d^2 T^2 / (16 pi^2 L), d the string separation."""
    return mass_kg * G * separation_m ** 2 * period_s ** 2 / (16 * math.pi ** 2 * length_m)


def channel_noise(times, values):
    """Per-sample SD of one channel after removing a straight line (a tied boat swings slowly).

    Returns {sigma, n, lag1}: ``lag1`` is the lag-1 autocorrelation of the residual, so a reader can
    tell white sensor noise (near 0) from slow motion left in it. None with fewer than 10 samples.
    """
    pairs = [(t, v) for t, v in zip(times, values) if v is not None and math.isfinite(v)]
    if len(pairs) < 10:
        return None
    t, v = (numpy.array(column, float) for column in zip(*pairs))
    residual = v - numpy.polyval(numpy.polyfit(t - t[0], v, 1), t - t[0])
    sigma = float(numpy.sqrt(residual @ residual / (len(residual) - 2)))
    lag1 = float(residual[1:] @ residual[:-1] / (residual @ residual)) if residual.any() else 0.0
    return dict(sigma=sigma, n=len(pairs), lag1=lag1)


def env_floor(wave_floor, sigma):
    """(environment SD, ok): sqrt(wave_floor^2 - sigma^2); ok is False when the sensor alone exceeds the floor."""
    excess = wave_floor ** 2 - sigma ** 2
    return math.sqrt(max(excess, 0.0)), excess >= 0


def bollard_side(points, zero_n):
    """One thruster side from (pulse offset us > 0, force N) readings: T = K u^2, u = offset / 400.

    Readings at or below ``zero_n`` (the scale resolution) are inside the deadband: ``db_us`` is the
    midpoint between the largest such offset and the smallest offset that pulled (None without a
    zero reading). K is the force at full pulse, fitted over the pulling readings, with its standard
    error and the residual SD; with one pulling reading the error is None (0 dof), never 0.
    """
    points = sorted((float(o), float(f)) for o, f in points if o > 0 and math.isfinite(f))
    if not points:
        return None

    zeros = [o for o, f in points if f <= zero_n]
    pulling = [(o, f) for o, f in points if f > zero_n]
    if not pulling:
        return dict(K=0.0, K_se=None, residual_sd=None, n=len(points), dof=0, db_us=None)
    first = min(o for o, _ in pulling)
    below = [o for o in zeros if o < first]
    u = numpy.array([o / 400 for o, _ in pulling])
    f = numpy.array([f for _, f in pulling])
    basis = u * u
    k = float(basis @ f / (basis @ basis))
    dof = len(pulling) - 1
    residual = f - k * basis
    sd = float(numpy.sqrt(residual @ residual / dof)) if dof > 0 else None
    return dict(K=k, K_se=sd / float(numpy.sqrt(basis @ basis)) if sd is not None else None, residual_sd=sd,
                n=len(points), dof=dof, db_us=(max(below) + first) / 2 if below else None,
                residuals=[float(r) for r in residual])


def bollard_fit(readings, zero_n):
    """Per output: forward (pulse > 1500) and reverse sides, and the deadband centre offset o_us.

    ``readings`` are (output, pulse us, force N), ``zero_n`` the scale resolution in N. The sides
    are motor directions; which one pushes the boat forward is the record's ``reversed`` flag, so
    K_r / K_f is a measurement here, never assumed.
    """
    result = {}
    for output in sorted({r[0] for r in readings}):
        rows = [(p - 1500, f) for o, p, f in readings if o == output]
        forward = bollard_side([(d, f) for d, f in rows if d > 0], zero_n)
        reverse = bollard_side([(-d, f) for d, f in rows if d < 0], zero_n)
        offset = None
        if forward and reverse and forward['db_us'] is not None and reverse['db_us'] is not None:
            offset = (forward['db_us'] - reverse['db_us']) / 2
        ratio = reverse['K'] / forward['K'] if forward and reverse and forward['K'] > 0 else None
        result[int(output)] = dict(forward=forward, reverse=reverse, o_us=offset, reverse_ratio=ratio)
    return result


def _slope(t, y, degree=1):
    """(slope at the first t, standard error) of a polynomial through (t, y); None with too few points.

    Degree 2 reads the slope where the window starts, before drag bends the response over.
    """
    if len(t) < degree + 4:
        return None
    t, y = numpy.asarray(t, float) - t[0], numpy.asarray(y, float)
    basis = numpy.vander(t, degree + 1, increasing=True)
    coefficients, *_ = numpy.linalg.lstsq(basis, y, rcond=None)
    residual = y - basis @ coefficients
    variance = residual @ residual / (len(t) - degree - 1)
    return float(coefficients[1]), float(numpy.sqrt(variance * numpy.linalg.inv(basis.T @ basis)[1, 1]))


def onset(rows, t_on, skip, edge, before):
    """Acceleration per axis a pulse started, net of the drift before it: ([ax, ay, alpha], [sigma x3]).

    ``rows`` are (t, vx, vy, wz). The quadratic's slope at t_on + skip over [t_on + skip, t_on + skip
    + edge] (``skip`` covers spin-up and the estimator's lag) minus the straight-line slope over the
    ``before`` s leading up to t_on; the sigma combines both standard errors. None when either
    window has too few samples.
    """
    accel, sigma = [], []
    for axis in (1, 2, 3):
        after = [(r[0], r[axis]) for r in rows if t_on + skip <= r[0] <= t_on + skip + edge and r[axis] is not None]
        prior = [(r[0], r[axis]) for r in rows if t_on - before <= r[0] < t_on and r[axis] is not None]
        a = _slope(*zip(*after), degree=2) if after else None
        b = _slope(*zip(*prior)) if prior else None
        if a is None or b is None:
            return None
        accel.append(a[0] - b[0])
        sigma.append(math.hypot(a[1], b[1]))
    return accel, sigma


def _lstsq(basis, target):
    """Coefficients, covariance (residual variance x (X'X)^-1), residuals, dof and condition of X."""
    basis, target = numpy.asarray(basis, float), numpy.asarray(target, float)
    coefficients, *_ = numpy.linalg.lstsq(basis, target, rcond=None)
    residual = target - basis @ coefficients
    dof = len(target) - basis.shape[1]
    variance = float(residual @ residual / dof) if dof > 0 else None
    covariance = (variance * numpy.linalg.pinv(basis.T @ basis)).tolist() if variance is not None else None
    singular = numpy.linalg.svd(basis, compute_uv=False)
    return dict(coefficients=[float(c) for c in coefficients], covariance=covariance,
                residuals=[float(r) for r in residual], dof=dof, residual_var=variance,
                cond=float(singular[0] / singular[-1]) if singular[-1] > 0 else math.inf)


def speed_fit(points):
    """e|e| = d1 v + d2 v|v| over (speed, applied effort, level) hold means of one signed axis.

    Both terms are kept whatever their sign (a negative one is the gate's NEG_COEFF, not a refit);
    ``levels`` counts distinct commanded levels and ``pure_error_var`` is the variance of the speeds
    between repeats of a level, the noise a lack-of-fit test compares the residuals with.
    """
    points = [(v, e, level) for v, e, level in points if math.isfinite(v) and math.isfinite(e)]
    if len(points) < 2:
        return None
    v = numpy.array([p[0] for p in points])
    fit = _lstsq(numpy.column_stack((v, v * numpy.abs(v))), [e * abs(e) for _, e, _ in points])
    groups = {}
    for speed, _, level in points:
        groups.setdefault(level, []).append(speed)
    spread = [s - numpy.mean(g) for g in groups.values() for s in g]
    repeats = sum(len(g) - 1 for g in groups.values())
    d1, d2 = fit.pop('coefficients')
    return dict(fit, d1=d1, d2=d2, n=len(points), levels=len(groups),
                pure_error_var=float(numpy.dot(spread, spread) / repeats) if repeats else None,
                points=[[float(a), float(b), float(c)] for a, b, c in sorted(points)])


def deceleration(rows, span):
    """(v, dv/dt) pairs from (t, v) rows: a straight line over each ``span`` s, centred on its sample."""
    pairs = []
    t = [r[0] for r in rows]
    for i, (centre, value) in enumerate(rows):
        window = [(a, b) for a, b in rows[max(0, i - 50):i + 50] if abs(a - centre) <= span / 2]
        found = _slope(*zip(*window)) if len(window) >= 5 else None
        if found and t[0] + span / 2 <= centre <= t[-1] - span / 2:
            pairs.append((value, found[0]))
    return pairs


def coast_fit(windows, span=0.4, floor=0.0):
    """-dv/dt = c0 + D1/M v + D2/M v|v| over the coast windows (thrust off) of one signed axis.

    ``windows`` are lists of (t, v) after a hold ended; only |v| above ``floor`` is used, below it the
    wave floor dominates. ``c0`` takes a steady current or bias, so it is reported, not hidden.
    """
    pairs = [p for rows in windows for p in deceleration(rows, span) if abs(p[0]) > floor]
    if len(pairs) < 4:
        return None
    v = numpy.array([p[0] for p in pairs])
    fit = _lstsq(numpy.column_stack((numpy.ones_like(v), v, v * numpy.abs(v))), [-a for _, a in pairs])
    c0, d1, d2 = fit.pop('coefficients')
    fit.pop('residuals')        # one per derivative sample; the counts and variance say enough
    return dict(fit, c0=c0, d1_over_m=d1, d2_over_m=d2, n=len(pairs), windows=len(windows),
                v_range=[float(v.min()), float(v.max())])


def heading_spread(headings):
    """Mean resultant length R of the hold headings (rad): 1 all one way, 0 evenly spread; None without any."""
    if not headings:
        return None
    return float(math.hypot(sum(math.cos(h) for h in headings), sum(math.sin(h) for h in headings)) / len(headings))
