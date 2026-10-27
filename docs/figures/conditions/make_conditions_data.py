#!/usr/bin/env python3
"""Data for the Part II (conditions and readiness) figures.

Writes whitespace-separated .dat tables next to this file, one per figure, for
pgfplots ``\\addplot table``. numpy only, fixed seeds, so the output is identical
on every run:

    python3 docs/figures/conditions/make_conditions_data.py

These are illustrations of the math in sections/conditions*.tex on synthetic
data. They are not boat measurements and not the production estimator
(mhseals_testing/analysis/conditions.py), which they re-implement only as far
as each figure needs.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
HZ = 5.0                # GPS velocity rate
DT = 1 / HZ


def write(name, header, *cols):
    rows = np.column_stack(cols)
    with open(HERE / name, 'w') as f:
        f.write(' '.join(header) + '\n')
        for r in rows:
            f.write(' '.join(f'{x:.6g}' for x in r) + '\n')


def phi(x):
    """Standard normal CDF."""
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))


def ou(n, tau, sigma, rng, dt=DT):
    """Exactly sampled Ornstein-Uhlenbeck (continuous AR(1)) series: lag-1 rho = exp(-dt/tau)."""
    rho = math.exp(-dt / tau)
    x = np.empty(n)
    x[0] = rng.normal(0, sigma)
    e = rng.normal(0, sigma * math.sqrt(1 - rho ** 2), n)
    for i in range(1, n):
        x[i] = rho * x[i - 1] + e[i]
    return x


def f_factor(b, tau):
    """Var(mean over b) = (S0 / b) f for e^{-|s|/tau} correlation (conditions spec 1.2)."""
    return 1 - tau / b * (1 - math.exp(-b / tau))


def scatter_ratio(k, b, tau):
    """E[sum (m_i - mbar)^2] / ((k - 1) S0 / b) for k consecutive batch means of AR(1) noise."""
    e = math.exp(-b / tau)
    var = f_factor(b, tau)
    pairs = sum((k - d) * tau / (2 * b) * (1 - e) ** 2 * e ** (d - 1) for d in range(1, k))
    return (k * var - (k * var + 2 * pairs) / k) / (k - 1)


# 1. coast: exponential vs quadratic decay --------------------------------------------
def coast():
    """Coast after a surge hold with the simulator's surge drag (sec:sim): M dv/dt = -(d1 v + d2 v|v|).

    Closed form: v(t) = (d1/d2) / ((1 + d1/(d2 v0)) e^{d1 t / M} - 1). The exponential uses the
    time constant linearised at v0, M / (d1 + 2 d2 v0), which is what a spin-up t95 measures.
    """
    m, d1, d2, v0 = 35.5, 10.0, 19.4, 0.38      # 30 kg + 5.5 kg surge added mass
    tau = m / (d1 + 2 * d2 * v0)
    t = np.linspace(0, 12, 241)
    v_exp = v0 * np.exp(-t / tau)
    v_true = (d1 / d2) / ((1 + d1 / (d2 * v0)) * np.exp(d1 * t / m) - 1)
    write('coast.dat', ['t', 'vexp', 'vtrue'], t, v_exp, v_true)
    at = 4 * tau
    vt = (d1 / d2) / ((1 + d1 / (d2 * v0)) * math.exp(d1 * at / m) - 1)
    write('coast_mark.dat', ['tau', 't4', 'vexp4', 'vtrue4'], [tau], [at], [v0 * math.exp(-4)], [vt])


# 2. the f factor and its sensitivity ----------------------------------------------
def f_curves():
    b = np.linspace(0.2, 10, 197)
    cols = [b]
    for tau in (0.5, 1.1, 2.0):
        cols.append([f_factor(x, tau) for x in b])
    write('ffactor.dat', ['b', 'f05', 'f11', 'f20'], *cols)


# 3. S0 recovery on AR(1) data: naive, 2 s / 4 s batches, with and without correction --
def s0_recovery():
    rng = np.random.default_rng(20261023)
    taus = np.array([0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0])
    windows, tw = 40, 8.0
    n = int(tw * HZ)
    out = {k: [] for k in ('naive', 'b2', 'b2f', 'b4', 'b4f')}
    for tau in taus:
        sigma = 1.0
        s0 = 2 * sigma ** 2 * tau
        acc = {k: [] for k in out}
        for _ in range(60):
            x = ou(windows * n, tau, sigma, rng).reshape(windows, n)
            # naive: treat samples as independent -> S0 = Var(sample) * dt
            acc['naive'].append(x.var(ddof=1) * DT)
            for b, key in ((2.0, 'b2'), (4.0, 'b4')):
                m = int(b * HZ)
                k = n // m
                sub = x[:, :k * m].reshape(windows, k, m).mean(axis=2)
                dev = sub - sub.mean(axis=1, keepdims=True)
                raw = b * (dev ** 2).sum() / (windows * (k - 1))
                acc[key].append(raw)
                acc[key + 'f'].append(raw / scatter_ratio(k, b, tau))
        for key in out:
            out[key].append(np.median(acc[key]) / s0)
    write('s0recovery.dat', ['tau', 'naive', 'b2', 'b2f', 'b4', 'b4f'], taus,
          *(out[k] for k in ('naive', 'b2', 'b2f', 'b4', 'b4f')))


# 4. a neutral record: 5 Hz samples, 4 s batch means, 8 s window means -----------------
def series():
    rng = np.random.default_rng(7)
    t = np.arange(0, 32, DT)
    v = 0.08 + ou(len(t), 1.1, 0.03, rng) + 0.006 * np.sin(2 * math.pi * t / 2.5)
    v = np.floor(v * 100) / 100                 # GPSRAW: cm/s, truncated
    write('series.dat', ['t', 'v'], t, v)
    tb = np.arange(0, 32, 4.0)
    mb = [v[(t >= a) & (t < a + 4)].mean() for a in tb]
    write('batches.dat', ['t0', 't1', 'm'], tb, tb + 4, mb)
    tw = np.arange(0, 32, 8.0)
    mw = [v[(t >= a) & (t < a + 8)].mean() for a in tw]
    write('windows.dat', ['t0', 't1', 'm'], tw, tw + 8, mw)


# 5. autocorrelation: 5 Hz OU samples vs the same record held to 50 Hz ------------------
def acf():
    rng = np.random.default_rng(11)
    x = ou(200000, 1.1, 1.0, rng)
    lags = np.arange(0, 16)                     # 0 .. 3 s at 5 Hz
    xc = x - x.mean()
    r = [1.0] + [float((xc[l:] * xc[:-l]).mean() / xc.var()) for l in lags[1:]]
    write('acf.dat', ['lag', 'rho', 'model'], lags * DT, r, np.exp(-lags * DT / 1.1))


# 6. windage geometry: d(psi) = a + Q(2 psi) b traced around a full turn --------------
def windage():
    a = np.array([0.06, 0.08])                  # c + kbar w, m/s, (E, N)
    w = np.array([-3.5, -3.5]) / math.sqrt(2) * 1.0   # wind over water toward SW, 3.5 m/s
    dk = -0.004                                 # (k_x - k_y)/2 < 0: beam leeway larger
    b = dk * w
    psi = np.linspace(0, 2 * math.pi, 361)
    q = np.stack([np.stack([np.cos(2 * psi), np.sin(2 * psi)]),
                  np.stack([np.sin(2 * psi), -np.cos(2 * psi)])])   # (2, 2, n)
    d = a[:, None] + np.einsum('ijn,j->in', q, b)
    write('windage.dat', ['psi', 'dE', 'dN'], np.degrees(psi), d[0], d[1])
    # six windows at headings of a survey (yaw holds spread psi)
    rng = np.random.default_rng(3)
    hs = np.radians([10, 70, 130, 190, 250, 310])
    pts = []
    for h in hs:
        qq = np.array([[math.cos(2 * h), math.sin(2 * h)], [math.sin(2 * h), -math.cos(2 * h)]])
        pts.append(a + qq @ b + rng.normal(0, 0.001, 2))
    pts = np.array(pts)
    write('windage_pts.dat', ['psi', 'dE', 'dN'], np.degrees(hs), pts[:, 0], pts[:, 1])
    write('windage_vec.dat', ['aE', 'aN', 'bE', 'bN'], [a[0]], [a[1]], [b[0]], [b[1]])


# 7. stationarity: window means, pooled estimate, change after a step ------------------
def stationarity():
    rng = np.random.default_rng(5)
    tw, s0 = 8.0, 0.0008                       # m^2/s -> sigma_m = 0.01 m/s per 8 s window
    sm = math.sqrt(s0 / tw)
    t = np.arange(14) * tw + tw / 2
    truth = np.where(t < 70, 0.10, 0.145)
    m = truth + rng.normal(0, sm, len(t))
    chi = np.full(len(t), np.nan)
    pool_start = 0
    flagged = False
    pooled = np.full(len(t), np.nan)
    for i in range(3, len(t)):
        if i - pool_start < 3:                  # a pool needs three windows, as in production
            continue
        d = m[pool_start:i].mean()
        pooled[i] = d
        chi[i] = tw * (m[i] - d) ** 2 / s0
        out = chi[i] > 13.8 and abs(m[i] - d) >= 0.02
        if out and flagged:
            pool_start, flagged = i - 1, False
        else:
            flagged = out
    write('stationarity.dat', ['t', 'm', 'truth', 'chi', 'pooled'], t, m, truth, chi, pooled)


# 8. trust: p = 2 Phi(x) - 1 and the drift trust T_d ------------------------------------
def trust():
    x = np.linspace(0, 3.5, 141)
    # a perfect column's measured cross response is y = sigma z, z ~ N(0, 1); its trust factor
    # phi(x - z) - phi(-x - z) falls with |z|, so its median sits at the median |z| = 0.6745
    write('ceiling.dat', ['x', 'p', 'median'], x, [2 * phi(v) - 1 for v in x],
          [phi(v - 0.6745) - phi(-v - 0.6745) for v in x])
    pts = []
    for v in (0.38, 0.21, 0.16, 0.09):
        xx = 0.015 * v / 0.0025
        pts.append((xx, 2 * phi(xx) - 1))
    pts = np.array(pts)
    write('ceiling_pts.dat', ['x', 'p'], pts[:, 0], pts[:, 1])
    se = np.linspace(0.001, 0.03, 146)
    write('drifttrust.dat', ['se', 'td'], se, 1 - np.exp(-0.02 ** 2 / (2 * se ** 2)))


# 9. narrowband cancellation: |mean of A sin| over a window vs its bound --------------
def narrowband():
    T = np.linspace(0.5, 10, 191)
    cols = [T]
    for tp in (1.0, 3.0, 4.0):
        w = 2 * math.pi / tp
        worst = [max(abs((math.cos(p) - math.cos(w * x + p)) / (w * x)) for p in np.linspace(0, 2 * math.pi, 73))
                 for x in T]
        cols += [worst, tp / (math.pi * T)]
    write('narrowband.dat', ['T', 'w1', 'b1', 'w3', 'b3', 'w4', 'b4'], *cols)


# 10. coverage of the pooled drift interval with and without the (1+r)/(1-r) factor -----
def coverage():
    rng = np.random.default_rng(99)
    taus = (1.0, 3.0, 6.0)
    b, tw, windows = 4.0, 8.0, 12
    n, m = int(tw * HZ), int(b * HZ)
    plain, inflated = [], []
    for tau in taus:
        hit_p = hit_i = 0
        seeds = 400
        for _ in range(seeds):
            x = ou(windows * n, tau, 0.03, rng).reshape(windows, n)
            sub = x.reshape(windows, n // m, m).mean(axis=2)
            dev = sub - sub.mean(axis=1, keepdims=True)
            k = n // m
            tau_c = min(tau, b)                 # the production cap: tau at most one batch
            s0 = b * (dev ** 2).sum() / (windows * (k - 1)) / scatter_ratio(k, b, tau_c)
            means = x.mean(axis=1)
            d = means.mean()
            chi2 = tw * ((means - d) ** 2).sum() / s0
            disp = max(1.0, chi2 / (windows - 1))
            var = s0 / (tw * windows) * disp
            r = means - d
            rr = float((r[1:] * r[:-1]).sum() / (r ** 2).sum()) + 1 / windows
            rr = min(max(rr, 0.0), 0.6)
            hit_p += abs(d) <= 1.96 * math.sqrt(var)
            hit_i += abs(d) <= 1.96 * math.sqrt(var * (1 + rr) / (1 - rr))
        plain.append(hit_p / seeds)
        inflated.append(hit_i / seeds)
    write('coverage.dat', ['tau', 'plain', 'inflated'], taus, plain, inflated)


# 11. the AR(1) correction applied to a narrowband wave: does it err safe? --------------
def narrowband_s0():
    """Pure swell of period P (random phase per window) plus white receiver noise.

    Compares the window-mean SD that S0 predicts (4 s batches, AR(1) correction with rho
    measured from the samples, tau capped at one batch) with the true SD of 8 s window means.
    """
    rng = np.random.default_rng(1)
    b, tw, windows = 4.0, 8.0, 2000
    n, m = int(tw * HZ), int(b * HZ)
    tt = np.arange(n) * DT
    periods, pred, true = (1.0, 1.5, 2.0, 2.5, 3.0, 4.0), [], []
    for p in periods:
        ph = rng.uniform(0, 2 * math.pi, windows)
        x = 0.05 * np.sin(2 * math.pi * tt[None, :] / p + ph[:, None]) + rng.normal(0, 0.005, (windows, n))
        xc = x - x.mean(axis=1, keepdims=True)
        rho = float((xc[:, 1:] * xc[:, :-1]).sum() / (xc ** 2).sum())
        tau = min(-DT / math.log(rho), b) if 0 < rho < 1 else 0.0
        sub = x.reshape(windows, n // m, m).mean(axis=2)
        dev = sub - sub.mean(axis=1, keepdims=True)
        raw = b * (dev ** 2).sum() / (windows * (n // m - 1))
        s0 = raw / scatter_ratio(n // m, b, tau) if tau > 0 else raw
        pred.append(math.sqrt(s0 / tw))
        true.append(float(x.mean(axis=1).std()))
    write('narrowband_s0.dat', ['period', 'pred', 'true', 'ratio'], periods, pred, true,
          np.array(pred) / np.array(true))


if __name__ == '__main__':
    for fn in (coast, f_curves, s0_recovery, series, acf, windage, stationarity, trust, narrowband, coverage, narrowband_s0):
        fn()
        print('wrote', fn.__name__)
