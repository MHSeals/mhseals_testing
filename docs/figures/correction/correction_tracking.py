"""Tracking-score, staleness and magnetic-interference figures (Part I).

Writes figures/correction/data/correction_ghost.dat, correction_stale.dat,
correction_stale_lines.dat and correction_mag.dat.
Pure numpy, seeded. Run from docs/:  python3 figures/correction_tracking.py

The score and the weighted Theil-Sen slope follow analysis/tracking.py
(k = 2, RMS over the counted samples, pair weights w_i w_j).
"""
import math
import os
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
RHO = 40.2 / 51.5
S_X, M11, D1, D2 = 2 * math.sqrt(2) * 51.5, 37.7, 10.0, 19.4


def write(name, header, rows):
    with open(os.path.join(OUT, name), "w") as f:
        f.write("# " + header + "\n")
        for r in rows:
            f.write(" ".join(f"{x:.6f}" for x in r) + "\n")


def v_ss(a, k=1.0):
    F = k * S_X * a * abs(a) * (1 + RHO) / 2
    return math.copysign((-D1 + math.sqrt(D1 * D1 + 4 * D2 * abs(F))) / (2 * D2), F)


def score(meas, pred, sigma_wave, k=2.0):
    err = np.sqrt(np.mean((meas - pred) ** 2))
    return max(0.0, 1 - err / (np.sqrt(np.mean(pred ** 2)) + k * sigma_wave))


def ghost():
    """Commanded surge steps; the boat has lost 25% thrust since the curves were fitted."""
    rng = np.random.default_rng(3)
    dt, tau, sw = 0.05, 1.2, 0.03
    t = np.arange(0, 60, dt)
    e = np.where(t < 5, 0, np.where(t < 25, 0.5, np.where(t < 40, 0.3, 0.0)))
    vh = np.zeros_like(t); v = np.zeros_like(t)
    for n in range(1, t.size):
        vh[n] = vh[n - 1] + dt / tau * (v_ss(e[n]) - vh[n - 1])          # forward model
        v[n] = v[n - 1] + dt / M11 * (S_X * 0.75 * e[n] ** 2 * (1 + RHO) / 2
                                      - D1 * v[n - 1] - D2 * v[n - 1] * abs(v[n - 1]))
    meas = v + sw * rng.standard_normal(t.size)
    rows = []
    for n in range(0, t.size, 2):
        w = (t > t[n] - 10) & (t <= t[n])
        counted = w & ((np.abs(e) > 0.06) | (np.abs(vh) > 2 * sw))
        sc = score(meas[counted], vh[counted], sw) if counted.sum() >= 0.5 * w.sum() and w.sum() > 20 else -1
        rows.append((t[n], meas[n], vh[n], sc))
    write("correction_ghost.dat", "t  v_measured  v_predicted  score_10s (-1 = gated)", rows)
    print("ghost: score at t=20, 35:", [round(r[3], 3) for r in rows if abs(r[0] - 20) < 0.06 or abs(r[0] - 35) < 0.06])


def theil_sen(x, y, w):
    sl, ww = [], []
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            if x[j] != x[i]:                      # equal sessions carry no slope
                sl.append((y[j] - y[i]) / (x[j] - x[i])); ww.append(w[i] * w[j])
    o = np.argsort(sl); c = np.cumsum(np.array(ww)[o])
    return float(np.array(sl)[o][np.searchsorted(c, c[-1] / 2)])


def stale():
    rng = np.random.default_rng(5)
    x = np.arange(9.0)
    y = 0.86 - 0.012 * x + 0.02 * rng.standard_normal(9)
    n = np.array([14, 9, 20, 12, 16, 11, 18, 10, 15.0])
    trust = np.array([0.9, 0.85, 0.95, 0.3, 0.9, 0.8, 0.9, 0.85, 0.9])
    y[3] = 0.42                                              # one bad day (rough water, low trust)
    w = n * trust
    b_ts = theil_sen(x, y, w)
    a_ts = np.median(y - b_ts * x)
    A = np.column_stack([np.ones_like(x), x])
    a_ls, b_ls = np.linalg.lstsq(A, y, rcond=None)[0]
    boots = []
    for _ in range(500):
        p = rng.integers(0, 9, 9)
        if len(set(p)) >= 3:
            boots.append(theil_sen(x[p], y[p], w[p]))
    hi = np.quantile(boots, 0.995)
    write("correction_stale.dat", "session  mean_score  weight", zip(x, y, w))
    write("correction_stale_lines.dat", "session  theil_sen  least_squares",
          [(xx, a_ts + b_ts * xx, a_ls + b_ls * xx) for xx in (0.0, 8.0)])
    print(f"stale: Theil-Sen slope {b_ts:+.4f}/session (99% upper {hi:+.4f}); OLS {b_ls:+.4f}")


def magnetics():
    """Compassmot fit for one thruster: dB_perp = k+- |u| per sign, heading error atan(dB/235 mG)."""
    rng = np.random.default_rng(9)
    k_plus, k_minus, sig = 34.0, 22.0, 2.0                      # mG per unit effort
    u = np.repeat([-0.75, -0.5, -0.25, 0.25, 0.5, 0.75], 4)
    dB = np.where(u > 0, k_plus * u, k_minus * u) + sig * rng.standard_normal(u.size)
    kp = (u[u > 0] @ dB[u > 0]) / (u[u > 0] @ u[u > 0])
    km = (u[u < 0] @ dB[u < 0]) / (u[u < 0] @ u[u < 0])
    write("correction_mag.dat", "effort  dB_perp_mG  (points)", zip(u, dB))
    print(f"mag: k+ = {kp:.1f} mG, k- = {km:.1f} mG; heading error at full effort "
          f"{math.degrees(math.atan(kp / 235)):.1f} deg / {math.degrees(math.atan(km / 235)):.1f} deg")
    return kp, km


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    ghost()
    stale()
    magnetics()
