"""Battery facts from the flight controller's /mavros/battery (health.md section D, W1).

ROS-free and made of plain dicts. ``from_mavros`` turns a BatteryState-like object into a
sample (NaN becomes None, a source under 1 V is absent); ``Power`` folds samples into the
``power.*`` facts: cell count, per-cell minimum, sag against the resting baseline, mAh used
(reset on a pack swap), percentage and time left. Thresholds for the checks live in YAML (W2).
"""
from __future__ import annotations

import math
from statistics import median

# Volts per cell: full / nominal / low / critical.
CHEMISTRIES = {
    'lipo': {'full': 4.20, 'nominal': 3.70, 'low': 3.50, 'critical': 3.30},
    'liion': {'full': 4.20, 'nominal': 3.60, 'low': 3.30, 'critical': 3.00},
    'lifepo4': {'full': 3.65, 'nominal': 3.20, 'low': 3.00, 'critical': 2.80},
}
MIN_VOLT = 1.0               # below this the source reports no pack
REST_AMPS = 0.5              # at or under this the pack is resting
REST_WINDOW = 10.0           # s from the first sample that count for the baseline
SWAP_JUMP = 0.5              # V step at rest that means a different pack
DEFAULT_CAPACITY = 3300      # ArduPilot BATT_CAPACITY default: not a real capacity
MIN_CELL = 3.0               # lowest per-cell voltage accepted by cell detection
# Fraction of charge at (critical, low, nominal, full) for the voltage estimate.
PCT_POINTS = (0.0, 15.0, 50.0, 100.0)


def _num(value):
    """A finite float, else None."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def from_mavros(message, t, source='mavros') -> dict | None:
    """Sample ``{t, volt, amp, cells_v, mah_used, pct, temp, source}`` or None when absent."""
    volt = _num(getattr(message, 'voltage', None))
    if volt is None or volt < MIN_VOLT:
        return None
    cells = [_num(c) for c in (getattr(message, 'cell_voltage', None) or ())]
    cells = cells if cells and all(c is not None and c > 0 for c in cells) else None
    pct = _num(getattr(message, 'percentage', None))
    return {'t': t, 'volt': volt, 'amp': _num(getattr(message, 'current', None)),
            'cells_v': cells, 'mah_used': None,
            'pct': None if pct is None else min(max(pct, 0.0), 1.0) * 100.0,
            'temp': _num(getattr(message, 'temperature', None)), 'source': source}


def detect_cells(volt, chemistry='lipo') -> int | None:
    """The cell count whose per-cell voltage is in [3.0, full], nearest the nominal; or None."""
    c = CHEMISTRIES[chemistry]
    best = [n for n in range(1, 17) if MIN_CELL <= volt / n <= c['full']]
    return min(best, key=lambda n: abs(volt / n - c['nominal'])) if best else None


def estimate_pct(cell_volt, chemistry='lipo') -> float:
    """Percent charge from the per-cell voltage, piecewise linear between the table rows."""
    c = CHEMISTRIES[chemistry]
    xs = (c['critical'], c['low'], c['nominal'], c['full'])
    if cell_volt <= xs[0]:
        return 0.0
    for i in range(3):
        if cell_volt <= xs[i + 1]:
            f = (cell_volt - xs[i]) / (xs[i + 1] - xs[i])
            return PCT_POINTS[i] + f * (PCT_POINTS[i + 1] - PCT_POINTS[i])
    return 100.0


class Power:
    """Accumulates samples; ``update(sample)`` returns the current facts dict.

    ``battery`` is the profile's ``{chemistry, cells, capacity_mah}`` (all optional). A
    configured cell count is never overridden; a mismatch is reported as ``cells_match``.
    """

    def __init__(self, battery=None):
        b = battery or {}
        self.chemistry = b.get('chemistry') or 'lipo'
        if self.chemistry not in CHEMISTRIES:
            raise ValueError(f'battery.chemistry: one of {sorted(CHEMISTRIES)}')
        self.cells_cfg = b.get('cells')
        cap = b.get('capacity_mah')
        self.capacity = None if cap in (None, DEFAULT_CAPACITY) else cap
        self._reset()

    def _reset(self):
        self.t0 = self.last = None
        self.rest = []                 # volts while resting inside the window
        self.baseline = None           # resting per-cell volts once the window closed
        self.mah = 0.0
        self.detected = None

    def update(self, s) -> dict:
        """Fold one sample (from ``from_mavros``) and return the facts."""
        amp = s['amp']
        resting = amp is not None and abs(amp) <= REST_AMPS
        if self.last is not None and resting and abs(s['volt'] - self.last['volt']) > SWAP_JUMP:
            self._reset()                                    # pack swap
        if self.t0 is None:
            self.t0 = s['t']
        if s['mah_used'] is not None:
            self.mah = s['mah_used']
        elif self.last is not None and amp is not None and amp > 0:
            self.mah += amp * max(s['t'] - self.last['t'], 0.0) / 3.6
        if resting and s['t'] - self.t0 <= REST_WINDOW and self.baseline is None:
            self.rest.append(s['volt'])
            self.detected = detect_cells(median(self.rest), self.chemistry)
        self.last = s
        cells = self.cells_cfg or self.detected
        cell_v = s['cells_v']
        cell_min = min(cell_v) if cell_v else (s['volt'] / cells if cells else None)
        if self.baseline is None and self.rest and s['t'] - self.t0 > REST_WINDOW and cells:
            self.baseline = median(self.rest) / cells
        return self._facts(s, amp, cells, cell_min)

    def _facts(self, s, amp, cells, cell_min):
        sag = None
        if self.baseline is not None and cell_min is not None:
            sag = max(self.baseline - cell_min, 0.0) * 1000.0
        pct, estimated = s['pct'], False
        if cell_min is not None:
            guess = estimate_pct(cell_min, self.chemistry)
            if pct is None or guess < pct:
                pct, estimated = guess, amp is not None and amp > 2.0
        left = None
        if self.capacity and amp is not None and amp > REST_AMPS:
            left = max(self.capacity - self.mah, 0.0) / (amp * 1000.0) * 60.0
        return {'volt': s['volt'], 'amps': amp,
                'watts': None if amp is None else s['volt'] * amp,
                'cells': cells, 'cells_detected': self.detected,
                'cells_match': None if not (self.cells_cfg and self.detected)
                else self.cells_cfg == self.detected,
                'cell_min': cell_min, 'sag_mV': sag, 'mah_used': self.mah,
                'pct': pct, 'pct_estimated': estimated, 'remaining_min': left,
                'loaded': amp is not None and amp > 2.0, 'source': s['source']}
