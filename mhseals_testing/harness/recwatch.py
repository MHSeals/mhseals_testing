"""Recording health: the ``bag.*`` and disk facts, the strip text and the disk auto-off.

``RecWatch`` is pure apart from two injected calls (``free`` for the disk, ``size`` for the
live segment), so tests drive a full disk by hand. The app calls ``sample`` about once a
second and acts on ``RecSample.stop_for_disk``: it stops the recorder, writes a system marker
and toasts. Recording is never turned back on automatically (bags.md section 2, disk safety).
"""

from dataclasses import dataclass
from pathlib import Path
import shutil

EMA_S = 30.0                    # time constant of the bag.rate_bps average
MIN_FREE_GB = 1.0               # recorder.min_free_gb in bags.md
DISK_MARKER = 'recording stopped: disk'


def disk_free_gb(path):
    """Free gigabytes on the filesystem holding ``path`` (or its nearest existing parent)."""
    path = Path(path)
    while not path.exists() and path != path.parent:
        path = path.parent
    return shutil.disk_usage(path).free / 1e9


def tree_bytes(path):
    """Bytes under ``path`` (0 when it does not exist yet)."""
    return sum(f.stat().st_size for f in Path(path).rglob('*') if f.is_file()) if Path(path).is_dir() else 0


def unfinalized(session):
    """Count of ``bag*`` segments in ``session`` that never got a metadata.yaml."""
    root = Path(session)
    return sum(1 for d in root.glob('bag*') if d.is_dir() and not (d / 'metadata.yaml').exists())


@dataclass(frozen=True)
class RecSample:
    mode: str                   # 'lean' or 'off'
    rate_bps: float             # EMA of the live segment's growth, 0 until two samples
    free_gb: float
    minutes_left: float | None  # None while the rate is unknown
    stop_for_disk: bool = False

    def facts(self, armed=False, unfinalized_count=0):
        """The numeric facts the health checks read, by name."""
        return {'bag.recording': self.mode, 'bag.rate_bps': self.rate_bps,
                'bag.off_armed': float(armed and self.mode == 'off'),
                'bag.unfinalized': float(unfinalized_count),
                'host.disk_free_gb': self.free_gb,
                'host.disk_minutes_left': self.minutes_left}


class RecWatch:
    def __init__(self, clock, free=disk_free_gb, size=tree_bytes, min_free_gb=MIN_FREE_GB):
        self.clock, self.free, self.size, self.min_free_gb = clock, free, size, min_free_gb
        self._last = None       # (time, bytes) of the previous sample
        self._rate = None
        self.tripped = False    # the disk stop fired; clears once recording is off

    def sample(self, segment, recording):
        """One reading. ``stop_for_disk`` is true once per trip, only while recording."""
        now, free = self.clock(), self.free(segment)
        if recording:
            size = self.size(segment)
            if self._last is not None and now > self._last[0]:
                rate = max(0.0, (size - self._last[1]) / (now - self._last[0]))
                weight = min(1.0, (now - self._last[0]) / EMA_S)
                self._rate = rate if self._rate is None else self._rate + weight * (rate - self._rate)
            self._last = (now, size)
        elif not recording:
            self._last, self._rate = None, None
            self.tripped = False
        stop = recording and free < self.min_free_gb and not self.tripped
        self.tripped = self.tripped or stop
        rate = self._rate or 0.0
        left = free * 1e9 / rate / 60 if rate > 0 else None
        return RecSample('lean' if recording else 'off', rate, free, left, stop)


def strip_text(sample, armed=False, sep=' · ', approx='≈'):
    """(state, text) for the header's rec chip. State is ok, warn, fail or off; no dots.

    ``rec lean ≈0.2 MB/s · 41 h`` when fine, ``rec off`` dim, and warn when armed.
    """
    if sample.mode == 'off':
        return ('warn' if armed else 'off'), 'rec off'
    text = f'rec {sample.mode}'
    if sample.rate_bps > 0:
        text += f' {approx}{sample.rate_bps / 1e6:.1f} MB/s'
        if sample.minutes_left is not None:
            text += f'{sep}{sample.minutes_left / 60:.0f} h'
    state = 'fail' if sample.free_gb < MIN_FREE_GB else 'warn' if sample.free_gb < 5 * MIN_FREE_GB else 'ok'
    return state, text
