"""Pure admission evidence; receipts belong to independent observers, not polls."""
from dataclasses import dataclass
import math

from mhseals_testing.health.pwm import Readback


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class CommandObservation:
    receipt: float
    applied: tuple | None
    effort: tuple | None
    raw_active: bool | None
    epoch: object
    effort_receipt: float | None = None
    raw_receipt: float | None = None

    def zero(self, now, ttl, epoch):
        if (self.epoch != epoch or not finite(self.receipt)
                or not 0 <= now - self.receipt <= ttl):
            return None
        states = []
        for values, receipt in ((self.applied, self.receipt), (self.effort, self.effort_receipt)):
            if (not finite(receipt) or not 0 <= now - receipt <= ttl
                    or not isinstance(values, tuple) or len(values) != 3 or not all(map(finite, values))):
                states.append(None)
            else:
                states.append(all(v == 0 for v in values))
        states.append(None if (type(self.raw_active) is not bool or not finite(self.raw_receipt)
                               or not 0 <= now - self.raw_receipt <= ttl) else not self.raw_active)
        return False if False in states else (None if None in states else True)


@dataclass(frozen=True)
class EvidenceState:
    eligible: bool
    neutral: bool | None
    zero: bool | None
    cut: bool
    coast_started: float | None
    epoch: object


class EvidenceGate:
    """Unknown intervals close admission; proven zero starts a new coast.

    ``coast_started`` is a boundary for the estimator's coast policy, not proof
    that coast has already elapsed. Both command channels are required.
    """
    def __init__(self, *, output_ttl, command_ttl):
        if not all(finite(v) and v > 0 for v in (output_ttl, command_ttl)):
            raise ValueError('positive finite evidence TTLs required')
        self.output_ttl, self.command_ttl = output_ttl, command_ttl
        self._epoch = None
        self._now = None
        self._quiet = False
        self._eligible = False
        self._coast = None

    def update(self, *, afloat, bench, readback, command, local, raw_active, now, epoch):
        reset = epoch != self._epoch or not finite(now) or (self._now is not None and now < self._now)
        previous = self._eligible
        if reset:
            self._quiet = False
            self._coast = None
        self._epoch = epoch
        self._now = now if finite(now) else None
        neutral = zero = None
        if finite(now):
            if isinstance(readback, Readback):
                neutral = readback.neutral(now, self.output_ttl)
            if isinstance(command, CommandObservation):
                zero = command.zero(now, self.command_ttl, epoch)
        local_zero = (isinstance(local, tuple) and len(local) == 3
                      and all(map(finite, local)) and all(v == 0 for v in local))
        quiet = neutral is True and zero is True and local_zero and raw_active is False
        if quiet and not self._quiet:
            self._coast = now
        if not quiet:
            self._coast = None
        self._quiet = quiet
        self._eligible = quiet and afloat is True and bench is False
        return EvidenceState(self._eligible, neutral, zero,
                             reset or not self._eligible or previous != self._eligible,
                             self._coast, epoch)
