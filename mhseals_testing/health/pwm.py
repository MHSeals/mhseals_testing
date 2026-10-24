"""Actual-output evidence shared by health and Conditions.

Receipts are monotonic acquisition times, never projection/poll times. Sysfs
records describe kernel settings, not measured voltage or propeller motion.
"""

from dataclasses import dataclass
from math import isfinite

from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM


@dataclass(frozen=True)
class Output:
    identity: str
    state: str  # enabled, disabled, unexported, unavailable, invalid
    period_ns: int | None = None
    duty_ns: int | None = None
    issue: str = ''

    @property
    def valid(self):
        if self.state == 'unexported':
            return self.period_ns is None and self.duty_ns is None
        if self.state not in ('enabled', 'disabled') or self.issue:
            return False
        period, duty = self.period_ns, self.duty_ns
        if type(period) is not int or type(duty) is not int:
            return False
        if period <= 0 or not 0 <= duty <= period:
            return False
        return self.state == 'disabled' or MIN_PWM * 1000 <= duty <= MAX_PWM * 1000

    @property
    def neutral(self):
        return self.valid and (self.state != 'enabled' or self.duty_ns == NEUTRAL_PWM * 1000)

    def row(self):
        if self.state == 'unexported':
            return 'OFF (unexported)'
        if self.state == 'unavailable':
            return 'unavailable'
        if self.period_ns is None or self.duty_ns is None:
            return 'invalid period/duty'
        prefix = 'ON' if self.state == 'enabled' else 'OFF'
        if self.period_ns <= 0:
            return f'{prefix} period unset'
        return f'{prefix} {self.duty_ns / 1000:g}us / {1e9 / self.period_ns:g}Hz'


@dataclass(frozen=True)
class Readback:
    backend: str
    receipt: float | None
    outputs: tuple[Output, ...] = ()
    verified: bool = False
    configuration_valid: bool = True

    def fresh(self, now, ttl):
        return (type(self.receipt) in (int, float) and isfinite(self.receipt)
                and type(now) in (int, float) and isfinite(now)
                and type(ttl) in (int, float) and isfinite(ttl)
                and 0 <= now - self.receipt <= ttl)

    def known(self, now, ttl):
        identities = [output.identity for output in self.outputs]
        return (self.verified and self.backend in ('odroid', 'topic')
                and self.configuration_valid and self.fresh(now, ttl)
                and len(identities) == 4 and all(identities)
                and all(isinstance(identity, str) for identity in identities)
                and len(set(identities)) == 4 and all(output.valid for output in self.outputs))

    def neutral(self, now, ttl):
        """True/False when all four outputs are known, otherwise None."""
        return all(output.neutral for output in self.outputs) if self.known(now, ttl) else None

    def health(self, now, ttl, armed=False):
        """Pure health projection using the same expiry/validation as Conditions."""
        if not self.known(now, ttl):
            return 'N/A', 'actual outputs unknown'
        enabled = sum(output.state == 'enabled' for output in self.outputs)
        level = ('FAIL' if armed and enabled != 4 else
                 'WARN' if not armed and enabled else 'OK')
        return level, f'{enabled}/4 ON; {sum(not o.neutral for o in self.outputs)} non-neutral'
