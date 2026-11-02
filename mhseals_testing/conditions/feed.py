"""Unwired Conditions source seam; never commands or mutates the estimator.

Call poll on every tick and inspect status independently of its optional sample.
Unknown eligibility is false. Downstream COND-NEUTRAL/EPOCH must close
eligibility on invalid status, expire on every tick, and clear estimator history when
``epoch`` changes (including ticks returning None). No production caller yet.
"""
from dataclasses import dataclass
import math


def _finite(value):
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class SourceSample:
    time: float                 # source receipt, never UI polling time
    count: int                  # source message count within this epoch
    stamp_time: float | None     # message stamp mapped onto the receipt clock, when known
    velocity: tuple             # ENU m/s
    heading: float              # compass radians
    rate: float                 # source attitude wz, rad/s
    accuracy: float             # monitor vel_acc in m/s; zero only for explicit sim odom
    eligible: bool
    epoch: int


@dataclass(frozen=True)
class PollStatus:
    valid: bool
    epoch: int
    reason: str


class ConditionsFeed:
    def __init__(self, profile):
        self.epoch = 0
        self._now = None
        self._last = None
        self._clock_epoch = None
        self._accepted = False
        self.configure(profile)

    def configure(self, profile):
        """Switch profiles explicitly; any change starts a new estimator epoch."""
        choice = profile.conditions_velocity
        kind = 'gpsraw' if choice == 'gpsraw' else 'odometry'
        sources = [s for s in profile.sensors if s.kind == kind and
                   (kind == 'gpsraw' or choice[5:] in (s.name, s.topic))]
        self.source = sources[0] if len(sources) == 1 else None
        if kind == 'odometry' and not profile.use_sim_time:
            self.source = None
        attitudes = [s for s in profile.sensors if s.kind == 'odometry' and s.topic == profile.odometry]
        self.attitude = self.source if kind == 'odometry' else (attitudes[0] if len(attitudes) == 1 else None)
        self.kind = kind
        self.epoch += 1
        self._last = None
        self._accepted = False
        self.status = PollStatus(False, self.epoch, 'profile changed')

    @staticmethod
    def _fresh(row, spec, captured, now):
        if not row or spec is None or row.get('status') != 'LIVE' or row.get('issue'):
            return False
        deadline = spec.deadline if spec.deadline is not None else 2.0
        received, stamp_age = row.get('received'), row.get('stamp_age')
        if not _finite(received):
            return False
        age = now - received
        if stamp_age is not None:
            stamp_age = stamp_age + now - captured if _finite(stamp_age) else math.nan
        return (_finite(age) and 0 <= age <= deadline and
                (stamp_age is None or (_finite(stamp_age) and -.25 <= stamp_age <= deadline)))

    def poll(self, snapshot, now, *, eligible, commanding, clock_epoch=None):
        """Consume a new source receipt once, even when invalid or ineligible.

        Snapshots require immutable row.received and plots.now capture metadata.
        A count restart with advancing receipt starts an epoch; a swapped old row
        is ignored. A backwards polling clock starts an epoch independently.
        Callers must not replay this snapshot after repairing missing attitude.
        """
        # Explicit replay/simulation epochs are independent of message headers.
        if clock_epoch is not None and clock_epoch != self._clock_epoch:
            self.epoch += 1
            self._last = None
            self._accepted = False
            self._clock_epoch = clock_epoch
        self.status = PollStatus(False, self.epoch, 'invalid clock')
        if not _finite(now):
            return None
        if self._now is not None and now < self._now:
            self.epoch += 1
            self._last = None
        self._now = now
        self.status = PollStatus(False, self.epoch, 'missing or invalid source')
        captured = snapshot.get('plots', {}).get('now')
        if not _finite(captured) or captured > now:
            return None  # capture time must share the polling/monitor clock
        rows = snapshot.get('sensors', {})
        row = rows.get(self.source.name) if self.source else None
        if not row:
            return None
        count, received = row.get('count'), row.get('received')
        if (type(count) is not int or count < 1 or not _finite(received)
                or received > captured):
            return None
        duplicate = False
        if self._last:
            old_count, old_time = self._last
            if received < old_time or (count < old_count and received == old_time):
                self.status = PollStatus(False, self.epoch, 'reordered source')
                return None
            duplicate = (count, received) == self._last
            if count <= old_count and not duplicate:
                self.epoch += 1  # equal count with a new receipt also denotes restart
        if not duplicate:
            self._accepted = False
        self._last = count, received
        self.status = PollStatus(False, self.epoch, 'invalid source or attitude')
        att = rows.get(self.attitude.name) if self.attitude else None
        if not (self._fresh(row, self.source, captured, now)
                and self._fresh(att, self.attitude, captured, now)):
            return None
        values, angles = row.get('values', {}), att.get('values', {})
        yaw, rate = angles.get('yaw'), angles.get('wz')
        if self.kind == 'gpsraw':
            e, n, accuracy, fix = (values.get(k) for k in ('ve', 'vn', 'vel_acc', 'fix'))
            if not _finite(fix) or fix < 3 or not _finite(accuracy) or accuracy <= 0:
                return None
        else:
            vx, vy = values.get('vx'), values.get('vy')
            if not all(_finite(v) for v in (vx, vy, yaw)):
                return None
            c, s = math.cos(yaw), math.sin(yaw)
            e, n, accuracy = vx * c - vy * s, vx * s + vy * c, 0.0
        if not all(_finite(v) for v in (e, n, yaw, rate)):
            return None
        # An invalid consumed receipt cannot be repaired into a new sample.
        self.status = PollStatus(not duplicate or self._accepted, self.epoch,
                                 'fresh' if not duplicate or self._accepted else 'consumed invalid receipt')
        if duplicate:
            return None
        self._accepted = True
        stamp_age = row.get('stamp_age')
        stamp_time = None if stamp_age is None else captured - stamp_age
        return SourceSample(received, count, stamp_time, (e, n), (math.pi / 2 - yaw) % (2 * math.pi),
                            rate, accuracy, eligible is True and commanding is False, self.epoch)
