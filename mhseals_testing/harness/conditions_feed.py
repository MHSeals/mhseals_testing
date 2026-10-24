"""Unwired Conditions source seam; never commands or mutates the estimator.

Call poll on every tick, with observed output/afloat/bench eligibility and command
state. Unknown eligibility is false. Downstream COND-NEUTRAL/EPOCH must close
eligibility even on None, expire on every tick, and clear estimator history when
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


class ConditionsFeed:
    def __init__(self, profile):
        self.epoch = 0
        self._now = None
        self._last = None
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

    @staticmethod
    def _fresh(row, spec):
        if not row or spec is None or row.get('status') != 'LIVE' or row.get('issue'):
            return False
        deadline = spec.deadline if spec.deadline is not None else 2.0
        age, stamp_age = row.get('age'), row.get('stamp_age')
        return (_finite(age) and 0 <= age <= deadline and
                (stamp_age is None or (_finite(stamp_age) and -.25 <= stamp_age <= deadline)))

    def poll(self, snapshot, now, *, eligible, commanding):
        """Consume a new source receipt once, even when invalid or ineligible.

        A count restart with advancing receipt starts an epoch; a swapped old row
        is ignored. A backwards polling clock starts an epoch independently.
        Callers must not replay this snapshot after repairing missing attitude.
        """
        if not _finite(now):
            return None
        if self._now is not None and now < self._now:
            self.epoch += 1
            self._last = None
        self._now = now
        rows = snapshot.get('sensors', {})
        row = rows.get(self.source.name) if self.source else None
        if not row:
            return None
        count, age = row.get('count'), row.get('age')
        if type(count) is not int or count < 1 or not _finite(age) or age < 0:
            return None
        received = now - age
        if not _finite(received):
            return None
        if self._last:
            old_count, old_time = self._last
            if count == old_count or received < old_time - 1e-9:
                return None
            if count < old_count:
                if received <= old_time + 1e-9:
                    return None
                self.epoch += 1
        self._last = count, received
        att = rows.get(self.attitude.name) if self.attitude else None
        if not self._fresh(row, self.source) or not self._fresh(att, self.attitude):
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
        stamp_age = row.get('stamp_age')
        stamp_time = None if stamp_age is None else now - stamp_age
        return SourceSample(received, count, stamp_time, (e, n), (math.pi / 2 - yaw) % (2 * math.pi),
                            rate, accuracy, eligible is True and commanding is False, self.epoch)
