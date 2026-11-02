"""Conditions feed: ``Conditions.summary()`` plus the heading, the health ``rose`` anchor (ui.md 6.3) and the nearest
point /perception/nearby saw (``nearby``, for the proximity glow)."""

import math
import time

from mhseals_testing.health.anchors import anchor_status

BAD_YAW = ('warn', 'fail')
NEARBY_STALE_S = 2.0      # /perception/nearby sends at up to 5 Hz, only for a new cloud


def compass(snapshot):
    """The boat's compass bearing in degrees from odometry yaw (ENU, counter-clockwise from east), or None."""
    yaw = snapshot.get('sensors', {}).get('odometry', {}).get('values', {}).get('yaw')
    return None if yaw is None else (90.0 - math.degrees(yaw)) % 360


def heading_sd(snapshot):
    """The odometry yaw sigma in degrees, or None while the covariance is missing, zero or not finite."""
    variance = snapshot.get('sensors', {}).get('odometry', {}).get('values', {}).get('yaw_var')
    if variance is None or not 0 < variance < math.pi ** 2:    # also False for NaN
        return None
    return math.degrees(math.sqrt(variance))


def yaw_state(engine):
    """``(yaw_ok, note)``: whether the ``rose`` anchor is clear, and the worst such check's short value."""
    if engine is None or engine.last is None:
        return True, ''
    status = anchor_status(engine.specs, engine.last, 'rose')
    worst = [engine.last[s.id] for s in engine.specs if 'rose' in s.show and s.id in engine.last
             and engine.last[s.id].status == status]
    return status not in BAD_YAW, worst[0].value if worst else ''


def nearby_point(ros, now=None):
    """``{range, bearing}`` of the nearest point /perception/nearby saw (bearing in degrees clockwise from the bow),
    for the proximity glow; None when it saw none or went quiet."""
    from mhseals_testing.ros import nearby
    at, found = getattr(ros, 'nearby', None) or (None, None)
    now = time.monotonic() if now is None else now
    near = nearby.nearest(found) if at is not None and now - at <= NEARBY_STALE_S else None
    return None if near is None else {'range': near[0], 'bearing': near[1]}


class ConditionsFeed:
    """Samples ``app.conditions`` and the engine each tick; ``view()['conditions']`` feeds the panel."""

    def __init__(self):
        self.info = None

    def tick(self, app, snapshot, now):
        yaw_ok, note = yaw_state(app.engine)
        self.info = {**app.conditions.summary(), 'heading': compass(snapshot), 'heading_sd': heading_sd(snapshot),
                     'yaw_ok': yaw_ok, 'yaw_note': note, 'nearby': nearby_point(getattr(app, 'ros', None))}

    def view(self):
        return {'conditions': self.info} if self.info else {}
