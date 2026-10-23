"""Conditions feed: ``Conditions.summary()`` plus the heading and the health ``rose`` anchor (ui.md 6.3)."""

import math

from mhseals_testing.health.anchors import anchor_status

BAD_YAW = ('warn', 'fail')


def compass(snapshot):
    """The boat's compass bearing in degrees from odometry yaw (ENU, counter-clockwise from east), or None."""
    yaw = snapshot.get('sensors', {}).get('odometry', {}).get('values', {}).get('yaw')
    return None if yaw is None else (90.0 - math.degrees(yaw)) % 360


def yaw_state(engine):
    """``(yaw_ok, note)``: whether the ``rose`` anchor is clear, and the worst such check's short value."""
    if engine is None or engine.last is None:
        return True, ''
    status = anchor_status(engine.specs, engine.last, 'rose')
    worst = [engine.last[s.id] for s in engine.specs if 'rose' in s.show and s.id in engine.last
             and engine.last[s.id].status == status]
    return status not in BAD_YAW, worst[0].value if worst else ''


class ConditionsFeed:
    """Samples ``app.conditions`` and the engine each tick; ``view()['conditions']`` feeds the panel."""

    def __init__(self):
        self.info = None

    def tick(self, app, snapshot, now):
        yaw_ok, note = yaw_state(app.engine)
        self.info = {**app.conditions.summary(), 'heading': compass(snapshot), 'yaw_ok': yaw_ok, 'yaw_note': note}

    def view(self):
        return {'conditions': self.info} if self.info else {}
