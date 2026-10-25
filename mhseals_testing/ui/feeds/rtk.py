"""RTK feed: the base status, the rover's GPS_RAW fix and the ECEF baseline as an ``RtkView`` (gps.md 7).

Each live tick it builds ``self.current`` (None when the profile has no RTK base and no status topic
exists, so Status shows no row) and publishes ``rtk.seconds_below_fixed`` for config/health/rtk.yaml:
seconds the rover has been below FIXED while the base streams, absent otherwise (that check skips). It
also publishes rtk.correction_age and rtk.bytes_per_s for rtk.age / rtk.flow.
"""

import math

from mhseals_testing import rtk
from mhseals_testing.analysis import geo
from mhseals_testing.health.model import Fact
from mhseals_testing.ui.panels.status import sensor
from mhseals_testing.ui.rtk_row import OWN, STALE, RtkView

FIX_WORDS = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'FLOAT', 6: 'FIXED'}
CHECK = 'rtk.converge'
VIEW_FIELDS = ('base', 'drops', 'pubs', 'survey', 'survey_limit', 'survey_left', 'survey_acc', 'survey_target')


def rover(snapshot, specs):
    """(fix word, lat, lon, alt) of the live GPS raw sensor; ('none', None...) when silent."""
    row = sensor(snapshot, specs, 'gpsraw')
    if not row or row['status'] != 'LIVE':
        return 'none', None, None, None
    v = row['values']
    return FIX_WORDS.get(v.get('fix'), 'none'), v.get('lat'), v.get('lon'), v.get('alt')


def rtk_source():
    """rtk.yaml ``source`` (ours | external | none); ours when the file is unreadable."""
    try:
        return rtk.load_config()['source']
    except (OSError, ValueError, TypeError, KeyError):
        return 'ours'


def build(status, snapshot, specs, converge_warn=False, tick=0, source='ours'):
    """An RtkView from the status dict (``snapshot['rtk']``, may be None) and the snapshot's GPS raw."""
    status = status or {}
    fix, lat, lon, alt = rover(snapshot, specs)
    view = RtkView(fix=fix, converge_warn=converge_warn, tick=tick, source=source,
                   **{k: status[k] for k in VIEW_FIELDS if status.get(k) is not None})
    if source != 'ours' and view.base in OWN:
        view.pubs = max(view.pubs, 2)     # our base runs while another source is configured: two bases mix
    if status.get('age') is not None:
        view.age = status['age'] + status.get('received_age', 0.0)
    pos = status.get('base_position') or {}
    base = [pos.get(k) for k in ('latitude', 'longitude', 'height')]
    here = [lat, lon, alt]
    if all(isinstance(x, (int, float)) and math.isfinite(x) for x in base + here):
        view.baseline = geo.baseline(geo.ecef(*base), *here)
    return view


class RtkFeed:
    """Ticked with the App's snapshot; ``current`` is the RtkView or None."""

    def __init__(self, profile, source=None):
        self.profile, self.current, self.below_since = profile, None, None
        self.source = rtk_source() if source is None else source

    def tick(self, app, snapshot, now):
        status = snapshot.get('rtk')
        if status is None and not self.profile.uses_fcu:     # no base to show
            self.current, self.below_since = None, None
        else:
            last = getattr(getattr(app, 'engine', None), 'last', None) or {}
            result = last.get(CHECK)
            self.current = build(status, snapshot, self.profile.sensors, bool(result and result.status == 'warn'),
                                 int(now * 5), self.source)
            self._count(now)
        facts = {} if self.below_since is None else {'rtk.seconds_below_fixed': Fact(now - self.below_since)}
        facts.update(self._link_facts(status))
        app.health.facts.publish('rtk', facts)

    def _link_facts(self, status):
        """rtk.correction_age (s, shown age incl. transport delay) and rtk.bytes_per_s; absent without a base."""
        view, facts = self.current, {}
        if view is not None and view.age is not None:
            facts['rtk.correction_age'] = Fact(view.age)
        rate = ((status or {}).get('rates') or {}).get('bytes_per_s')
        if view is not None and view.base == 'streaming' and rate is not None:
            facts['rtk.bytes_per_s'] = Fact(rate)
        return facts

    def _count(self, now):
        """Start the below-FIXED clock while the link is fresh and the rover is not FIXED; stop it otherwise.

        With external corrections there is no link age to see, so the clock runs only in FLOAT:
        DGPS may be SBAS, and FLOAT proves carrier-phase corrections reach the rover.
        """
        view = self.current
        fresh = view.base == 'streaming' and view.age is not None and view.age <= STALE
        if view.source == 'external' and view.base not in OWN:
            fresh = view.fix == 'FLOAT'
        if fresh and view.fix != 'FIXED':
            self.below_since = now if self.below_since is None else self.below_since
        else:
            self.below_since = None

    def view(self):
        return {}
