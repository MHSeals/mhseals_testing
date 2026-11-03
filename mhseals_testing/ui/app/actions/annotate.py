"""Field annotation keys from any loop (the page, a run, manual): b marks, N bad data, " what the operator said.

Marks never block control and open nothing: b journals at once and offers the digit tags in the footer for
``pick`` seconds (config/annotate.yaml); Space and every other key keep their meaning meanwhile. The Labels
tab (actions/labels.py) reviews them afterwards.
"""

import math
import time

from mhseals_testing.annotate import config
from mhseals_testing.harness import journal
from mhseals_testing.ui import theme

FLASH = 3.0              # s a mark's footer flash stays (the pick's length by default)


def plain(key):
    """The key of a press (``press:b`` -> ``b``); None for releases and repeats (Kitty key events)."""
    if not isinstance(key, str):
        return None
    kind, _, name = key.rpartition(':') if ':' in key and len(key) > 2 else ('', '', key)
    return name if kind in ('', 'press') else None


class Annotate:
    """Mixed into App."""

    pick = None          # {'kind': 'tag'|'said', 'id', 'until'} while the digit pick is open
    flash = None         # (text, until): the footer's short confirmation
    bad_on = False

    # --- marks, anywhere -----------------------------------------------------------------------------------
    def annotate_key(self, key):
        """b, N, " and a pick's digit from any loop (the page, a run, manual). True when it was one of ours."""
        key = plain(key)
        try:
            if key == 'b':
                self.mark()
            elif key == 'N':
                self.toggle_bad()
            elif key == '"':
                self.say()
            elif key and key.isdigit() and self.pick_open():
                self.pick_digit(key)
            else:
                return False
        except (OSError, ValueError) as error:      # a full disk or a torn journal never stops the boat
            self.notice = f'Mark not saved: {error}'
        return True

    def mark(self):
        """b: a marker stamped now, with the digit tags offered for a moment."""
        number = journal.next_marker_id(journal.read(self.paths.journal))
        self.emit('marker', kind='user', id=number, note='', t_ns=time.time_ns())
        self.open_pick('tag', number)
        self.flash_text(f'{theme.pick("▾", "v")}{number} marked{theme.SEPARATOR}a digit tags it, optional')

    def toggle_bad(self):
        self.bad_on = not self.bad_on
        self.emit('bad_data', on=self.bad_on, who='operator')
        self.flash_text('bad data from here; N ends it' if self.bad_on else 'bad data ended')

    def say(self):
        """": what the operator just said, journaled now with the pose and scene; a digit picks the words."""
        number = 1 + max((e.get('id', 0) for e in journal.read(self.paths.journal) if e.get('phase') == 'said'), default=0)
        self.emit('said', id=number, text='', source='operator', pose=self.pose(), scene=self.scene())
        self.open_pick('said', number)
        self.flash_text(theme.pick('❝', '"') + f'{number} logged with the pose{theme.SEPARATOR}a digit picks the words')

    def open_pick(self, kind, number):
        self.pick = {'kind': kind, 'id': number, 'until': self.clock() + config.load()['pick']}

    def pick_open(self):
        if self.pick is not None and self.clock() > self.pick['until']:
            self.pick = None
        return self.pick is not None

    def pick_digit(self, digit):
        pick, self.pick = self.pick, None
        settings = config.load()
        if pick['kind'] == 'tag' and digit in settings['tags']:
            tag = settings['tags'][digit]['label']
            self.emit('marker_tag', id=pick['id'], tag=tag)
            self.flash_text(f'{theme.pick("▾", "v")}{pick["id"]} {tag}')
        elif pick['kind'] == 'said' and digit == '0':
            if self.moving() or self.armed:
                self.flash_text('typed in Labels later; the pose is kept')
            else:
                text = self.text_input('What was said', 'The instruction, in the operator\'s words')
                if text:
                    self.emit('said_text', id=pick['id'], text=text)
        elif pick['kind'] == 'said' and digit in settings['phrases']:
            self.emit('said_text', id=pick['id'], text=settings['phrases'][digit])
            self.flash_text(f'"{settings["phrases"][digit]}"')

    def pick_hints(self):
        """The footer's hints while a pick is open: the digits and what they choose."""
        if not self.pick_open():
            return None
        settings = config.load()
        if self.pick['kind'] == 'tag':
            return [(d, t['label'].replace('_', ' ')) for d, t in settings['tags'].items()]
        return [(d, p) for d, p in settings['phrases'].items()] + [('0', 'type it')]

    def flash_text(self, text):
        self.flash = (text, self.clock() + FLASH)

    def flashing(self):
        """The flash while it lasts; with a pick open it counts the seconds left to choose."""
        if self.flash is None or self.clock() > self.flash[1]:
            return None
        if self.pick_open():
            return f"{self.flash[0]}{theme.SEPARATOR}{math.ceil(self.pick['until'] - self.clock())} s"
        return self.flash[0]

    def pose(self):
        sensors = self.snapshot().get('sensors', {})
        odom = (sensors.get('odometry') or {}).get('values') or {}
        gps = (sensors.get('gpsraw') or {}).get('values') or {}
        return {k: v for k, v in (('x', odom.get('x')), ('y', odom.get('y')), ('yaw', odom.get('yaw')),
                                  ('vx', odom.get('vx')), ('vy', odom.get('vy')), ('wz', odom.get('wz')),
                                  ('lat', gps.get('lat')), ('lon', gps.get('lon'))) if v is not None}

    def scene(self):
        conditions = getattr(self, 'conditions_set', None)
        return {'running': self.running, 'run': self.run_id, 'armed': self.armed, 'manual': self.manual,
                'driving': list(self.driving() or ()) or None, 'conditions': conditions,
                'nearby': (getattr(self, 'feeds', None) and next((f.info.get('nearby') for f in self.feeds
                                                                    if getattr(f, 'info', None)), None))}
