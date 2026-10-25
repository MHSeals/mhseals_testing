"""The Layers box of the minimap (ui.md 5.6, 7 view.minimap): layer toggles and what they cost.

Pure and ROS-free: the ROS side is whatever ``feeds`` offers as ``want_layers`` (``MapFeeds``).
The costmap is 58-250 KB per message, so toggling `c` or `r` re-leases it for the view's
half-width, and with both off the subscription is dropped. The sim truth layer has no key: it
is listed when the profile has course truth.
"""

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap.state import FLOORS

SOURCES = ('auto', 'local', 'global')
OBJECTS = ('off', 'tracks', 'raw')      # `d` cycles; raw is tracks plus the raw detections
TRAILS = ('plain', 'gps', 'speed', 'track', 'off')
COLUMN = 14     # cells per column of the box


class LayerBox:
    """Layer toggles for one minimap view; ``press`` takes a ``view.minimap`` action name."""

    def __init__(self, feeds, state=None, truth=False, live=True):
        """``state``: a MapState or a callable returning one (or None).
        ``feeds`` None in review: the costmap there comes from BagSource, not a lease.
        ``live`` False refuses ``trail_restart`` (map.md 7: live only)."""
        self.feeds, self.state, self.truth, self.live = feeds, state, truth, live
        self.costmap = self.hits = self.referee = False
        self.wake = self.rings = self.tasks = True
        self.fov = None   # None until the map first draws: minimap.yaml fov.show seeds it
        self.up = 'N'     # N (north-up) or H (heading-up)
        self.source, self.objects, self.trail, self.floor = 'auto', 'tracks', 'plain', 0

    def _state(self):
        """The MapState, resolved at press time: ``state`` may be a callable (as MiniMap accepts)."""
        return self.state() if callable(self.state) else self.state

    def press(self, action, half_width):
        """Apply one action; returns whether it belonged to the box. ``half_width`` is the view's (m)."""
        if action == 'layer_costmap':
            self.costmap = not self.costmap
        elif action == 'layer_hits':
            self.hits = not self.hits
        elif action == 'costmap_source':
            self.source = SOURCES[(SOURCES.index(self.source) + 1) % len(SOURCES)]
        elif action == 'layer_detections':
            self.objects = OBJECTS[(OBJECTS.index(self.objects) + 1) % len(OBJECTS)]
        elif action == 'detection_floor':
            self.floor = (self.floor + 1) % len(FLOORS)
            if state := self._state():
                state.set_floor(FLOORS[self.floor])
        elif action == 'trail_mode':
            self.trail = TRAILS[(TRAILS.index(self.trail) + 1) % len(TRAILS)]
        elif action == 'trail_restart':
            if self.live and (state := self._state()):
                state.restart_trail()
        elif action == 'referee':
            self.referee = not self.referee
        elif action == 'layer_fov':
            self.fov = self.fov is False
        elif action == 'layer_wake':
            self.wake = not self.wake
        elif action == 'layer_rings':
            self.rings = not self.rings
        elif action == 'layer_tasks':
            self.tasks = not self.tasks
        elif action == 'up_mode':
            self.up = 'H' if self.up == 'N' else 'N'
        else:
            return False
        self.sync(half_width)
        return True

    def sync(self, half_width):
        """Lease the costmap for the view as it is now; call again when the zoom changes."""
        if self.feeds is not None:
            self.feeds.want_layers(self.costmap, self.hits, half_width, self.source)

    def rows(self):
        """Layer cells ``(key, label, on)`` in reading order; the pinned costmap source and the
        detection floor ride in the labels (ui.md 5.6), so the box stays six interior rows."""
        floor = f'{FLOORS[self.floor]:g}'.lstrip('0')
        objects = 'objects' if self.objects == 'off' else self.objects
        return [('c', 'costmap' if self.source == 'auto' else self.source, self.costmap),
                ('r', 'hits', self.hits),
                ('d', f'{objects} {floor}' if floor else objects, self.objects != 'off'),
                ('y', 'trail' if self.trail == 'off' else self.trail, self.trail != 'off'),
                ('R', 'referee', self.referee),
                ('F', 'fov', self.fov is not False),
                ('w', 'wake', self.wake),
                ('g', 'rings', self.rings),
                ('T', 'tasks', self.tasks)]

    def render(self, focused=False, up=None, zoom='fit', centre='boat'):
        """The Layers panel. On layers are fg, off ones muted, so the state reads without colour
        or glyphs. ``zoom`` and ``centre`` are the view's facts, which the box does not own."""
        def cell(key, label, on=True):
            out = Text(no_wrap=True)
            out.append(key, style=f'bold {theme.ACCENT}')
            out.append(f' {label}'.ljust(COLUMN - 1), style=None if on else theme.MUTED)
            return out

        cells = [cell(*row) for row in self.rows()]
        body = Text(no_wrap=True)
        for i in range(0, len(cells), 2):
            for part in cells[i:i + 2]:
                body.append_text(part)
            body.append('\n')
        for i, (key, label) in enumerate((('u', f'{up or self.up}{theme.pick("↑", "^")}'), ('z', zoom), ('e', centre))):
            body.append(key, style=f'bold {theme.ACCENT}')
            body.append(f' {label}' + ('  ' if i < 2 else ''))
        if self.truth:
            body.append('  [truth]', style=theme.MUTED)
        return theme.block(body, 'Layers', focused=focused)
