"""Popups and prompts: the generic ones and perception, Nav2 and camera views."""

from rich.console import Group
from rich.text import Text

from mhseals_hardware.menu import Cancelled
from mhseals_testing.runner import fold
from mhseals_testing.ros import camera_feed
from mhseals_testing.ui import camera_view, imaging as pixels, perception, popups, theme
from mhseals_testing.ui.popups import hold
from mhseals_testing.ui.camera_view import CameraView


CAMERA_ROWS = 12         # cell rows of the perception popup's camera view


class Popups:
    """Popups and prompts, mixed into App."""

    def text_input(self, title, message):
        """A one-line text popup; the typed text on Enter, None on Esc."""
        modal = dict(kind='input', title=title, message=message, text='', width=64,
                     placeholder='add a note', hint=f'Enter save{theme.SEPARATOR}Esc keeps it bare')

        def handle(key):
            if key == 'enter':
                return modal['text'].strip()
            if key == 'escape':
                return ''
            modal['text'] = popups.edit(modal['text'], key)
        return self.popup(modal, handle) or None

    def perception_popup(self, view=False):
        """V: the perception view of ``self.facts`` (filled by the perception probe).

        ``v`` opens the camera view (its subscription exists only while it is open); ``view=True``
        opens it at once (ui.md keymap camera). Esc closes the view first, then the popup.
        Closing, resizing and detaching delete the kitty image.
        """
        modal = perception.modal(self.facts)
        state = dict(view=None, note='')
        idle_hint = modal['hint']

        def refresh():  # the probe fills facts while the popup is open; scroll is kept
            modal['rows'] = perception.modal(self.facts)['rows']
            current = state['view']
            if current is None:
                modal['hint'] = idle_hint
                if state['note']:
                    modal['body'] = Text(state['note'], style=theme.MUTED)
                return
            current.resize((min(self.console.size.width - 8, 96), CAMERA_ROWS))
            modal['hint'] = theme.SEPARATOR.join(('Esc close view', camera_view.KEYS_HINT))
            modal['body'] = Group(*current.lines(), Text(current.status(), style=theme.MUTED))

        def close_view():
            if state['view'] is not None:
                state['view'].close()
                state['view'] = None
                modal.pop('body', None)

        def open_view():
            feed = self.camera_feed()
            if feed is None:
                state['note'] = 'camera view needs a ROS node (none in this session)'
                self.notice = state['note']
                return
            state['note'] = ''
            node = getattr(getattr(self, 'ros', None), 'node', None)
            self.camera_boxes = None if node is None else camera_feed.node_observations(  # subscribed while `d` is on
                node, lambda label: camera_view.class_rgb(self.minimap.table, label))
            state['view'] = CameraView(
                feed, (1, CAMERA_ROWS), self.write_terminal,
                kitty=pixels.kitty_camera(),
                save_dir=self.paths.directory / 'snapshots' / 'camera', detections=self.camera_detections,
                observations=self.camera_boxes, work=camera_view.background)   # decode and encode off the keys

        def handle(key):
            current = state['view']
            if current is not None and current.key(key):  # raw: d and D differ
                return None
            key = fold(key)
            if self.scroll(modal, key, ('k', 'j')):
                return None
            if key == 'v':
                close_view() if current is not None else open_view()
            elif key in ('escape', 'q'):
                if current is not None and key == 'escape':
                    close_view()
                else:
                    return True
        try:
            if view:
                open_view()
                refresh()
            self.popup(modal, handle, refresh)
        finally:
            close_view()

    def camera_feed(self):
        """A CameraFeed on the ROS node, or None without one."""
        node = getattr(self.ros, 'node', None)
        return None if node is None else camera_feed.profile_feed(node, self.profile.sensors)

    def camera_detections(self, size, floor, at=None):
        """Observation boxes in camera pixels with class colours (perception-v2 7.3); floor boxes stay []."""
        boxes = getattr(self, 'camera_boxes', None)
        return [] if floor or boxes is None else boxes.rects(size, at)

    def write_terminal(self, text):
        """Kitty uploads and deletes go straight to the terminal, after Rich's refresh."""
        self.console.file.write(text)
        self.console.file.flush()

    def popup(self, modal, handle, refresh=None):
        return popups.popup(self, modal, handle, refresh)

    scroll = staticmethod(popups.scroll)

    def show(self, title, message=None, rows=None, width=60):
        return popups.show(self, title, message, rows, width)

    def live_nav2(self):
        """Return the live Nav2 applier, built when first needed (it subscribes to ROS state)."""
        if self._nav2_live is None and self.nav2_live_factory is not None:
            self._nav2_live = self.nav2_live_factory()
        return self._nav2_live

    def apply_nav2_live(self, requested):
        if self.watching or self.review:
            return {'refused': 'this instance only watches; the session owner applies'}
        return self.live_nav2().applier(requested)

    def nav2_popup(self):
        """Preview the Nav2 file against the running nodes; y applies it live, Esc leaves it."""
        source = self.nav2_file
        previewed = source.current  # apply exactly what is shown, even if the file changes meanwhile
        live = None if self.watching or self.review else self.live_nav2()
        if live is None:
            rows = [(key, f'{old if old is not None else theme.DASH} {theme.ARROW} '
                          f'{new if new is not None else theme.DASH}')
                    for key, old, new in source.changes()]
            refused = False
        else:
            try:
                found = live.preview(source.requested(previewed))
            except Exception as error:  # the page must survive a ROS failure
                self.notice = f'Nav2 preview failed: {error}'
                return
            rows = [(row['key'], f'{theme.DASH if row["old"] is None else row["old"]} '
                                 f'{theme.ARROW} {row["new"]}'
                                 + (' (derived)' if row['derived'] else '')) for row in found.rows]
            rows += [('refused', reason) for reason in found.refusals]
            refused = bool(found.refusals)
            if not rows:
                rows = [('', 'The running nodes already hold these values')]
        rows += [(key, f'removed from file {theme.ARROW} restart to reset') for key in source.removed()]
        rows = rows or [('', 'No changes pending')]
        label = 'Apply live' if live else 'Apply'
        options = ['Close'] if refused else [label, 'Close']
        try:   # Close is focused: Enter never applies, y does
            answer = self.choose('Nav2 parameters', options, {'y': 0} if len(options) > 1 else None,
                                 rows=rows, default=len(options) - 1, width=76,
                                 risky={0} if len(options) > 1 else set())
        except Cancelled:
            return
        if answer == label:
            self.notice = source.apply(previewed)

    def prompt(self, message, title='Ready'):
        # Owner decision 2026-10-26: a --sim-arm run whose checks passed (the desk sets agent_ready for it,
        # and only while it holds the arm) has its Ready prompts confirmed by the agent, journaled as such.
        who = getattr(self, 'agent_ready', None)
        if who and title == 'Ready' and self.armed:
            self.ros.emit('ready', by='agent', who=who, sim_arm=True, message=message)
            self.notice = f'Ready confirmed by agent {who} (sim auto-arm)'
            return ''
        return popups.prompt(self, message, title)

    def choose(self, title, options, shortcuts=None, message=None, danger=False, rows=None,
               default=0, width=64, risky=None, quiet=False):
        return popups.choose(self, title, options, shortcuts, message, danger, rows, default, width, risky, quiet)

    def ask_text(self, title, message, text='', check=None):
        return popups.ask_text(self, title, message, text, check)

    def type_word(self, title, message, word, rows=None, width=64, quiet=False):
        return popups.type_word(self, title, message, word, rows, width, quiet)

    def hold_confirm(self, title, verb, rows=None, message=None, key=None, width=64, quiet=False):
        """Hold-to-confirm (ui/popups/hold.py): True once Enter, ``key`` or a mouse button is held."""
        return hold.confirm(self, title, verb, rows, message, key, width, quiet)
