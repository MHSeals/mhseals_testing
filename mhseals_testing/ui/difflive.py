"""A full-screen Rich Live that sends only the screen rows that changed since the last frame.

Rich's Live rewrites every cell of the screen on each refresh: about 16-30 KiB a frame at 120x40, which
tmux re-parses and ssh carries to the station. DiffLive lays the frame out the same way (Rich's Screen:
the console's size, padded and cropped), compares each row's rendered text with what it sent last, and
writes only the changed rows, each behind a cursor move, in one write per frame. An unchanged frame writes
nothing. ``invalidate()`` (or the ``full_every`` period of config/perf.yaml) sends the whole screen again,
for anything that may have drawn over it outside the frame.

Outside a terminal, or not in screen mode, it is Rich's Live unchanged.
"""

import time

from rich.live import Live

from mhseals_testing.ui import perf

SYNC_BEGIN, SYNC_END = '\x1b[?2026h', '\x1b[?2026l'   # synchronized output: the terminal shows the frame whole


class DiffLive(Live):
    """Rich Live (screen=True, auto_refresh=False) writing changed rows only."""

    def __init__(self, *args, clock=time.monotonic, settings=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.clock = clock
        self.settings = settings or perf.settings()
        self.rows = []               # the rendered text of each screen row as last sent
        self.size = None
        self.full_at = 0.0           # when the whole screen was last sent
        self.sent = 0                # bytes written by the last refresh

    def invalidate(self):
        """Send the whole screen with the next frame (something else drew on it)."""
        self.rows = []

    def process_renderables(self, renderables):
        self.invalidate()            # a print through the console while live redraws the whole screen
        return super().process_renderables(renderables)

    def refresh(self):
        console = self.console
        if not (self._screen and console.is_terminal and not console.is_dumb_terminal) or getattr(self, '_nested', False):
            return super().refresh()
        with self._lock:
            width, height = console.size
            options = console.options.update_dimensions(width, height)
            lines = console.render_lines(self.renderable, options, pad=True)[:height]
            now = self.clock()
            full = (width, height) != self.size or not self.rows or (
                self.settings['full_every'] and now - self.full_at >= self.settings['full_every'])
            if full:
                self.rows, self.size, self.full_at = [None] * height, (width, height), now
            out = []
            render = console._render_buffer
            for y, line in enumerate(lines):
                text = render(line)
                if text != self.rows[y]:
                    self.rows[y] = text
                    out.append(f'\x1b[{y + 1};1H{text}')
            for y in range(len(lines), height):        # a short render: blank the rest once
                if self.rows[y] != '':
                    self.rows[y] = ''
                    out.append(f'\x1b[{y + 1};1H\x1b[0m\x1b[2K')
            self.sent = 0
            if not out:
                return
            data = ''.join(out)
            if self.settings['sync']:
                data = SYNC_BEGIN + data + SYNC_END
            console.file.write(data)          # one write per frame, through the console's file (sixel wraps it)
            console.file.flush()
            self.sent = len(data)


def live_class():
    """DiffLive, or Rich's Live when config/perf.yaml ``diff`` is off."""
    return DiffLive if perf.settings()['diff'] else Live
