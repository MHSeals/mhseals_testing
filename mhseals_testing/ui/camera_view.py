"""The View section of the perception popup (perception.md section 8, ui.md popup.perception).

Pulls the newest frame from a ``CameraFeed``, scales it to the pane, draws detection boxes
(``d``; floor boxes with ``D``) and shows it as a kitty placeholder image or, anywhere else,
as half-blocks. ``f`` freezes, ``s`` saves the original JPEG and its detections JSON, ``+``/``-`` change the rate.
The picture keeps the source aspect inside the pane and is never enlarged. Closing, a
resize and the app's detach all call ``close``/``resize``, which delete the kitty image.

Decoding, PNG encoding and the output stream are injected, so tests need no JPEG or terminal.
"""

import io
import json
import time
from pathlib import Path

import numpy as np
from rich.text import Text

from mhseals_testing.ros.camera_feed import RATES_HZ
from mhseals_testing.ui import graphics, palette, theme

CELL_PX = (8, 16)       # pixels per cell sent to kitty (width, height)
BOX_RGB = (255, 255, 0)
BAD_FRAME = 'bad frame'
WAITING = 'waiting for a frame'
NEEDS_COLOUR = 'camera view needs colour'
KEYS_HINT = 'd boxes  D floor  f freeze  s save  +- rate'


def fit(max_cols, max_rows, width, height, cell=CELL_PX):
    """(cols, rows) of the largest pane inside the maximum that keeps the source aspect.

    ``cell`` is the pixel size of one cell; the pane never needs more pixels than the source has.
    """
    rows = min(max_rows, max(1, height // cell[1]))
    cols = round(rows * cell[1] * width / (height * cell[0]))
    return max(1, min(max_cols, width // cell[0] or 1, cols)), rows


def decode_jpeg(data):
    """JPEG bytes -> HxWx3 uint8 array (Pillow is imported only when a frame arrives)."""
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(data)).convert('RGB'))


def encode_png(array):
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(array).save(out, 'PNG')
    return out.getvalue()


def scale(image, width, height):
    """Nearest-neighbour resize of an HxWx3 array to height x width (never reads outside it)."""
    ys = np.minimum(np.arange(height) * image.shape[0] // height, image.shape[0] - 1)
    xs = np.minimum(np.arange(width) * image.shape[1] // width, image.shape[1] - 1)
    return image[ys][:, xs]


def draw_box(image, rect, rgb=BOX_RGB):
    """One-pixel outline of an (x0, y0, x1, y1) rect, edges inclusive of x1-1 / y1-1."""
    x0, y0, x1, y1 = rect
    image[y0, x0:x1] = image[y1 - 1, x0:x1] = rgb
    image[y0:y1, x0] = image[y0:y1, x1 - 1] = rgb


class CameraView:
    """One open camera view. ``detections(size, floor)`` -> [(rect, rgb)] in source pixels."""

    def __init__(self, feed, size, write, kitty=False, save_dir=None, detections=None,
                 decode=decode_jpeg, encode=encode_png):
        self.feed, self.write, self.kitty = feed, write, kitty
        self.cols, self.rows = size
        self.save_dir, self.detections = save_dir, detections
        self.decode, self.encode = decode, encode
        self.clock = time.time
        self.boxes = self.floor = self.frozen = False
        self.notice = ''
        self._drawn = False         # a kitty image is on the terminal
        self._shown = None          # (received_at, size, flags) last uploaded
        self._frame = None          # (received_at, array, jpeg bytes)
        self._bad = None            # received_at of a frame that would not decode
        self.pane = (self.cols, self.rows)   # the size actually drawn (aspect-fitted)
        feed.open()

    # --- keys ---------------------------------------------------------------------

    def key(self, key):
        """Handle f s d D + -; True when the key was ours."""
        if key == 'd':
            self.boxes = not self.boxes
        elif key == 'D':
            self.floor = not self.floor
        elif key == 'f':
            self.frozen = not self.frozen
        elif key == 's':
            self.notice = self.save()
        elif key in ('+', '='):
            self._step(1)
        elif key in ('-', '_'):
            self._step(-1)
        else:
            return False
        return True

    def _step(self, direction):
        index = min(max(RATES_HZ.index(self.feed.hz) + direction, 0), len(RATES_HZ) - 1)
        self.feed.set_rate(RATES_HZ[index])

    # --- frames -------------------------------------------------------------------

    def _source(self):
        """The array to show: the newest frame, or the held one while frozen."""
        latest = None if self.frozen else self.feed.latest()
        if latest is not None and latest[1] != self._bad and (
                self._frame is None or latest[1] != self._frame[0]):
            try:
                self._frame = (latest[1], self.decode(latest[0]), latest[0])
                if self.notice == BAD_FRAME:
                    self.notice = ''
            except Exception:       # a corrupt frame must not end the TUI
                self._bad, self.notice = latest[1], BAD_FRAME
        return self._frame

    def status(self):
        """'1 Hz . 0.3 s old' (perception.md 7), plus 'frozen' and the last notice."""
        parts = [f'{self.feed.hz} Hz']
        if self._frame is not None:
            parts.append(f'{max(0.0, self.feed.now() - self._frame[0]):.1f} s old')
        if self.frozen:
            parts.append('frozen')
        if self.notice:
            parts.append(self.notice)
        return theme.SEPARATOR.join(parts)

    def _annotated(self, source, kitty):
        height, width = source.shape[:2]
        cell = CELL_PX if kitty else (1, 2)
        self.pane = fit(self.cols, self.rows, width, height, cell)
        target_w, target_h = self.pane[0] * cell[0], self.pane[1] * cell[1]
        image = scale(source, target_w, target_h).copy()
        if self.detections and (self.boxes or self.floor):
            for floor in (False, True):
                if (self.floor if floor else self.boxes):
                    for rect, rgb in self.detections((width, height), floor):
                        x0, y0, x1, y1 = (round(v * f) for v, f in zip(
                            rect, (target_w / width, target_h / height) * 2))
                        draw_box(image, (x0, y0, max(x1, x0 + 1), max(y1, y0 + 1)), rgb)
        return image

    def text_only(self):
        """True under ASCII or NO_COLOR: half-blocks would be wrong there (health.md fallback)."""
        return bool(theme.ASCII or palette.detect_tier() == 'mono')

    def _detection_list(self, frame):
        """The text fallback: a notice and one line per detection ('x0,y0-x1,y1 of WxH')."""
        height, width = frame[1].shape[:2]
        out = [NEEDS_COLOUR]
        if self.detections:
            for floor in (False, True):
                if self.floor if floor else self.boxes:
                    out += [('floor ' if floor else '') + '%d,%d-%d,%d' % tuple(round(v) for v in rect)
                            for rect, _ in self.detections((width, height), floor)]
        out = out[:self.rows]
        return [Text(line, style=theme.MUTED) for line in out] + [Text('') for _ in range(self.rows - len(out))]

    def lines(self):
        """Rich Text lines for the pane (``rows`` of them); uploads the kitty frame when it changed."""
        frame = self._source()
        if frame is None:
            return [Text(f'{WAITING} on {self.feed.topic}' if row == 0 else '', style=theme.MUTED)
                    for row in range(self.rows)]
        if self.text_only():
            return self._detection_list(frame)
        image = self._annotated(frame[1], self.kitty)
        cols, rows = self.pane
        blank = [Text('') for _ in range(self.rows - rows)]
        if not self.kitty:
            return graphics.half_blocks(image.tolist()) + blank
        stamp = (frame[0], self.cols, self.rows, self.boxes, self.floor)
        if stamp != self._shown:
            try:
                self.write(graphics.upload(self.encode(image), cols, rows))
            except Exception:
                self.notice = 'image encode failed'
                image = self._annotated(frame[1], False)
                return graphics.half_blocks(image.tolist()) + [Text('') for _ in range(self.rows - self.pane[1])]
            self._shown, self._drawn = stamp, True
        return graphics.placeholder_rows(cols, rows) + blank

    def save(self):
        """Write the original JPEG and the matching detections JSON; the notice names the JPEG."""
        frame = self._frame
        if frame is None or self.save_dir is None:
            return 'no frame to save'
        folder = Path(self.save_dir)
        folder.mkdir(parents=True, exist_ok=True)
        height, width = frame[1].shape[:2]
        found = {'floor' if floor else 'boxes': [{'rect': list(rect), 'rgb': list(rgb)}
                                                 for rect, rgb in self.detections((width, height), floor)]
                 for floor in (False, True)} if self.detections else {}
        stem = time.strftime('camera_%Y%m%d_%H%M%S', time.localtime(self.clock()))
        base, count = stem, 1
        while (folder / f'{base}.jpg').exists():
            count += 1
            base = f'{stem}_{count:02d}'
        (folder / f'{base}.jpg').write_bytes(frame[2])
        (folder / f'{base}.json').write_text(json.dumps(
            {'time': self.clock(), 'size': [width, height], 'detections': found}))
        return f'saved {base}.jpg'

    # --- lifetime -----------------------------------------------------------------

    def resize(self, size):
        """A new pane size: delete the image so it is re-sent at the new cell count."""
        if tuple(size) != (self.cols, self.rows):
            self._delete()
            self.cols, self.rows = size

    def _delete(self):
        if self._drawn:
            self.write(graphics.delete())
        self._drawn, self._shown = False, None

    def close(self):
        """Release the subscription and remove the image (view close, popup close, detach)."""
        self.feed.close()
        self._delete()
