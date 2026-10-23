"""The View section of the perception popup (perception.md section 8, ui.md popup.perception).

Pulls the newest frame from a ``CameraFeed``, scales it to the pane, draws detection boxes
(``d``; floor boxes with ``D``) and shows it as a kitty placeholder image or, anywhere else,
as half-blocks. ``f`` freezes, ``s`` saves a PNG, ``+``/``-`` change the rate. Closing, a
resize and the app's detach all call ``close``/``resize``, which delete the kitty image.

Decoding, PNG encoding and the output stream are injected, so tests need no JPEG or terminal.
"""

import io
from pathlib import Path

import numpy as np
from rich.text import Text

from mhseals_testing.ros.camera_feed import RATES_HZ
from mhseals_testing.ui import graphics, theme

CELL_PX = (8, 16)       # pixels per cell sent to kitty (width, height)
BOX_RGB = (255, 255, 0)
WAITING = 'waiting for a frame'


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
        self.boxes = self.floor = self.frozen = False
        self.notice = ''
        self._drawn = False         # a kitty image is on the terminal
        self._shown = None          # (received_at, size, flags) last uploaded
        self._frame = None          # (received_at, array)
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
        if latest is not None and (self._frame is None or latest[1] != self._frame[0]):
            self._frame = (latest[1], self.decode(latest[0]))
        return self._frame

    def _annotated(self, source):
        height, width = source.shape[:2]
        target_w, target_h = ((self.cols * CELL_PX[0], self.rows * CELL_PX[1]) if self.kitty
                              else (self.cols, self.rows * 2))
        image = scale(source, target_w, target_h).copy()
        if self.detections and (self.boxes or self.floor):
            for floor in (False, True):
                if (self.floor if floor else self.boxes):
                    for rect, rgb in self.detections((width, height), floor):
                        x0, y0, x1, y1 = (round(v * f) for v, f in zip(
                            rect, (target_w / width, target_h / height) * 2))
                        draw_box(image, (x0, y0, max(x1, x0 + 1), max(y1, y0 + 1)), rgb)
        return image

    def lines(self):
        """Rich Text lines for the pane (``rows`` of them); uploads the kitty frame when it changed."""
        frame = self._source()
        if frame is None:
            return [Text(WAITING if row == 0 else '', style=theme.MUTED) for row in range(self.rows)]
        image = self._annotated(frame[1])
        if not self.kitty:
            return graphics.half_blocks(image.tolist())
        stamp = (frame[0], self.cols, self.rows, self.boxes, self.floor)
        if stamp != self._shown:
            self.write(graphics.upload(self.encode(image), self.cols, self.rows))
            self._shown, self._drawn = stamp, True
        return graphics.placeholder_rows(self.cols, self.rows)

    def save(self):
        """Write the current picture (boxes included) as a PNG in the session; the notice names it."""
        frame = self._frame
        if frame is None or self.save_dir is None:
            return 'no frame to save'
        folder = Path(self.save_dir)
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'camera_{len(list(folder.glob("camera_*.png"))) + 1:03d}.png'
        path.write_bytes(self.encode(self._annotated(frame[1])))
        return f'saved {path.name}'

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
