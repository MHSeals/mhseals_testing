"""The perception popup's camera view: subscription lifetime, kitty delete, half-block golden, overlay cells."""
import io
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from rich.console import Console

from mhseals_testing.ros.camera_feed import CameraFeed
from mhseals_testing.ui import app, camera_view, graphics

GOLDEN = Path(__file__).parent / 'golden' / 'camera_halfblocks_8x8.txt'


class Bus:
    def __init__(self):
        self.live, self.t = {}, 0.0

    def subscribe(self, topic, callback):
        handle = object()
        self.live[handle] = callback
        return handle

    def unsubscribe(self, handle):
        del self.live[handle]

    def publish(self, data=b'x'):
        for callback in list(self.live.values()):
            callback(SimpleNamespace(data=data))


def frame8():
    """A fixed 8x8 RGB gradient (the 'decoded JPEG')."""
    y, x = np.mgrid[0:8, 0:8]
    return np.stack([x * 36, y * 36, (x + y) * 18], axis=-1).astype(np.uint8)


def view(bus, size=(8, 4), kitty=False, out=None, **kwargs):
    feed = CameraFeed(bus.subscribe, bus.unsubscribe, clock=lambda: bus.t)
    out = [] if out is None else out
    return camera_view.CameraView(feed, size, out.append, kitty=kitty, decode=lambda data: frame8(),
                                  encode=lambda image: b'png', **kwargs), out


def ansi(lines):
    console = Console(color_system='256', force_terminal=True, file=io.StringIO(), width=20)
    for line in lines:
        console.print(line)
    return console.file.getvalue()


def test_half_block_fallback_matches_golden_from_a_fixed_8x8_frame():
    bus = Bus()
    v, out = view(bus)
    bus.publish()
    text = ansi(v.lines())
    if not GOLDEN.exists():
        GOLDEN.write_text(text)
    assert text == GOLDEN.read_text() and out == []        # no kitty bytes on the fallback


def test_closing_releases_the_subscription_and_deletes_the_kitty_image():
    bus = Bus()
    v, out = view(bus, kitty=True)
    assert len(bus.live) == 1
    bus.publish()
    rows = v.lines()
    assert out[0].startswith('\x1b_Ga=T,f=100,U=1,i=31,c=8,r=4') and len(rows) == 4
    v.lines()
    assert len(out) == 1                                    # the same frame is not re-sent
    v.close()
    assert not bus.live and out[-1] == graphics.delete()
    v.close()
    assert out.count(graphics.delete()) == 1


def test_resize_deletes_the_image_and_a_never_drawn_view_deletes_nothing():
    bus = Bus()
    v, out = view(bus, kitty=True)
    v.resize((6, 3))
    assert out == []
    bus.publish()
    v.lines()
    v.resize((6, 3))                                        # same size: keep it
    v.resize((10, 5))
    assert out[-1] == graphics.delete() and len(v.lines()) == 5


def test_boxes_land_on_the_expected_cells_and_d_toggles_them():
    bus = Bus()
    # 8x8 source on an 8x4-cell (8x8 px) pane: a source rect (2, 2, 6, 6) is cells x 2..5, rows 1..2
    v, _ = view(bus, detections=lambda size, floor: [((2, 2, 6, 6), (255, 255, 0))])
    bus.publish()
    plain = ansi(v.lines())
    assert v.key('d') and v.boxes
    boxed = v.lines()
    assert ansi(boxed) != plain

    def colour(line, column):                                 # (top, bottom) of the cell
        spans = [s for s in line.spans]
        style = next(s.style for s in spans if s.start <= column < s.end)
        return style.color.number, style.bgcolor.number
    yellow = 226
    assert colour(boxed[1], 2)[0] == yellow and colour(boxed[1], 5)[0] == yellow   # top edge, pixel row 2
    assert colour(boxed[1], 3)[1] != yellow and colour(boxed[2], 3)[1] == yellow   # bottom edge, pixel row 5
    assert colour(boxed[0], 3) == colour(v.lines()[0], 3)
    v.key('d')
    assert ansi(v.lines()) == plain


def test_freeze_holds_the_frame_save_writes_a_png_and_rate_steps(tmp_path):
    bus = Bus()
    v, _ = view(bus, save_dir=tmp_path / 'camera')
    bus.publish()
    held = ansi(v.lines())
    v.key('f')
    bus.t = 5.0
    bus.publish()
    assert ansi(v.lines()) == held
    assert v.key('s') and v.notice == 'saved camera_001.png'
    assert (tmp_path / 'camera' / 'camera_001.png').read_bytes() == b'png'
    v.key('+')
    assert v.feed.hz == 2
    v.key('-')
    v.key('-')
    assert v.feed.hz == 1
    assert not v.key('x')
