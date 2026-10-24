"""The perception popup's camera view: subscription lifetime, kitty delete, half-block golden, overlay cells."""
import io
import json
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
    decode = kwargs.pop('decode', lambda data: frame8())
    return camera_view.CameraView(feed, size, out.append, kitty=kitty, decode=decode,
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
    v, out = view(bus, kitty=True, decode=lambda data: np.kron(frame8(), np.ones((8, 8, 1), np.uint8)))
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


def test_freeze_holds_the_frame_save_writes_the_jpeg_and_detections_and_rate_steps(tmp_path):
    bus = Bus()
    v, _ = view(bus, save_dir=tmp_path / 'snapshots',
                detections=lambda size, floor: [] if floor else [((1, 2, 3, 4), (255, 0, 0))])
    v.clock = lambda: 1700000000.0
    bus.publish(b'JPEGDATA')
    held = ansi(v.lines())
    v.key('f')
    bus.t = 5.0
    bus.publish(b'OTHER')
    assert ansi(v.lines()) == held
    assert v.key('s') and v.notice.startswith('saved camera_') and v.notice.endswith('.jpg')
    name = v.notice.split()[1][:-4]
    assert (tmp_path / 'snapshots' / f'{name}.jpg').read_bytes() == b'JPEGDATA'   # the frame shown, original bytes
    saved = json.loads((tmp_path / 'snapshots' / f'{name}.json').read_text())
    assert saved['size'] == [8, 8] and saved['detections'] == {
        'boxes': [{'rect': [1, 2, 3, 4], 'rgb': [255, 0, 0]}], 'floor': []}
    v.key('s')
    assert len(list((tmp_path / 'snapshots').glob('*.jpg'))) == 2                # same second: suffix, no overwrite
    v.key('+')
    assert v.feed.hz == 2
    v.key('-')
    v.key('-')
    assert v.feed.hz == 1
    assert not v.key('x')


def test_the_pane_keeps_the_source_aspect_and_never_upscales():
    # 16:9 in a 96x12 pane: 43 cols of 8x16 px (half-block: 43 x 24 px), not 96
    assert camera_view.fit(96, 12, 1280, 720) == (43, 12)
    assert camera_view.fit(96, 12, 1280, 720, (1, 2)) == (43, 12)
    assert camera_view.fit(30, 12, 1280, 720) == (30, 12)           # the width limit still wins
    assert camera_view.fit(96, 12, 40, 32) == (5, 2)                # a tiny source is not enlarged
    bus = Bus()
    v, _ = view(bus, size=(20, 12))                                 # an 8x8 source in a 20x12 pane
    bus.publish()
    lines = v.lines()
    assert v.pane == (8, 4) and len(lines) == 12 and all(not l.plain for l in lines[4:])
    k, out = view(bus, size=(20, 12), kitty=True)
    bus.publish()
    k.lines()
    assert ',c=1,r=1' in out[0]                                      # 8 px wide: one cell


def test_a_bad_frame_becomes_a_notice_and_a_good_one_clears_it():
    bus = Bus()
    feed = CameraFeed(bus.subscribe, bus.unsubscribe, clock=lambda: bus.t)

    def decode(data):
        if data == b'bad':
            raise ValueError('not a jpeg')
        return frame8()

    def encode(image):
        raise OSError('disk')
    v = camera_view.CameraView(feed, (8, 4), lambda text: None, kitty=True, decode=decode, encode=encode)
    bus.publish(b'bad')
    assert 'waiting for a frame on ' + feed.topic in v.lines()[0].plain and v.notice == 'bad frame'
    v.lines()                                                        # not retried every refresh
    assert 'bad frame' in v.status()
    bus.t = 2.0
    bus.publish(b'good')
    assert len(v.lines()) == 4 and v.notice == 'image encode failed'  # encode failure: half-blocks, no raise


def test_status_line_shows_rate_age_and_frozen():
    bus = Bus()
    v, _ = view(bus)
    assert v.status() == '1 Hz'
    bus.publish()
    bus.t = 0.3
    v.lines()
    assert v.status() == '1 Hz · 0.3 s old'
    v.key('f')
    assert v.status().endswith('0.3 s old · frozen')


def test_ascii_or_no_colour_lists_detections_instead_of_half_blocks(monkeypatch):
    bus = Bus()
    v, _ = view(bus, size=(8, 4), detections=lambda size, floor: [((1, 2, 5, 6), (255, 0, 0))])
    bus.publish()
    for setup in (lambda: monkeypatch.setattr(camera_view.theme, 'ASCII', True),
                  lambda: (monkeypatch.setattr(camera_view.theme, 'ASCII', False),
                           monkeypatch.setenv('NO_COLOR', '1'))):
        setup()
        lines = v.lines()
        assert len(lines) == 4
        text = '\n'.join(line.plain for line in lines)
        assert '▀' not in text and camera_view.NEEDS_COLOUR in text and '1,2-5,6' not in text
        v.boxes = True
        assert '1,2-5,6' in '\n'.join(line.plain for line in v.lines())
        v.boxes = False


def test_png_uses_explicit_fast_octree_and_preserves_colours_and_thin_box(monkeypatch):
    from PIL import Image
    colours = np.array([(255, 0, 0), (0, 255, 0), (0, 0, 255),
                        (255, 255, 0), (0, 0, 0), (255, 255, 255)], dtype=np.uint8)
    image = np.repeat(np.repeat(colours[None], 12, axis=1), 32, axis=0)
    camera_view.draw_box(image, (3, 3, 10, 29))
    calls = []
    quantize = Image.Image.quantize

    def spy(self, colors=256, method=None, **kwargs):
        calls.append((colors, method))
        return quantize(self, colors, method=method, **kwargs)

    monkeypatch.setattr(Image.Image, 'quantize', spy)
    encoded = camera_view.encode_png(image)
    assert calls == [(64, Image.Quantize.FASTOCTREE)]
    decoded = Image.open(io.BytesIO(encoded))
    assert decoded.mode == 'P' and decoded.size == (72, 32)
    rgb = np.asarray(decoded.convert('RGB'))
    assert np.array_equal(rgb[16, np.arange(6) * 12 + 6], colours)
    assert np.array_equal(rgb, image)  # includes every one-pixel box edge


def test_real_camera_serializer_runs_once_per_fresh_frame_and_save_retains_source(monkeypatch, tmp_path):
    from PIL import Image
    bus, out, uploads, methods = Bus(), [], [], []
    source = io.BytesIO()
    Image.fromarray(np.tile(frame8(), (9, 16, 1))).save(source, 'JPEG')
    jpeg = source.getvalue()
    feed = CameraFeed(bus.subscribe, bus.unsubscribe, clock=lambda: bus.t)
    quantize = Image.Image.quantize

    def spy(self, colors=256, method=None, **kwargs):
        methods.append((colors, method))
        return quantize(self, colors, method=method, **kwargs)

    monkeypatch.setattr(Image.Image, 'quantize', spy)
    monkeypatch.setattr(graphics, 'upload', lambda data, cols, rows: uploads.append(data) or 'upload')
    monkeypatch.setattr(camera_view.CameraView, 'text_only', lambda self: False)
    v = camera_view.CameraView(feed, (8, 2), out.append, kitty=True, save_dir=tmp_path,
                              detections=lambda size, floor: [((16, 18, 32, 36), camera_view.BOX_RGB)])
    v.key('d')
    bus.publish(jpeg)
    v.lines()
    v.lines()
    assert methods == [(64, Image.Quantize.FASTOCTREE)] and len(uploads) == 1
    rgb = np.asarray(Image.open(io.BytesIO(uploads[0])).convert('RGB'))
    assert rgb.shape == (32, 56, 3)
    assert (rgb[8, 7:14] == camera_view.BOX_RGB).all()  # scaled source rect
    v.save()
    saved = next(tmp_path.glob('*.jpg'))
    assert saved.read_bytes() == jpeg and Image.open(saved).size == (128, 72)
    assert json.loads(saved.with_suffix('.json').read_text())['size'] == [128, 72]
    bus.t = 2
    bus.publish(jpeg)
    v.lines()
    assert methods == [(64, Image.Quantize.FASTOCTREE)] * 2 and len(uploads) == 2
    v.close()
