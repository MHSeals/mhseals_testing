"""The image-tier prototype (ui/pixels.py): a live demo, a cost bench and widget PNGs.

    python3 tools/graphics_proto.py demo [--tier auto|kitty|sixel|text] [--seconds 20]
    python3 tools/graphics_proto.py bench [--cell 10x20] [--updates 60]
    python3 tools/graphics_proto.py png OUT_DIR [--cell 10x20]

``demo`` draws the rose, a Motion plot, the minimap, the inspect dial and a spark in the current
terminal through the same hooks as App.refresh (probe, before/frame/after), turning the boat, with
a popup over the rose every few seconds. ``bench`` prints per widget and tier the CPU and bytes per
update. Run from the package root; the minimap scene comes from test/test_minimap_frame.py.
"""

import argparse
import io
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / 'test')]

from rich.console import Console, Group  # noqa: E402
from rich.layout import Layout  # noqa: E402
from rich.panel import Panel  # noqa: E402
from rich.text import Text  # noqa: E402

from mhseals_testing.ui import braille, pixels, sparks, theme  # noqa: E402
from mhseals_testing.ui.panels import conditions  # noqa: E402

HUE = '#E9F3F6'


def scene_state():
    from test_minimap_frame import TABLE, box, scene
    return TABLE, box, scene()


def rose(i):
    heading = (30 + 3 * i) % 360
    command = (0.6 * math.cos(i / 9), 0.3 * math.sin(i / 7), 0.4 * math.sin(i / 11))
    return conditions.rose((0.05, 0.08), heading, True, 0.0, command, 1.0, theme.ACCENT)


def plot(i, width=40, rows=4):
    now = i / 30
    canvas = braille.canvas(width, rows, name='bench.plot')
    canvas.hline(0.0, -1, 1)
    canvas.line([(now - 10 + k / 30, 0.8 * math.sin((now - 10 + k / 30) * 1.3)) for k in range(300)],
                now - 10, now, -1, 1, 'pred')
    canvas.line([(now - 10 + k / 30, 0.7 * math.sin((now - 10 + k / 30) * 1.3 - 0.4) + 0.05 * math.sin(k))
                 for k in range(300)], now - 10, now, -1, 1, 'meas')
    return canvas.rows()


def spark(i):
    values = [math.sin((i + k) / 6) * 0.8 + 0.1 * math.sin(k * 1.7) for k in range(120)]
    return sparks.spark(values, 30, -1, 1, band=(-0.5, 0.5), height=2)


def dial(i):
    from mhseals_testing.ui.popups import inspect
    return inspect.dial([((20 + 2 * i) % 360, theme.ACCENT, False), ((35 + 2 * i) % 360, 'bold', False),
                         ((60 + i) % 360, theme.MUTED, True)], 21, 9)


class Map:
    def __init__(self, cols=60, rows=20):
        from mhseals_testing.ui.minimap import panel
        self.panel, self.cols, self.rows = panel, cols, rows
        self.table, self.box, self.state = scene_state()

    def __call__(self, i):
        from mhseals_testing.ui.minimap.emit import emit, emit_image
        t, x, y, yaw = self.state.pose
        self.state.ingest('pose_map', t=t + 0.2, x=x + 0.3 * math.cos(yaw), y=y + 0.3 * math.sin(yaw),
                          yaw=yaw + math.radians(2))
        canvas = self.panel.draw(self.state, self.cols, self.rows, 10.0, self.table, theme.ASCII,
                                 self.box(self.state), True, 'truecolor')
        look = lambda layer: self.panel.style(layer, 'truecolor')
        lines = emit_image(canvas, look) if pixels.active() else emit(canvas, look, 'truecolor')
        return [Text.from_ansi(line) for line in lines]


WIDGETS = {'rose': rose, 'plot': plot, 'spark': spark, 'dial': dial}


# --- bench ---------------------------------------------------------------------------------------

def measure(tier, make, updates, cell):
    """``(cold bytes, mean frame text bytes, mean image bytes per update, mean ms per update)``."""
    pixels.use(tier, cell)
    braille.HZ = None                       # every call is an update here
    console = Console(file=io.StringIO(), width=200, height=60, force_terminal=True,
                      color_system='truecolor', legacy_windows=False)
    stream = pixels._Stream(io.StringIO())
    pixels._stream = stream if tier == 'sixel' else None
    text_bytes, image_bytes, times, cold = [], [], [], 0
    for i in range(updates + 1):
        began = time.perf_counter()
        rows = make(i)
        segments = pixels.frame(list(console.render(Group(*rows))), (200, 60))
        out = []
        pixels.before(out.append)
        text = console._render_buffer(segments)
        if tier == 'sixel':
            text = pixels._MARKS.sub(stream._swap, text)
        pixels.after(out.append)
        elapsed = (time.perf_counter() - began) * 1000
        image = sum(len(s.encode()) for s in out)
        if i == 0:
            cold = image
            continue
        times.append(elapsed)
        text_bytes.append(len(text.encode()))
        image_bytes.append(image)
    braille.HZ = 5.0
    mean = lambda xs: sum(xs) / len(xs)
    return cold, mean(text_bytes), mean(image_bytes), mean(times), pixels.STATS['tiles']


def bench(args):
    cell = tuple(int(v) for v in args.cell.split('x'))
    widgets = dict(WIDGETS, minimap=Map())
    print(f'cell {cell[0]}x{cell[1]} px, {args.updates} updates each; bytes are UTF-8 on the wire')
    print(f'{"widget":8} {"tier":6} {"cold B":>8} {"text B/frame":>13} {"image B/update":>15} '
          f'{"ms/update":>10} {"tiles":>6}')
    for name, make in widgets.items():
        for tier in ('text', 'kitty', 'sixel'):
            if name == 'minimap':
                make = Map()
            cold, text, image, ms, tiles = measure(tier, make, args.updates, cell)
            print(f'{name:8} {tier:6} {cold:8.0f} {text:13.0f} {image:15.0f} {ms:10.2f} {tiles:6d}')


# --- png -----------------------------------------------------------------------------------------

def png(args):
    """Each widget's raster as a PNG (on the theme bg), for the gallery."""
    import zlib
    import struct
    import numpy as np
    cell = tuple(int(v) for v in args.cell.split('x'))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    pixels.use('kitty', cell)
    rasters = {}
    real = pixels.surface

    def spy(name, paint, cols, rows, **kw):
        raster = paint if isinstance(paint, pixels.Raster) else pixels.Raster(cols, rows)
        if not isinstance(paint, pixels.Raster):
            paint(raster)
        rasters[name] = raster
        return real(name, raster, cols, rows, holes=kw.get('holes', ()), tile=kw.get('tile', pixels.TILE))
    pixels.surface = spy
    for name, make in dict(WIDGETS, minimap=Map()).items():
        make(5)
    pixels.surface = real
    bg = np.array(pixels.token('bg'), np.uint8)
    for name, raster in rasters.items():
        rgba = raster.rgba()
        rgb = np.where(rgba[..., 3:] > 0, rgba[..., :3], bg)
        raw = b''.join(b'\0' + row.tobytes() for row in rgb)
        chunk = lambda tag, data: (struct.pack('>I', len(data)) + tag + data
                                   + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))
        data = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', raster.w, raster.h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
        (out / f'{name}.png').write_bytes(data)
        print(out / f'{name}.png', raster.w, 'x', raster.h)


# --- demo ----------------------------------------------------------------------------------------

def demo(args):
    from rich.live import Live
    from mhseals_hardware.keyboard import KeyReader
    from mhseals_testing.ui.layout import Overlay
    console = Console()
    writer = lambda text: (console.file.write(text), console.file.flush())
    minimap = Map(56, 18)
    with KeyReader() as keys:
        tier = pixels.start(keys, console, writer, args.tier)
        try:
            with Live(console=console, screen=True, auto_refresh=False) as live:
                began, i = time.monotonic(), 0
                while time.monotonic() - began < args.seconds:
                    i += 1
                    top = Layout()
                    top.split_row(Layout(Panel(Group(*rose(i)), title='rose'), size=25),
                                  Layout(Panel(Group(*minimap(i)), title='minimap'), size=60),
                                  Layout(Panel(Group(*dial(i)), title='inspect dial'), size=25))
                    bottom = Layout()
                    bottom.split_row(Layout(Panel(Group(*plot(i)), title='motion plot'), size=46),
                                     Layout(Panel(Group(*spark(i)), title='spark'), size=36),
                                     Layout(Panel(Text(f'tier {tier}  cell {pixels.CELL}  K {pixels.K}\n'
                                                       f'tiles {pixels.STATS["tiles"]}  kitty B '
                                                       f'{pixels.STATS["upload"]}  sixel B {pixels.STATS["sixel"]}'),
                                                  title='pixels')))
                    page = Layout()
                    page.split_column(Layout(top, size=22), Layout(bottom, size=8), Layout(Text('')))
                    frame = page
                    if (i // 60) % 2:      # a popup over the rose every other 2 s
                        frame = Overlay(page, Panel(Text('a popup over the rose\n' * 4), title='popup'),
                                        30, console.size.width, console.size.height)
                    segments = pixels.frame(list(console.render(frame)), tuple(console.size))
                    pixels.before(writer)
                    from rich.segment import Segments
                    live.update(Segments(segments), refresh=True)
                    pixels.after(writer)
                    keys.read(1 / 30)
        finally:
            stats, cell = dict(pixels.STATS), pixels.CELL
            pixels.stop(console, writer)
    print(f'tier {tier}, cell {cell}, {i} frames; {stats}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('demo')
    d.add_argument('--tier', default='auto', choices=pixels.TIERS)
    d.add_argument('--seconds', type=float, default=20.0)
    b = sub.add_parser('bench')
    b.add_argument('--cell', default='10x20')
    b.add_argument('--updates', type=int, default=60)
    p = sub.add_parser('png')
    p.add_argument('out')
    p.add_argument('--cell', default='10x20')
    args = parser.parse_args()
    {'demo': demo, 'bench': bench, 'png': png}[args.cmd](args)


if __name__ == '__main__':
    os.environ.setdefault('COLORTERM', 'truecolor')
    main()
