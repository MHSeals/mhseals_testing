"""Render the README screenshots offline: python3 docs/screens/render.py [OUT_DIR]  (default: this folder).

Synthetic data from the test fixtures, no ROS, no boat and no terminal; the scenes in screens.yaml decide
the images and the README's screens table. An image scene draws the TUI's own kitty/sixel tiles (pixels.py)
as one PNG over Rich's SVG text, at the cells sixel paints them; a text scene is the braille fallback.
`make -C docs screens` regenerates them, `test/test_screens.py` fails when the committed SVGs are stale.
"""
import base64
import io
import json
import math
import os
import re
import struct
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / 'test')]   # mhseals_hardware comes from PYTHONPATH, as for the tests
os.environ['COLORTERM'] = 'truecolor'
os.environ.pop('NO_COLOR', None)
os.environ.pop('TMUX', None)
os.environ.pop('COLORFGBG', None)   # the dark palette, whatever terminal runs this
os.environ['TZ'] = 'UTC'   # the agent scene prints clock times; CI renders in UTC
import time  # noqa: E402
time.tzset()

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from rich.console import Console  # noqa: E402
from rich.segment import Segment, Segments  # noqa: E402
from rich.terminal_theme import MONOKAI  # noqa: E402

from mhseals_testing.profiles import load_profile  # noqa: E402
from mhseals_testing.ui import layout, pixels  # noqa: E402
from mhseals_testing.ui.minimap import tasks  # noqa: E402
from mhseals_testing.ui.minimap.marks import Obj  # noqa: E402
from mhseals_testing.ui.minimap.layerbox import LayerBox  # noqa: E402
from mhseals_testing.ui.minimap.panel import MiniMap  # noqa: E402
from mhseals_testing.ui.minimap.state import MapState  # noqa: E402
from mhseals_testing.ui.popups import inspect  # noqa: E402

import test_agent_view  # noqa: E402
import test_dashboard  # noqa: E402
import test_inspect_popup  # noqa: E402

PROFILE = load_profile('boat')
HINTS = [('Tab', 'Focus'), ('a', 'Arm'), ('Space', 'Stop'), ('?', 'Help'), ('q', 'Quit')]


def base_view(**extra):
    return test_dashboard.view(PROFILE, **{'host': 'blastoise', 'hints': HINTS, 'notice': 'Ready', **extra})


def drive():
    monitor, _ = test_inspect_popup.scene()
    return monitor.snapshot(PROFILE.required_tf), base_view(tab='Drive', armed=True, arm_label='armed')


def map_state():
    s = MapState()
    s.ingest('gps_raw', fix_type=6, h_acc_m=0.4, vel_cms=80)
    for i in range(300):
        a = i / 299 * 1.2
        s.ingest('pose_map', t=i * 0.4, x=14 * math.sin(a) - 14 * math.sin(1.2),
                 y=-14 * math.cos(a) + 14 * math.cos(1.2), yaw=0.0)
    s.ingest('pose_map', t=120.0, x=0.0, y=0.0, yaw=math.radians(30))
    s.ingest('plan', points=[(0, 0), (6, 4), (12, 9), (16, 14)])
    s.ingest('tf_static', transforms={('base_link', 'front_camera_link'): (0.5, 0.0, 0.3, 0.0)})
    s.ingest('fov', hfov_deg=110.0, range_m=12.0, frame='front_camera_link')
    s.ingest('objects', items=[
        Obj(6, 1, 'red_buoy', 0.9), Obj(4, 6, 'green_buoy', 0.9), Obj(11, 4, 'red_pole_buoy', 0.8),
        Obj(-6, 8, 'yellow_target_boat', 0.85, motion='moving', vx=1.5, vy=-0.8),
        Obj(14, -4, 'black_buoy', 0.6, motion='unknown')])
    message = json.loads((ROOT / 'test/fixtures/task_hypotheses.json').read_text())
    s.ingest('tasks', items=tasks.parse(message))
    s.ingest('mission_goal', x=12.0, y=-2.0, yaw=0.0, why='search the east field for the speed gate')
    return s


def map_():
    monitor, _ = test_inspect_popup.scene()
    state = map_state()
    minimap = MiniMap(state, clock=lambda: 120.0, half=20.0)
    return monitor.snapshot(PROFILE.required_tf), base_view(tab='Map', focus='map', minimap=minimap,
                                                            layerbox=LayerBox(None, state))


def compass():
    modal = test_inspect_popup.popup_for('Yaw')
    monitor, _ = test_inspect_popup.scene()
    return monitor.snapshot(PROFILE.required_tf), base_view(tab='Drive', modal=modal)


def agent():
    record = test_agent_view.record(test_agent_view.claim())
    desk = test_agent_view.desk()
    monitor, _ = test_inspect_popup.scene()
    extra = dict(armed=True, authority=('agent', f'AGENT {test_agent_view.WHO} 4m'), activity='agent',
                 focus='activity', agent_desk=desk, authority_record=record,
                 agent=test_agent_view.authority_chip.banner(record, test_agent_view.NOW, desk=desk))
    return monitor.snapshot(PROFILE.required_tf), base_view(tab='Drive', **extra)


def agent_stopped():
    """Space under an agent claim: STOPPED keeps row two and names the paused claim."""
    view = test_agent_view
    record = dict(view.record(view.claim(state='suspended'), owner='hold'), since_ns=view.NS - int(8e9),
                  holds=[{'cause': 'keys', 'by': 'tui', 'since_ns': view.NS - int(8e9)}])
    desk = dict(view.desk(), act=None)
    monitor, _ = test_inspect_popup.scene()
    extra = dict(armed=False, authority=view.authority_chip.chip(record, view.NOW), activity='agent',
                 focus='activity', agent_desk=desk, authority_record=record,
                 notice='stopped by operator; outputs neutral',
                 agent=view.authority_chip.banner(record, view.NOW, desk=desk))
    return monitor.snapshot(PROFILE.required_tf), base_view(tab='Drive', **extra)


SCENES = {'drive': drive, 'map': map_, 'compass': compass, 'agent': agent, 'agent-stopped': agent_stopped}


# The scene's `graphics`: `text` draws the braille, `image` the tiles a sixel or kitty terminal shows
# (sixel's placement: kitty keeps the minimap braille).
TIERS = {'text': 'text', 'image': 'sixel'}
CELL = (12, 24)                 # px per cell the tiles are painted at: about Rich's SVG cell, as a terminal draws them
SVG_CELL = (12.2, 24.4)         # Rich's export_svg cell: char_height 20, aspect 0.61, line height 1.22
_MARKS = re.compile('[\U000F0000-\U000FFFFD]')   # pixels' sixel markers: cells an image covers


def png(rgba):
    """A deterministic RGBA PNG (stdlib only)."""
    h, w = rgba.shape[:2]
    raw = b''.join(b'\0' + row.tobytes() for row in rgba)

    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def overlay(width, height, parts):
    """One transparent screen-sized image with every tile part at its cell, or None."""
    if not parts:
        return None
    cw, ch = CELL
    canvas = np.zeros((height * ch, width * cw, 4), np.uint8)
    for row, col, rgba in parts:
        h, w = rgba.shape[:2]
        canvas[row * ch:row * ch + h, col * cw:col * cw + w] = rgba
    return canvas


def render(entry):
    width, height = entry['size']
    pixels.use(TIERS[entry.get('graphics', 'text')], cell=CELL)
    snapshot, view = SCENES[entry['scene']]()
    console = Console(width=width, height=height, file=io.StringIO(), record=True,
                      force_terminal=True, color_system='truecolor', legacy_windows=False)
    segments = pixels.frame(list(console.render(layout.render(snapshot, view, width, height))), (width, height))
    image = overlay(width, height, pixels.placed() if pixels.TIER == 'sixel' else [])
    console.print(Segments([Segment(_MARKS.sub(' ', s.text), s.style, s.control) for s in segments]))
    svg = console.export_svg(title='boat_test', theme=MONOKAI, unique_id=entry['name'])
    if image is not None:
        uri = 'data:image/png;base64,' + base64.b64encode(png(image)).decode()
        tag = (f'<image x="0" y="1.5" width="{width * SVG_CELL[0]:g}" height="{height * SVG_CELL[1]:g}" '
               f'preserveAspectRatio="none" style="image-rendering:pixelated" href="{uri}"/>')
        matrix = f'<g class="{entry["name"]}-matrix">'
        svg = svg.replace(matrix, tag + matrix, 1)
    return svg


RICH_VERSION = 'rich-version.txt'   # the rich that drew the SVGs; another rich draws other bytes
README = ROOT / 'README.md'
BLOCK = re.compile(r'(<!-- screens: docs/screens/screens.yaml -->).*?(<!-- /screens -->)', re.S)   # one line


def table(entries):
    """The README's screens table: two per row, each image with its caption."""
    cells = [f'<td width="50%"><img src="docs/screens/{e["name"]}.svg" alt="{e["alt"]}">'
             f'<br><sub>{e["caption"]}</sub></td>' for e in entries]
    return '<table>' + ''.join(f'<tr>{"".join(cells[i:i + 2])}</tr>' for i in range(0, len(cells), 2)) + '</table>'


def readme(entries, text):
    """``text`` (the README) with its screens table drawn from screens.yaml."""
    out, n = BLOCK.subn(lambda m: m.group(1) + table(entries) + m.group(2), text)
    if n != 1:
        raise SystemExit(f'{README}: no <!-- screens: docs/screens/screens.yaml --> ... <!-- /screens --> block')
    return out


def render_all(out, readme_to=README):
    from importlib.metadata import version
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    entries = yaml.safe_load((HERE / 'screens.yaml').read_text())
    for entry in entries:
        (out / f"{entry['name']}.svg").write_text(render(entry))
    (out / RICH_VERSION).write_text(version('rich') + '\n')
    readme_to.write_text(readme(entries, README.read_text()))


if __name__ == '__main__':
    # The default redraws the images here and the README's table; an OUT_DIR (the staleness test) gets both.
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE
    render_all(out, README if out == HERE else out / 'README.md')
