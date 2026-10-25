"""Render the README screenshots offline: python3 docs/screens/render.py [OUT_DIR]  (default: this folder).

Synthetic data from the test fixtures, no ROS and no boat; the scenes in screens.yaml decide the images.
`make -C docs screens` regenerates them, `test/test_screens.py` fails when the committed SVGs are stale.
"""
import io
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / 'test')]   # mhseals_hardware comes from PYTHONPATH, as for the tests
os.environ['COLORTERM'] = 'truecolor'
os.environ.pop('NO_COLOR', None)
os.environ.pop('TMUX', None)
os.environ['TZ'] = 'UTC'   # the agent scene prints clock times; CI renders in UTC
import time  # noqa: E402
time.tzset()

import yaml  # noqa: E402
from rich.console import Console  # noqa: E402
from rich.terminal_theme import MONOKAI  # noqa: E402

from mhseals_testing.profiles import load_profile  # noqa: E402
from mhseals_testing.ui import layout  # noqa: E402
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
    return test_dashboard.view(PROFILE, host='blastoise', hints=HINTS, notice='Ready', **extra)


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


SCENES = {'drive': drive, 'map': map_, 'compass': compass, 'agent': agent}


def render(entry):
    width, height = entry['size']
    snapshot, view = SCENES[entry['scene']]()
    console = Console(width=width, height=height, file=io.StringIO(), record=True,
                      force_terminal=True, color_system='truecolor', legacy_windows=False)
    console.print(layout.render(snapshot, view, width, height))
    return console.export_svg(title='boat_test', theme=MONOKAI, unique_id=entry['name'])


RICH_VERSION = 'rich-version.txt'   # the rich that drew the SVGs; another rich draws other bytes


def render_all(out):
    from importlib.metadata import version
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for entry in yaml.safe_load((HERE / 'screens.yaml').read_text()):
        (out / f"{entry['name']}.svg").write_text(render(entry))
    (out / RICH_VERSION).write_text(version('rich') + '\n')


if __name__ == '__main__':
    render_all(sys.argv[1] if len(sys.argv) > 1 else HERE)
