"""Mission panels (ui/panels/mission.py): the Drive strip over Activity and the Map's Mission and Decisions boxes."""
import json
from pathlib import Path

from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.feeds.mission import MissionState
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.minimap.state import MapState
from mhseals_testing.ui.panels import mission as panels

from test_dashboard import monitor, view

STATUS = json.loads((Path(__file__).parent / 'fixtures' / 'mission_status.json').read_text())


def mission(status=STATUS):
    clock = [0.0]
    state = MissionState(MapState(), clock=lambda: clock[0], wall=lambda: 1761400000.0)
    state.ingest('status', status)
    for event in status.get('recent', []):
        state.ingest('event', event)
    return state, clock


def paint(renderable, width=120, height=40):
    console = Console(width=width, height=height, color_system=None, record=True, legacy_windows=False)
    console.print(renderable)
    return console.export_text()


def page(tab, state, width=120):
    extra = {'mission': state} if state is not None else {}     # no feed: no view key
    v = view(load_profile('boat'), tab=tab, focus='map' if tab == 'Map' else 'status',
             layerbox=LayerBox(None, MapState()), **extra)
    return paint(layout.render(monitor().snapshot(), v, width, 40), width)


def test_the_strip_shows_stage_why_and_ladder_and_only_while_fresh():
    state, clock = mission()
    text = paint(panels.strip({'mission': state}), 60)
    lines = text.splitlines()
    assert lines[0].startswith('╭─ Mission · search ─') and lines[1].split('│')[1].strip() == 'sweep_lawnmower · 23 s'
    assert '→ search the east field' in text and '✗ docking ×3 ▸ search · speed_challenge' in text
    assert 'search    nothing left, searching for the spee…' in lines[4]   # own line, the current stage not repeated
    clock[0] = 5.0
    assert panels.strip({'mission': state}) is None and panels.strip({}) is None


def test_drive_gives_the_strip_rows_over_activity_and_none_without_a_status():
    state, clock = mission()
    text = page('Drive', state)
    assert text.index('Mission') < text.index('Activity') and 'sweep_lawnmower' in text
    clock[0] = 9.0
    assert 'sweep_lawnmower' not in page('Drive', state)


def test_map_puts_mission_and_decisions_beside_layers():
    state, clock = mission()
    text = page('Map', state)
    row = next(line for line in text.splitlines() if 'Layers' in line)
    assert row.index('Layers') < row.index('Mission') < row.index('Decisions')
    assert 'T tasks' in text and '✗ docking ×3' in text and 'give up' in text
    clock[0] = 12.0
    assert '12 s old' in page('Map', state)
    assert 'Decisions' not in page('Map', state, width=90)
    assert 'Mission' not in page('Map', None).split('Layers')[1].splitlines()[0]   # no feed: Layers alone


def test_an_old_status_without_the_new_keys_still_draws():
    state, _ = mission({'task': 'docking', 'status': 'RUNNING', 'elapsed': 3.0, 'tip': 'approach the bay'})
    text = paint(panels.strip({'mission': state}), 60)
    assert 'Mission · docking' in text and '│ 3 s' in text and '→ approach the bay' in text


def test_ascii_mode_keeps_every_panel_ascii(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    monkeypatch.setattr(theme, 'SEPARATOR', ' | ')
    monkeypatch.setattr(theme, 'ARROW', '->')
    monkeypatch.setattr(theme, 'OVERFLOW', 'crop')
    state, _ = mission()
    for renderable in (panels.strip({'mission': state}), panels.pane({'mission': state}),
                       panels.decisions({'mission': state}, 8)):
        assert paint(renderable, 60).replace('\n', '').isascii()
