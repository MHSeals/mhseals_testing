"""The Drive minimap column (UIW30): the 150-column cut, the 2 Hz cache, zoom keys and auto_tab."""
import io
import os
from pathlib import Path

import pytest
from rich.console import Console
from rich.text import Text

from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.minimap.panel import MiniMap
from mhseals_testing.ui.minimap.state import MapState
from test_app import make_app, nav_app, run
from test_layout import monitor, view
from mhseals_testing.profiles import load_profile


def state():
    s = MapState()
    for i in range(30):
        s.ingest('pose_map', t=float(i), x=i * 0.5, y=0.0, yaw=0.0)
    s.ingest('plan', points=[(15, 0), (18, 5), (20, 10)])
    return s


def drive_text(width, height, minimap):
    v = {**view(load_profile('boat')), 'minimap': minimap}
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=False)
    console.print(layout.render(monitor().snapshot(), v, width, height))
    return console.file.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_drive_160x45_matches_its_golden():
    golden = Path(__file__).parent / 'golden' / 'layout' / 'drive_160x45.txt'
    actual = drive_text(160, 45, MiniMap(state(), clock=lambda: 0.0))
    if os.environ.get('UPDATE_GOLDEN'):
        golden.write_text(actual)
    assert actual == golden.read_text(), 'drive_160x45.txt differs; UPDATE_GOLDEN=1 to regenerate'


def test_no_minimap_column_at_149_columns():
    assert 'Minimap' in drive_text(150, 45, MiniMap(state()))
    assert 'Minimap' not in drive_text(149, 45, MiniMap(state()))


def test_frame_is_drawn_at_most_twice_a_second():
    now, s = [0.0], state()
    mini = MiniMap(s, clock=lambda: now[0])
    first = mini(26, 8)
    s.ingest('pose_map', t=40.0, x=50.0, y=0.0, yaw=0.0)
    now[0] = 0.2
    assert mini(26, 8) is first
    now[0] = 0.6
    assert mini(26, 8) is not first


def test_zoom_steps_the_half_width_and_zero_resets_it():
    mini = MiniMap(None)
    mini.zoom(1)
    assert mini.subtitle == theme.pick('±', '+-') + '10 m'
    mini.reset()
    assert mini.subtitle == theme.pick('±', '+-') + '20 m'


def test_auto_tab_is_a_persisted_setting_on_by_default():
    from mhseals_testing.harness import settings
    current = settings.Settings()
    assert current.auto_tab is True
    current.set('auto_tab=false')
    assert current.auto_tab is False


def test_ctrl_hjkl_reaches_the_minimap_column_at_160_columns(tmp_path):
    app, _ = make_app(tmp_path)
    app.console.size = (160, 45)
    app.focus = 'thrusters'
    app.dispatch('ctrl-l')
    assert app.focus == 'minimap'
    app.dispatch('ctrl-j')
    assert app.focus == 'activity'
    app.focus = 'minimap'
    app.dispatch('ctrl-h')
    assert app.focus == 'thrusters'


def test_ascii_render_has_no_braille_arrow_or_escapes(monkeypatch):
    from mhseals_testing.ui.minimap import panel
    monkeypatch.setattr(theme, 'ASCII', True)
    s = state()
    s.ingest('objects', items=[])
    mini = MiniMap(s, clock=lambda: 0.0)
    text = '\n'.join(mini(26, 10))
    assert text.isascii() and '\x1b' not in text
    assert mini.subtitle == '+-20 m'


def test_layer_styles_resolve_and_trail_bands_map_to_tokens():
    from mhseals_testing.ui.minimap.panel import style
    assert style('boat:accent') and style('trail.gps.fix') and style('trail.speed.2') and style('trail.track.ok')
    assert style('object:obj.red@50').endswith('dim')


def test_minimap_keys_when_focused(tmp_path):
    app, _ = make_app(tmp_path)
    app.focus = 'minimap'
    app.dispatch('z')
    assert app.minimap.half == 10
    app.dispatch('Z')
    app.dispatch('Z')
    assert app.minimap.half == 40
    app.dispatch('0')
    assert app.minimap.half == 20


@pytest.mark.parametrize('flag,tab', [(True, 'Map'), (False, 'Drive')])
def test_starting_nav2_bringup_switches_to_map_only_with_auto_tab(tmp_path, monkeypatch, flag, tab):
    monkeypatch.setitem(layout.BODIES, 'Map', lambda snapshot, view, width, height: Text('MAP'))
    app, _ = nav_app(tmp_path)
    app.settings.auto_tab = flag
    run(app, 'n')
    assert app.tab == tab
    if flag:
        app.show_tab('Drive')
        assert app.focus == 'status'   # TabMemory kept Drive's focus


@pytest.mark.parametrize('half', [8, 40])
def test_nan_yaw_draws_a_warn_arrow_and_no_ray(half):
    from mhseals_testing.ui.minimap.panel import draw
    s = MapState()
    s.ingest('pose_map', t=0.0, x=1.0, y=1.0, yaw=float('nan'))
    canvas = draw(s, 40, 12, half)
    assert 'boat:warn' in repr(vars(canvas)) and 'boat:accent' not in repr(vars(canvas))


def test_a_failing_draw_keeps_the_last_frame(monkeypatch):
    from mhseals_testing.ui.minimap import panel
    mm = MiniMap(state(), clock=iter(range(0, 1000, 10)).__next__)
    good = mm(40, 12)
    monkeypatch.setattr(panel, 'draw', lambda *a: 1 / 0)
    assert mm(40, 12) == good


def test_auto_tab_stays_on_drive_while_map_has_no_body(tmp_path, monkeypatch):
    monkeypatch.delitem(layout.BODIES, 'Map', raising=False)
    app, _ = nav_app(tmp_path)
    app.settings.auto_tab = True
    run(app, 'n')
    assert app.tab == 'Drive'


def test_drive_minimap_leaves_procedure_keys_to_the_procedures(tmp_path, monkeypatch):
    """e (centre), w and g bind on the Map tab only: on Drive they stay the procedures."""
    app, _ = make_app(tmp_path)
    app.console.size = (160, 45)
    app.focus = 'minimap'
    ran = []
    monkeypatch.setattr(app, 'run_procedure', lambda p: ran.append(p.key))
    app.dispatch('e')
    assert ran == ['e']
    app.show_tab('Map')
    app.dispatch('e')
    assert ran == ['e']
