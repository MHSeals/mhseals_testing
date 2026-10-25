"""Perception popup (V): snapshots at 100 and 60 columns, NO_COLOR and ASCII."""
import io
import os
import re
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

from rich.console import Console

from mhseals_testing.health.model import Fact
from mhseals_testing.ros import camera_feed
from test_camera_view import Bus
from mhseals_testing.ui import app, layout, perception

TESTS = Path(__file__).parent


def facts():
    cls = {'red_buoy': dict(conf=.71, rate=1.9, history=[.6, .7, .72, .71]),
           'green_buoy': dict(conf=.68, rate=1.7, history=[.5, .66, .68]),
           'black_buoy': dict(conf=0, rate=0), 'blue_buoy': dict(conf=0, rate=0), 'a_pole': dict(conf=0, rate=0),
           'z_pole': dict(conf=0, rate=0)}
    stages = {'camera': {'hz': 14.9}, 'sdk': {'hz': 14.6}, 'backend': {'hz': 9.0}, 'tracker': {'hz': 10.0}}
    return {'perc.zed_native.stages': Fact(stages),
            'perc.zed_native.latency': Fact(.231, data=dict(p95=.312, history=[.2, .25, .3, .23])),
            'perc.zed_native.inference_ms': Fact(52.0),
            'perc.zed_native.clock_offset': Fact(-.004),
            'perc.zed_native.tracker_latency': Fact(.112),
            'perc.zed_native.funnel_loss': Fact(.12, data=dict(drops={'stale': 10, 'reorder': 1, 'malformed': 0})),
            'perc.zed_native.split': Fact('S12 M1 U2 P8'),
            'perc.zed_native.ids': Fact('true', data=dict(id_assoc_share=.93)),
            'perc.zed_native.costmap_age': Fact(.21),
            'perc.zed_native.heading': Fact('ok', data=dict(lag_s='0.03', rho='0.21', excited='true')),
            'perc.zed_native.classes': Fact(2, data=cls)}


def draw(width, height=40, **kwargs):
    panel, panel_width = layout.popup(perception.modal(facts(), **kwargs), width, height)
    console = Console(width=panel_width, file=io.StringIO(), record=True)
    console.print(panel)
    return console.export_text()


def test_popup_shows_each_section_at_100_columns():
    text = draw(100)
    for want in ('Perception', 'zed_native', 'camera 14.9', 'p50 231', 'p95 312', 'clock -4 ms',
                 'infer 52 ms', 'tracker 112 ms', 'loss 12%', 'stale 10', 'S12 M1 U2 P8', 'ids 93%',
                 'costmap 0.2 s', 'lag 0.03 s ρ 0.21 ok', 'red_buoy', 'conf 0.71', '1.9/s', 'v view',
                 '4 not seen', 'a_pole, black_buoy, blue_buoy', 'Esc close'):
        assert want in text, want
    assert text.index('red_buoy') < text.index('green_buoy')   # busiest class first


def test_stage_loss_over_fifteen_percent_is_marked():
    text = draw(100)
    assert 'sdk 14.6 →! backend 9.0 → tracker 10.0' in text.replace('  ', ' ')
    assert 'camera 14.9 → sdk' in text


def test_popup_fits_60_columns_and_clips_to_a_short_terminal():
    assert max(len(line) for line in draw(60).splitlines()) <= 60
    lines = draw(60, height=12).splitlines()
    assert len(lines) <= 12 and 'scroll' in '\n'.join(lines)


def test_no_facts_says_so_and_unknown_source_is_empty():
    modal = perception.modal({})
    assert modal['rows'] == [('', 'No perception facts yet')]
    assert perception.sources(facts()) == ['zed_native']


def test_capital_v_opens_the_popup_and_lowercase_keeps_results_log():
    seen = []
    host = SimpleNamespace(perception_popup=lambda view=False: seen.append('popup'), activity='results', review=None)
    host.open = lambda action: app.App.open(host, action)
    app.App.dispatch(host, 'V')
    assert seen == ['popup']
    host = SimpleNamespace(perception_popup=lambda view=False: seen.append('popup'), activity='results',
                           focus='activity', cursors={'activity': 3}, panels=lambda: ['activity'],
                           review=None, manual=False, tab='Drive')
    host.open = lambda action: app.App.open(host, action)
    host.on_activity = lambda key: app.App.on_activity(host, key)
    app.App.dispatch(host, 'v')
    assert seen == ['popup'] and host.activity == 'log'
    app.App.dispatch(host, 'v')
    assert seen == ['popup'] and host.activity == 'results'


def test_popup_closes_on_escape_and_scrolls():
    shown = {}
    keys = iter(['j', 'escape'])
    host = SimpleNamespace(facts=facts(), scroll=app.App.scroll,
                           popup=lambda modal, handle, refresh=None: shown.update(
                               modal=modal, results=[handle(next(keys)) for _ in range(2)]))
    app.App.perception_popup(host)
    assert shown['results'] == [None, True] and shown['modal']['scroll'] == 1


def test_popup_rebuilds_rows_from_changed_facts_without_a_key():
    held = {}
    host = SimpleNamespace(facts={}, scroll=app.App.scroll)

    def popup(modal, handle, refresh=None):
        held['before'] = list(modal['rows'])
        modal['scroll'] = 2
        host.facts.update(facts())
        refresh()
        held.update(after=modal['rows'], scroll=modal['scroll'])
    host.popup = popup
    app.App.perception_popup(host)
    assert held['before'] == [('', 'No perception facts yet')]
    assert any(name == 'red_buoy' for name, _ in held['after']) and held['scroll'] == 2


def test_heading_row_says_when_the_boat_has_not_turned_enough():
    held = {'perc.s.heading': Fact('ok', data=dict(excited='false'))}
    assert perception.heading_row(held, 's') == 'not excited · ok'
    assert perception.heading_row({'perc.s.heading': Fact('error', data=dict(lag_s='0.2', rho='0.8'))}, 's') \
        == 'lag 0.20 s ρ 0.80 error'


def test_seen_classes_past_the_limit_are_counted_apart_from_not_seen():
    cls = {f'c{i}': dict(rate=10 - i * .5) for i in range(10)}
    cls['idle'] = dict(rate=0)
    rows = perception.class_rows({'perc.s.classes': Fact('x', data=cls)}, 's')
    assert len([n for n, _ in rows if n]) == perception.CLASS_ROWS
    assert ('', '+2 more') in rows
    assert ('', '1 not seen: idle') in rows


SCRIPT = """
import sys
sys.path.insert(0, sys.argv[1])
from test_perception_popup import draw
from rich.console import Console
from mhseals_testing.ui import layout, perception
from test_perception_popup import facts
panel, w = layout.popup(perception.modal(facts()), 100, 40)
console = Console(width=w, force_terminal=True)   # real terminal output, honours NO_COLOR
console.print(panel)
"""


def run(**env):
    base = {k: v for k, v in os.environ.items() if k not in ('MHS_ASCII', 'TERM', 'NO_COLOR')}
    out = subprocess.run([sys.executable, '-c', SCRIPT, str(TESTS)], env={**base, **env},
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    return out.stdout


def test_ascii_mode_is_pure_ascii():
    text = run(MHS_ASCII='1')
    assert text.isascii() and 'red_buoy' in text and '->' in text


def test_no_color_output_has_no_colour_codes():
    colour = re.compile(r'\x1b\[(?:[0-9;]*;)?(?:3[0-8]|4[0-8]|9[0-7]|10[0-7])[;m]')
    text = run(NO_COLOR='1')
    assert 'red_buoy' in text and not colour.search(text)   # bold/dim may stay
    assert colour.search(run())                              # control: colour without NO_COLOR


def test_the_popup_opens_the_view_on_v_and_releases_it_on_every_exit(tmp_path):
    bus = Bus()
    written = []
    keys = iter(['v', 'd', 'v', 'v', 'escape', 'escape'])
    seen = []

    def popup(modal, handle, refresh=None):
        for _ in range(4):
            handle(next(keys))
            seen.append(len(bus.live))
        handle_result = [handle(next(keys)), handle(next(keys))]
        seen.append(handle_result)
    host = SimpleNamespace(
        facts={}, scroll=app.App.scroll, popup=popup, write_terminal=written.append,
        keys=SimpleNamespace(graphics=True, xtversion='kitty(0.35)'), paths=SimpleNamespace(directory=tmp_path),
        camera_detections=lambda size, floor: [],
        camera_feed=lambda: camera_feed.CameraFeed(bus.subscribe, bus.unsubscribe))
    app.App.perception_popup(host)
    assert seen[:4] == [1, 1, 0, 1] and seen[4] == [None, True]    # v opens, v closes, v reopens; Esc closes the view, then the popup
    assert not bus.live


def view_host(bus, tmp_path, popup, feed=True):
    return SimpleNamespace(
        facts={}, scroll=app.App.scroll, popup=popup, write_terminal=lambda text: None, notice='',
        keys=SimpleNamespace(graphics=False), paths=SimpleNamespace(directory=tmp_path),
        console=SimpleNamespace(size=SimpleNamespace(width=60)), camera_detections=lambda size, floor: [],
        camera_feed=(lambda: camera_feed.CameraFeed(bus.subscribe, bus.unsubscribe)) if feed else (lambda: None))


def test_view_lists_its_keys_and_status_and_v_without_a_node_says_so(tmp_path):
    bus = Bus()
    shown = {}

    def popup(modal, handle, refresh=None):
        shown['hint'] = modal['hint']
        handle('v')
        refresh()
        shown['hint_open'], shown['body'] = modal['hint'], modal['body']
        handle('escape')
        shown['closed'] = 'body' not in modal
    app.App.perception_popup(view_host(bus, tmp_path, popup))
    assert 'v view' in shown['hint'] and shown['closed']
    for want in ('d boxes', 'D floor', 'f freeze', 's save', '+- rate'):
        assert want in shown['hint_open']
    assert '1 Hz' in shown['body'].renderables[-1].plain

    host = view_host(bus, tmp_path, lambda modal, handle, refresh=None: (handle('v'), refresh(), shown.update(
        body=modal['body'])), feed=False)
    app.App.perception_popup(host)
    assert 'needs a ROS node' in host.notice and 'needs a ROS node' in shown['body'].plain


def test_capital_v_opens_the_popup_with_the_view_already_open(tmp_path):
    bus = Bus()
    seen = []
    host = view_host(bus, tmp_path, lambda modal, handle, refresh=None: seen.append(len(bus.live)))
    app.App.perception_popup(host, view=True)
    assert seen == [1] and not bus.live                             # open inside the popup, released after
    calls = []
    outer = SimpleNamespace(perception_popup=lambda view=False: calls.append(view), review=None)
    app.App.dispatch(outer, 'V')
    assert calls == [True]
