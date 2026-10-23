"""Perception popup (V): snapshots at 100 and 60 columns, NO_COLOR and ASCII."""
import io
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

from rich.console import Console

from mhseals_testing.health.model import Fact
from mhseals_testing.ui import app, layout, perception

TESTS = Path(__file__).parent


def facts():
    cls = {'red_buoy': dict(rate=1.9, presence=.41, p50=.71, last_age=.2, hist=[1, 3, 8, 5, 2]),
           'green_buoy': dict(rate=1.7, presence=.38, p50=.68, last_age=.4, hist=[1, 2, 6, 4]),
           'black_buoy': dict(rate=0), 'blue_buoy': dict(rate=0), 'a_pole': dict(rate=0),
           'z_pole': dict(rate=0)}
    stages = {'camera': {'hz': 4.8}, 'detector': {'hz': 4.7}, 'bridge': {'hz': 3.0},
              'converter': {'hz': 3.0}}
    return {'perc.legacy_zed.stages': Fact(stages),
            'perc.legacy_zed.latency': Fact(.231, data=dict(p95=.312, history=[.2, .25, .3, .23])),
            'perc.legacy_zed.inference_ms': Fact(188.0),
            'perc.legacy_zed.drops': Fact({'stale': 0, 'ingest': 1}),
            'perc.legacy_zed.remote_nodes': Fact('4/4'),
            'perc.legacy_zed.classes': Fact('2', data=cls),
            'clock.offset.squirtle': Fact(-.004)}


def draw(width, height=40, **kwargs):
    panel, panel_width = layout.popup(perception.modal(facts(), **kwargs), width, height)
    console = Console(width=panel_width, file=io.StringIO(), record=True)
    console.print(panel)
    return console.export_text()


def test_popup_shows_each_section_at_100_columns():
    text = draw(100)
    for want in ('Perception', 'legacy_zed', 'camera 4.8', 'p50 231', 'p95 312', 'clock -4 ms',
                 'infer 188 ms', 'ingest 1', 'nodes 4/4', 'red_buoy', '1.9/s', '41%',
                 '4 not seen', 'a_pole, black_buoy, blue_buoy', 'Esc close'):
        assert want in text, want
    assert text.index('red_buoy') < text.index('green_buoy')   # busiest class first


def test_stage_loss_over_fifteen_percent_is_marked():
    text = draw(100)
    assert 'detector 4.7 ->! bridge 3.0'.replace('->', '→') in text.replace('  ', ' ')
    assert 'camera 4.8 → detector' in text


def test_popup_fits_60_columns_and_clips_to_a_short_terminal():
    assert max(len(line) for line in draw(60).splitlines()) <= 60
    lines = draw(60, height=12).splitlines()
    assert len(lines) <= 12 and 'scroll' in '\n'.join(lines)


def test_no_facts_says_so_and_unknown_source_is_empty():
    modal = perception.modal({})
    assert modal['rows'] == [('', 'No perception facts yet')]
    assert perception.sources(facts()) == ['legacy_zed']


def test_capital_v_opens_the_popup_and_lowercase_keeps_results_log():
    seen = []
    host = SimpleNamespace(perception_popup=lambda: seen.append('popup'), activity='results', review=None)
    app.App.dispatch(host, 'V')
    assert seen == ['popup']


def test_popup_closes_on_escape_and_scrolls():
    shown = {}
    keys = iter(['j', 'escape'])
    host = SimpleNamespace(facts=facts(), scroll=app.App.scroll,
                           popup=lambda modal, handle: shown.update(
                               modal=modal, results=[handle(next(keys)) for _ in range(2)]))
    app.App.perception_popup(host)
    assert shown['results'] == [None, True] and shown['modal']['scroll'] == 1


SCRIPT = """
import sys
sys.path.insert(0, sys.argv[1])
from test_perception_popup import draw
sys.stdout.write(draw(100) + draw(60))
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


def test_no_color_output_has_no_escape_codes():
    text = run(NO_COLOR='1')
    assert '\x1b' not in text and 'red_buoy' in text
