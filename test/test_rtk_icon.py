"""Correction-source icon: ring colour per rover RTK state, fill per link, in every state; render golden.

Update the golden with ``pytest test/test_rtk_icon.py --update-goldens`` and review the diff.
"""

import io
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing.ui import rtk_icon, theme
from mhseals_testing.ui.rtk_row import RtkView as V

GOLDEN = Path(__file__).parent / 'golden' / 'rtk_icon.txt'
HERE4 = dict(base_model='Here4')
STREAM = dict(base='streaming', age=0.4, **HERE4)
NTRIP = dict(base='streaming', kind='ntrip', mount='SRSA', age=0.4)

# (name, view): every link state x every rover state the ring colours
STATES = [
    ('no base, rover 3D', V(fix='3D')),
    ('identifying', V(base='identifying', tick=4, **HERE4)),
    ('survey 0%', V(base='surveying', survey=0.0, **HERE4)),
    ('survey 42%', V(base='surveying', survey=0.42, fix='3D', **HERE4)),
    ('survey 99%', V(base='surveying', survey=0.99, fix='DGPS', base_model='Here+')),
    ('site, rover none', V(**STREAM, site='dock')),
    ('site, rover 2D', V(**STREAM, site='dock', fix='2D')),
    ('site, rover 3D', V(**STREAM, site='dock', fix='3D')),
    ('site, rover DGPS', V(**STREAM, site='dock', fix='DGPS')),
    ('site, rover FLOAT', V(**STREAM, site='dock', fix='FLOAT')),
    ('site, rover FIXED', V(**STREAM, site='dock', fix='FIXED')),
    ('stale 3.0 s FLOAT', V(**dict(STREAM, age=3.0), fix='FLOAT')),
    ('dead 14 s FIXED', V(**dict(STREAM, age=14.0), fix='FIXED')),
    ('NTRIP 4 km FIXED', V(**NTRIP, station_km=4.2, fix='FIXED')),
    ('NTRIP VRS 6 km FLOAT', V(**dict(NTRIP, mount='NAD_MSM4_VRS'), vrs=True, station_km=5.6, fix='FLOAT')),
    ('NTRIP 25 km DGPS', V(**NTRIP, station_km=25.0, fix='DGPS')),
    ('NTRIP 60 km FLOAT', V(**NTRIP, station_km=60.0, fix='FLOAT')),
    ('NTRIP picking', V(base='identifying', kind='ntrip', tick=1)),
    ('external FIXED', V(source='external', fix='FIXED')),
    ('source none', V(source='none', fix='3D')),
    ('base error', V(base='error', **HERE4)),
]


def render(ascii):
    lines = []
    for name, view in STATES:
        lines.append(f'-- {name}')
        lines += [line.plain for line in rtk_icon.pane_lines(view, ascii)]
        lines.append('row ' + rtk_icon.source_row(view, 34, ascii).plain)
    return '\n'.join(lines) + '\n'


def test_render_matches_the_golden(request):
    text = '== unicode\n' + render(False) + '== ascii\n' + render(True)
    if request.config.getoption('--update-goldens'):
        GOLDEN.write_text(text, encoding='utf-8')
    assert text == GOLDEN.read_text(encoding='utf-8')


def test_ascii_render_is_pure_ascii():
    assert render(True).isascii()


@pytest.mark.parametrize('fix,token', [('FIXED', 'rtk.fixed'), ('FLOAT', 'rtk.float'), ('DGPS', 'rtk.dgps'),
                                       ('3D', 'rtk.dgps'), ('2D', 'rtk.none'), ('none', 'rtk.none')])
def test_ring_colour_is_the_rover_state_token(fix, token):
    view = V(**STREAM, fix=fix)
    assert rtk_icon.ring_style(view) == theme.PALETTE.style(token)
    assert rtk_icon.icon(view)[0].spans[-1].style == theme.PALETTE.style(token)


def test_fill_follows_survey_age_and_station_distance():
    assert rtk_icon.fill(V(base='surveying', survey=0.5)) == 0.5
    assert rtk_icon.fill(V(**STREAM)) == 1.0
    assert rtk_icon.fill(V(**dict(STREAM, age=3.0))) == pytest.approx(0.5)
    assert rtk_icon.fill(V(**dict(STREAM, age=6.0))) == 0.0
    near, mid, far = (rtk_icon.fill(V(**NTRIP, station_km=km)) for km in (8, 25, 80))
    assert near == 1.0 and 0.25 < mid < 1.0 and far == 0.25
    assert rtk_icon.fill(V(base='identifying')) is None            # the ring turns instead


def test_identifying_ring_turns_with_the_tick():
    frames = {tuple(l.plain for l in rtk_icon.icon(V(base='identifying', tick=t))) for t in range(12)}
    assert len(frames) == 12
    assert len({rtk_icon.ring_glyph(V(base='identifying', tick=t)) for t in range(4)}) == 4


def test_nocolor_keeps_fill_and_state_readable():
    """NO_COLOR drops colour only: the fill is shape (lit vs dashed segments), the state a word."""
    console = Console(file=io.StringIO(), width=40, no_color=True, force_terminal=True)
    for line in rtk_icon.pane_lines(V(**dict(STREAM, age=3.0), fix='FLOAT')):
        console.print(line)
    out = console.file.getvalue()
    assert '━' in out and '┄' in out and 'rover FLOAT' in out and '\x1b[3' not in out


def test_feed_reads_the_source_from_the_status_document():
    """rtk_base status -> RtkView: the model name for a local base; mountpoint and live distance for NTRIP."""
    from types import SimpleNamespace
    from mhseals_testing.ui.feeds.rtk import build
    spec = SimpleNamespace(kind='gpsraw', name='GPS raw')
    gps = dict(sensors={'GPS raw': dict(status='LIVE', values=dict(fix=6, lat=27.3721, lon=-82.4469, alt=3.0))})
    base = build(dict(base='streaming', age=0.3, model='F9P', base_model='Here4', site='dock',
                      source=dict(kind='base')), gps, (spec,))
    assert (base.kind, base.base_model, base.site, base.fix) == ('base', 'Here4', 'dock', 'FIXED')
    net = build(dict(base='streaming', age=0.3, model='NTRIP', source=dict(kind='ntrip', ntrip=dict(
        mount='SRSA', km=11.0, vrs=False, station=dict(latitude=27.38, longitude=-82.56)))), gps, (spec,))
    assert (net.kind, net.mount, net.base_model) == ('ntrip', 'SRSA', '')
    assert net.station_km == pytest.approx(11.2, abs=0.2) and net.baseline == pytest.approx(net.station_km * 1000)
    legacy = build(dict(base='streaming', age=0.3, model='M8P'), gps, (spec,))     # an older rtk_base
    assert (legacy.kind, legacy.base_model) == ('base', 'Here+')


def test_labels_name_the_base_model_or_the_mountpoint_and_distance():
    assert rtk_icon.label(V(**STREAM, site='dock')) == 'Here4 site dock'
    assert rtk_icon.label(V(base='surveying', base_model='Here+')) == 'Here+ svin'
    assert rtk_icon.label(V(**NTRIP, station_km=12.4)) == 'NTRIP SRSA 12km'
    assert rtk_icon.label(V(**dict(NTRIP, mount='NAD_MSM4_VRS'), vrs=True, station_km=5.6)) == 'NTRIP NAD_MSM4_VRS VRS 6km'
