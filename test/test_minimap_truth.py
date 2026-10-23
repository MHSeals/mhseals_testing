"""Map against sim truth (map.md 11.2-11.3): fixture goldens always, the live check only with CRANE_ML."""
import json
import os
from functools import partial
from pathlib import Path

import numpy as np
import pytest

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui.minimap import canvas, truth, view

FIXTURE = Path(__file__).parent / 'fixtures' / 'minimap' / 'crane_nav_channel'
SIZES = ((56, 30), (120, 40))


def load(folder=FIXTURE):
    return dict(
        elements=json.loads((folder / 'course_truth.json').read_text())['elements'],
        odom=truth.read_jsonl(folder / 'odom.jsonl'),
        crumbs=truth.read_jsonl(folder / 'trail.jsonl'),
        tracks=json.loads((folder / 'objects.json').read_text())['tracks'])


def truth_in_map(data):
    transform = truth.align(data['odom'], data['crumbs'])
    points = truth.apply(transform, [[e['x'], e['y']] for e in data['elements']])
    track = truth.apply(transform, [[o['x'], o['y']] for o in data['odom']])
    return [(e['label'], tuple(p)) for e, p in zip(data['elements'], points)], track


def fit_view(data, cols, rows):
    points = [(c['x'], c['y']) for c in data['crumbs']] + [(t['x'], t['y']) for t in data['tracks']]
    return view.fit(points, cols * 2, rows * 4)


def render(data, cols, rows):
    """ASCII scene: trail dots, truth dots and tracked objects as their class letter."""
    centre, half = fit_view(data, cols, rows)
    classes = load_classes()
    pos = lambda p: ((p[0] - centre[0]) / half * cols + cols, rows * 2 - (p[1] - centre[1]) / half * cols)
    c = canvas.Canvas(cols, rows)
    for crumb in data['crumbs']:
        c.dot(*map(int, map(np.floor, pos((crumb['x'], crumb['y'])))), 'trail')
    labels, _ = truth_in_map(data)
    for _, p in labels:
        c.dot(*map(int, map(np.floor, pos(p))), 'truth_dot')
    cell = partial(truth.cell_of, centre=centre, half_width=half, cols=cols, rows=rows)
    for t in data['tracks']:
        if t['score'] >= truth.MIN_SCORE:
            c.glyph(*cell((t['x'], t['y'])), classes.symbol(t['label']).ascii, 'object')
    return '\n'.join(line.rstrip(canvas.BLANK) for line in c.text()).replace(canvas.BLANK, ' ') + '\n'


@pytest.mark.parametrize('cols,rows', SIZES)
def test_scene_matches_golden(cols, rows, request):
    text = render(load(), cols, rows)
    golden = FIXTURE / f'golden_fit_{cols}x{rows}.txt'
    if request.config.getoption('--update-goldens'):
        golden.write_text(text)
    assert text == golden.read_text()


def check_truth(data, cols=56, rows=30):
    """The map-vs-truth assertions of map.md 11.3 for one session's data."""
    centre, half = fit_view(data, cols, rows)
    cell = partial(truth.cell_of, centre=centre, half_width=half, cols=cols, rows=rows)
    labelled, track = truth_in_map(data)
    assert truth.object_misses(data['tracks'], labelled, cell) == []
    assert truth.trail_share(data['crumbs'], [tuple(p) for p in track], cell) >= truth.TRAIL_SHARE


def test_tracks_and_trail_are_within_one_cell_of_truth():
    check_truth(load())


def test_a_confident_track_in_the_wrong_place_or_class_is_a_miss():
    data = load()
    data['tracks'][0]['x'] += 3.0
    data['tracks'][1]['label'] = 'red_pole_buoy'
    centre, half = fit_view(data, 56, 30)
    cell = partial(truth.cell_of, centre=centre, half_width=half, cols=56, rows=30)
    misses = truth.object_misses(data['tracks'], truth_in_map(data)[0], cell)
    assert [m['score'] for m in misses] == [0.92, 0.88]


def test_umeyama_recovers_a_rotation_and_shift_and_rejects_a_reflection():
    src = np.array([[0, 0], [4, 0], [4, 2], [1, 5.0]])
    a = np.radians(40)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    got_rot, got_t = truth.umeyama(src, src @ rot.T + [3, -2])
    assert np.allclose(got_rot, rot) and np.allclose(got_t, [3, -2])
    assert np.linalg.det(truth.umeyama(src, src * [1, -1])[0]) > 0
    with pytest.raises(ValueError):
        truth.umeyama(src[:1], src[:1])


def test_cell_of_puts_the_view_centre_in_the_middle_cell():
    assert truth.cell_of((5, 5), (5, 5), 10, 56, 30) == (28, 15)
    assert truth.cell_of((5 + 10, 5), (5, 5), 10, 56, 30)[0] == 56  # right edge is off canvas


def test_torn_trail_lines_are_skipped(tmp_path):
    f = tmp_path / 't.jsonl'
    f.write_text('{"t": 1}\n{"t": 2, "x"\n\n{"t": 3}\n')
    assert [r['t'] for r in truth.read_jsonl(f)] == [1, 3]


@pytest.mark.skipif(not os.environ.get('CRANE_ML'), reason='set CRANE_ML to a headless sim session folder')
def test_live_session_matches_truth():
    check_truth(load(Path(os.environ['CRANE_ML'])))
