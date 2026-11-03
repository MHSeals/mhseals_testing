"""The Labels tab: its panel from a synthetic model, and the review keys on a live App."""

import io

import pytest
from rich.console import Console

from mhseals_testing.annotate import manifest
from mhseals_testing.ui.panels import labels

pytestmark = pytest.mark.smoke   # tools/smoke

T0 = 1_762_180_000.0      # 2025-11-03 14:13:20 UTC


def seg(kind, a, b, label='', status='pending', quality='ok', **extra):
    return {'id': f'{kind}:{a}', 'kind': kind, 't0': T0 + a, 't1': T0 + b, 'label': label, 'status': status,
            'quality': quality, 'note': '', 'flags': [], **extra}


def model(cursor=4, readonly=False):
    rows = [seg('procedure', 60, 400, 'characterize', status='accepted', quality='suspect', flags=['wake']),
            seg('steady', 80, 92, 'steady', quality='suspect', flags=['wake']),
            seg('mark', 95, 95, 'wake', note='cruiser passed'),
            seg('neutral', 130, 142, 'neutral'),
            seg('intervention', 300, 330, 'keyboard over agent', quality=''),
            seg('bad', 340, 360, 'bad data', status='rejected', quality=''),
            seg('said', 420, 420, 'hold position here', outcome='done'),
            seg('said', 470, 470, '', outcome='')]
    return {'session': '20261103_141320_blastoise_boat', 'live': False, 'scales': ('wind', 'waves', 'current'),
            'conditions': {'wind': 2, 'waves': 1, 'current': 0}, 'rows': rows,
            'armed': [seg('armed', 50, 500)], 'calibration': {'neutral:130'}, 'commands': rows[-2:],
            'cursor': cursor, 'selected': rows[cursor - 3] if cursor >= 3 else None, 'pending': 4,
            'readonly': readonly, 't0': T0, 't1': T0 + 520}


def text(renderable, width=120, height=30):
    console = Console(width=width, height=height, file=io.StringIO(), force_terminal=False, color_system=None)
    console.print(renderable)
    return console.file.getvalue()


def test_panel_shows_conditions_timeline_and_items():
    out = text(labels.render(model(), 120, 30))
    assert 'Wind' in out and 'moderate' in out and 'unset' in out
    assert 'runs' in out and 'calib' in out and 'marks' in out
    assert 'cruiser passed' in out and 'hold position here' in out and 'words later (n)' in out
    assert '4 to review' in out and '1 calibration window' in out


def test_panel_fits_80_columns_and_says_read_only():
    out = text(labels.render(model(readonly=True), 80, 24), 80, 24)
    assert max(len(line) for line in out.splitlines()) <= 80 and 'edits after disarm' in out


def test_labels_tab_review_keys(tmp_path):
    from test_app import journaling, make_app
    app, _ = make_app(tmp_path)
    journal = journaling(app)
    app.dispatch('b')
    app.dispatch('"')
    app.dispatch('1')                         # the first phrase
    app.dispatch('L')
    assert app.tab == 'Labels' and app.focus == 'labels'
    app.dispatch('right')                     # wind 1
    app.dispatch('right')                     # wind 2
    assert app.labels_model()['conditions']['wind'] == 2
    rows = app.labels_rows()
    app.cursors['labels'] = 3 + [r['kind'] for r in rows].index('mark')
    app.dispatch('r')                         # reject the mark
    app.cursors['labels'] = 3 + [r['kind'] for r in rows].index('said')
    app.dispatch('2')                         # outcome: partly
    doc = manifest.load(app.paths.directory)
    assert [s['status'] for s in doc['segments'] if s['kind'] == 'mark'] == ['rejected']
    assert doc['commands'][0]['text'] == 'hold position here' and doc['commands'][0]['outcome'] == 'partly'
    assert doc['conditions'][-1]['wind'] == 2
    app.dispatch('u')                         # undo the outcome
    assert manifest.load(app.paths.directory)['commands'][0]['outcome'] == ''
    app.arming.armed = True
    app.dispatch('y')
    assert 'wait for disarmed' in app.notice
    assert sum(1 for e in journal.read(app.paths.journal) if e['phase'] == 'label_edit') == 3
