"""Field annotation: the journal fold, the manifest and its reader, and the live edge feed."""

import json
from types import SimpleNamespace

import pytest

from mhseals_testing import data
from mhseals_testing.annotate import fold, manifest

pytestmark = pytest.mark.smoke   # tools/smoke

S = 1_000_000_000


def ev(t, phase, **details):
    return {'t_ns': int(t * S), 'phase': phase, **details}


def journal():
    return [
        ev(100, 'session_start'),
        ev(101, 'conditions_set', wind=2, waves=1, current=1),
        ev(110, 'armed', by='operator'),
        ev(111, 'gnss_fix', fix='FIXED'),
        ev(120, 'characterize_start', key='f'),
        ev(121, 'step', label='surge+', done=0, total=6),
        ev(140, 'step', label='surge-', done=1, total=6),
        ev(150, 'steady', t0_ns=141 * S, command=[0.4, 0, 0], velocity=[0.5, 0, 0], spread=[0.02, 0, 0], fix='FIXED'),
        ev(152, 'marker', id=1, kind='user', note='', tag='wake'),
        ev(160, 'characterize_stop', outcome='aborted by operator', by='operator'),
        ev(184, 'steady', t0_ns=176 * S, command=[0, 0, 0], fix='FIXED'),
        ev(171, 'bad_data', on=True),
        ev(175, 'bad_data', on=False),
        ev(180, 'owner', owner='agent', prev='none'),
        ev(185, 'owner', owner='keyboard', prev='agent'),
        ev(190, 'owner', owner='none', prev='keyboard'),
        ev(191, 'said', id=1, text='hold position here', source='pick', pose={'x': 1.0, 'y': 2.0, 'yaw': 0.1}),
        ev(195, 'hold_on', cause='link lost'),
        ev(198, 'hold_off', cause='link lost'),
        ev(200, 'disarmed', by='operator'),
    ]


def by_kind(folded, kind):
    return [i for i in folded['items'] if i['kind'] == kind]


def test_fold_builds_segments_from_the_journal():
    folded = fold.fold(journal())
    proc, = by_kind(folded, 'procedure')
    assert (proc['t0'], proc['t1'], proc['detail']['outcome']) == (120, 160, 'aborted by operator')
    assert [(s['label'], s['t0'], s['t1']) for s in by_kind(folded, 'step')] == [('surge+', 121, 140), ('surge-', 140, 160)]
    assert [i['label'] for i in by_kind(folded, 'intervention')] == ['characterize aborted', 'keyboard over agent']
    assert by_kind(folded, 'intervention')[1]['t1'] == 190
    assert by_kind(folded, 'link')[0]['t1'] == 198 and by_kind(folded, 'bad')[0]['t1'] == 175
    assert by_kind(folded, 'armed')[0]['t1'] == 200
    assert by_kind(folded, 'mark')[0]['effect'] == 'suspect'



def test_quality_and_calibration_candidates(tmp_path):
    (tmp_path / 'session.json').write_text(json.dumps({'id': 's1', 'profile': {'name': 'boat'}}))
    doc = manifest.build(tmp_path, journal())
    steady = next(s for s in doc['segments'] if s['kind'] == 'steady')
    assert steady['quality'] == 'suspect' and steady['flags'] == ['wake']       # the wake span reaches back 5 s
    neutral = next(s for s in doc['segments'] if s['kind'] == 'neutral')
    assert neutral['quality'] == 'ok' and neutral['conditions'] == {'wind': 2, 'waves': 1, 'current': 1}
    assert [c['kind'] for c in doc['calibration']] == ['neutral']               # the steady one is suspect
    assert doc['commands'][0]['text'] == 'hold position here' and doc['commands'][0]['pose']['x'] == 1.0
    assert doc['pending'] == 5 and doc['profile'] == 'boat'


def test_review_edits_are_lines_and_undo(tmp_path):
    events = journal() + [
        ev(300, 'label_edit', target='mark:1', action='reject'),
        ev(301, 'label_edit', target='procedure:120000', action='split', value=135.0),
        ev(302, 'label_edit', target='said:1', action='outcome', value='held'),
        ev(303, 'label_edit', target='said:1', action='accept'),
        ev(304, 'label_edit', target='said:1', action='reject'),
        ev(305, 'label_edit', action='undo'),
    ]
    doc = manifest.build(tmp_path, events)
    steady = next(s for s in doc['segments'] if s['kind'] == 'steady')
    assert steady['quality'] == 'ok'                                  # the rejected wake no longer downgrades
    assert len(doc['calibration']) == 2
    assert [(s['t0'], s['t1']) for s in doc['segments'] if s['kind'] == 'procedure'] == [(120, 135), (135, 160)]
    assert doc['commands'][0]['status'] == 'accepted' and doc['commands'][0]['outcome'] == 'held'


def test_write_and_load_refuses_another_major(tmp_path):
    for e in journal():
        data.append_jsonl(tmp_path / 'events.jsonl', e)
    manifest.write(tmp_path)
    doc = manifest.load(tmp_path)
    assert manifest.overlaps(doc, 172, 173) == ['bad'] and manifest.overlaps(doc, 102, 105) == []
    assert all(s['kind'] == 'neutral' for s in manifest.segments(doc, kinds=('steady', 'neutral')))
    (tmp_path / 'manifest.json').write_text(json.dumps({'schema': 'mhseals.session_manifest/2'}))
    with pytest.raises(ValueError):
        manifest.load(tmp_path)
