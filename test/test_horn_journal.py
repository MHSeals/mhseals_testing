import json

from mhseals_testing.harness.horn import TOPIC, HornJournal, program


def truth(tmp_path, pattern='two'):
    path = tmp_path / 'course_truth.json'
    path.write_text(json.dumps({'states': [
        {'id': 'light_1', 'program': {'type': 'toggle'}},
        {'id': 'harbor_alert.horn', 'program': {'type': 'horn', 'pattern': pattern, 'freq_hz': 600.0,
                                                'repeats': 1, 'trigger_s': 30.0, 'gap_s': 2.0}}]}))
    return path


def test_each_horn_edge_is_journaled_with_the_courses_pattern(tmp_path):
    seen = []
    hear = HornJournal(lambda phase, **f: seen.append((phase, f)), truth(tmp_path))
    for on in (False, True, True, False, True, False):   # crane: off at start, then two blasts
        hear(json.dumps({'on': on, 'freq_hz': 600 if on else 0}))
    assert [f['on'] for _, f in seen] == [True, False, True, False]
    assert all(phase == 'horn' for phase, _ in seen)
    assert seen[0][1] == dict(on=True, freq_hz=600.0, pattern='two', id='harbor_alert.horn', topic=TOPIC)
    assert TOPIC == '/crane/sim/horn'


def test_one_blast_pattern_and_unreadable_inputs(tmp_path):
    seen = []
    hear = HornJournal(lambda phase, **f: seen.append(f), truth(tmp_path, 'one'))
    hear('not json')
    hear(json.dumps({'freq_hz': 600}))
    hear(json.dumps({'on': True, 'freq_hz': 600}))
    assert [(f['on'], f['pattern']) for f in seen] == [(True, 'one')]


def test_without_a_course_horn_the_pattern_is_unknown(tmp_path):
    assert program(tmp_path / 'missing.json') == (None, None)
    seen = []
    HornJournal(lambda phase, **f: seen.append(f))(json.dumps({'on': True, 'freq_hz': 600}))
    assert seen[0]['pattern'] is None and seen[0]['id'] is None
