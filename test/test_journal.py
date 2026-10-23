import json

from mhseals_testing import events
from mhseals_testing.harness import journal
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.report import build_report


def event(phase, **details):
    return events.payload(phase, 1.0, details)


def test_marker_note_folds_to_the_latest_note():
    log = [event('marker', kind='user', id=1, note=''), event('marker', kind='user', id=2, note=''),
           event('marker_note', id=1, note='first'), event('marker_note', id=1, note='edited'),
           event('marker_note', id=9, note='orphan')]
    folded = journal.markers(log)
    assert [(m['id'], m['note']) for m in folded] == [(1, 'edited'), (2, '')]
    assert journal.next_marker_id(log) == 3 and journal.next_marker_id([]) == 1


def test_torn_last_line_and_garbage_are_skipped(tmp_path):
    path = tmp_path / 'events.jsonl'
    journal.append(path, event('a'))
    journal.append(path, event('b'))
    with open(path, 'a') as raw:
        raw.write('[1]\n{"phase": "c", "t_ns": 12')  # not an object, then torn
    assert [e['phase'] for e in journal.read(path)] == ['a', 'b']
    assert journal.read(tmp_path / 'missing.jsonl') == []


def test_every_line_is_stamped_and_an_oversized_note_stays_one_line(tmp_path):
    path = tmp_path / 'events.jsonl'
    journal.append(path, event('marker_note', id=1, note='x' * 9000))
    assert len(path.read_bytes()) <= journal.MAX_LINE + 1
    [only] = journal.read(path)
    assert isinstance(only['t_ns'], int) and only['rec'] is True


def test_report_takes_all_journal_events_not_the_last_twelve(tmp_path):
    log = [event('step', n=n) for n in range(30)] + [event('marker', kind='user', id=1, note='')]
    report = build_report(load_profile('boat'), BoatMonitor(clock=lambda: 2).snapshot(),
                          journal_events=log)
    assert len(report['events']) == 31 > 12
    assert all(isinstance(e['t_ns'], int) for e in report['events'])
    assert [m['id'] for m in report['markers']] == [1]
    json.dumps(report)
