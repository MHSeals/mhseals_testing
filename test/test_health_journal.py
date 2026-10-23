"""Journal: transitions, snapshots and trial-window validity."""

from mhseals_testing.health.engine import HealthReport, Transition
from mhseals_testing.health.journal import Journal
from mhseals_testing.health.model import CheckSpec, Result


class FakeEngine:
    muted = set()
    specs = [CheckSpec(id='compass', kind='k', params=None, gates=('characterize',), title='Compass'),
             CheckSpec(id='gps', kind='k', params=None, gates=('nav',), title='GPS')]


def change(journal, check_id, old, new, at):
    journal.record(HealthReport({}, (Transition(check_id, old, new, Result(new), at),)))


def test_window_is_invalid_when_a_gated_check_left_ok_inside_it():
    journal = Journal()
    change(journal, 'compass', 'ok', 'warn', 12)
    change(journal, 'compass', 'warn', 'ok', 15)
    change(journal, 'gps', 'ok', 'fail', 12)
    assert journal.window_problems(FakeEngine, 'characterize', 10, 20) == ['Compass was warn']
    assert journal.window_problems(FakeEngine, 'characterize', 16, 20) == []
    assert journal.window_problems(FakeEngine, 'characterize', 0, 11) == []


def test_a_check_already_bad_at_the_start_invalidates_the_window():
    journal = Journal()
    change(journal, 'compass', 'ok', 'fail', 5)
    assert journal.window_problems(FakeEngine, 'characterize', 10, 20) == ['Compass was fail']


def test_muted_checks_do_not_invalidate():
    journal = Journal()
    change(journal, 'compass', 'ok', 'fail', 5)
    engine = type('E', (), {'specs': FakeEngine.specs, 'muted': {'compass'}})
    assert journal.window_problems(engine, 'characterize', 10, 20) == []


def test_snapshot_counts_and_lists_only_bad_checks():
    entry = Journal().snapshot('end', {'a': Result('ok'), 'b': Result('warn'), 'c': Result('fail')}, 7.0)
    assert entry == {'label': 'end', 'at': 7.0, 'counts': {'ok': 1, 'warn': 1, 'fail': 1},
                     'bad': {'b': 'warn', 'c': 'fail'}}
