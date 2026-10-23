"""Enter routing per timeline item kind, the run popup rows and X exclusion (TS107)."""
import pytest

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.ui.popups import run


@pytest.mark.parametrize('kind, owner', [
    ('procedure', 'run'), ('manual', 'run'), ('config', 'history'), ('health', 'health'),
    ('marker', 'marker'), ('hold', 'show'), ('cluster', 'choose'), ('unknown', 'show')])
def test_routing_table_per_item_kind(kind, owner):
    assert run.route(dict(kind=kind)) == owner


def test_space_passes_through_the_popup():
    assert 'space' in run.PASSTHROUGH


def test_exclusion_needs_a_reason():
    with pytest.raises(ValueError):
        run.exclusion('  ')
    assert run.exclusion(' bad tide ') == dict(reason='bad tide', by='tui')


def test_x_writes_excluded_on_the_stored_result_and_rows_show_it():
    monitor = BoatMonitor(clock=lambda: 0)
    monitor.record_result(dict(procedure='axis', verdict='SIGN MATCH'))
    monitor.record_result(dict(procedure='step', verdict='OK'))
    monitor.exclude_result(0, run.exclusion('wrong conditions'))
    first, second = monitor.snapshot()['results']
    assert first['excluded'] == dict(reason='wrong conditions', by='tui')
    assert 'excluded' not in second
    assert ('excluded', 'wrong conditions (tui)') in run.run_rows(first)
