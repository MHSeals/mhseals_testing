import pytest

from mhseals_testing import data
from mhseals_testing.harness import links
from mhseals_testing.history import History


@pytest.fixture
def hist(tmp_path):
    h = History(tmp_path / 'h')
    h.record('hardware', {'a': 1}, source='t')
    h.record('nav2', {'v': 1}, source='t')
    return h


def test_session_block_snapshots_and_mid_session_change(hist, tmp_path):
    cfg = links.session_config(hist)
    assert cfg == {'start': {'hardware': 'c0001', 'nav2': 'c0002', 'fcu': None, 'rtk': None}, 'changes': []}
    snaps = tmp_path / 'snapshots'
    assert len(links.snapshot_start(hist, cfg, snaps)) == 2
    change = hist.record('hardware', {'a': 2}, source='tui', summary=['a 1->2'])
    event = links.add_change(cfg, hist, change, snaps)
    assert event == {'type': 'config_changed', 'id': 'c0003', 'subsystem': 'hardware', 'summary': ['a 1->2']}
    assert cfg['changes'] == ['c0003'] and (snaps / 'hardware-c0003.json').exists()
    links.add_change(cfg, hist, change, snaps)
    assert cfg['changes'] == ['c0003']
    assert data.read_json(snaps / 'hardware-c0003.json')['value'] == {'a': 2}


def test_run_config_and_index_counts(hist):
    first = links.run_config(hist)
    hist.record('hardware', {'a': 2}, source='t')
    second = links.run_config(hist, staged='c0009')
    assert second['staged'] == 'c0009' and 'staged' not in first
    assert links.config_runs([first, first, second]) == {
        'hardware:c0001': 2, 'nav2:c0002': 3, 'hardware:c0003': 1}


def test_pooling_refused_without_pool_flag():
    a = {'run_id': 'r1', 'config': {'hardware': 'c1', 'fcu': 'c5'}}
    b = {'run_id': 'r2', 'config': {'hardware': 'c1', 'fcu': 'c5', 'nav2': 'c9'}}
    c = {'run_id': 'r3', 'config': {'hardware': 'c2', 'fcu': 'c5'}}
    assert links.check_pool([a, b]) == {'pooled': False, 'groups': 1}
    with pytest.raises(links.PoolRefused, match='hardware c2'):
        links.check_pool([a, b, c])
    assert links.check_pool([a, c], pool=True) == {'pooled': True, 'groups': 2}
