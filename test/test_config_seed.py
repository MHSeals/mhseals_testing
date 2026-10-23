import os

from mhseals_testing.history import History
from mhseals_testing.history.seed import FCU_NAME, read_param_file, seed


def _param(tmp_path):
    f = tmp_path / 'blastoise.param'
    f.write_text('# header\nEK3_SRC1_YAW,8\nCOMPASS_USE,0\nCOMPASS_ORIENT         0.000000\nSTAT_RUNTIME,123\nBARO1_GND_PRESS 101325\n')
    return f


def _snapshots(tmp_path):
    folder = tmp_path / 'bags'
    folder.mkdir()
    for n, (name, gain) in enumerate((('b.hardware.yaml', 2), ('a.hardware.yaml', 1))):
        f = folder / name
        f.write_text(f'gain: {gain}\n')
        os.utime(f, (1_700_000_000 + n * 100,) * 2)       # b is older than a
    (folder / 'bad.hardware.yaml').write_text('- not a mapping\n')
    return folder


def test_param_file_drops_ignored_and_comments(tmp_path):
    assert read_param_file(_param(tmp_path)) == {'EK3_SRC1_YAW': 8.0, 'COMPASS_USE': 0.0, 'COMPASS_ORIENT': 0.0}


def test_seed_observes_names_fcu_and_is_idempotent(tmp_path):
    h = History(tmp_path / 'h')
    readers = {'hardware': lambda: ({'overrides': {'gain': 9}}, 'gain: 9\n', None), 'nav2': lambda: None}
    first = seed(h, readers, fcu_param=_param(tmp_path))
    assert [c['subsystem'] for c in first] == ['fcu', 'hardware']
    fcu = h.head('fcu')
    assert (fcu['name'], fcu['source']) == (FCU_NAME, 'seed') and 'COMPASS_USE' in fcu['note']
    assert seed(h, readers, fcu_param=_param(tmp_path)) == []
    assert len(h.changes()) == 2


def test_hardware_import_is_dated_ordered_and_leaves_current_as_head(tmp_path):
    h = History(tmp_path / 'h')
    readers = {'hardware': lambda: ({'overrides': {'gain': 9}}, 'gain: 9\n', None)}
    snaps = _snapshots(tmp_path)
    seed(h, readers, hardware_snapshots=snaps)
    seed(h, readers, hardware_snapshots=snaps)
    changes = h.changes('hardware')
    assert [c['source'] for c in changes] == ['import', 'import', 'external']
    assert [h.version(c)['value']['overrides']['gain'] for c in changes] == [2, 1, 9]
    assert changes[0]['time'] < changes[1]['time'] and changes[0]['applied'] == 'observed'
    assert h.version(changes[0])['raw'] == 'gain: 2\n'
