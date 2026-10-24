"""File subsystems feed the config history: writers record, hand edits are external."""

import yaml

from mhseals_testing import rtk
from mhseals_testing.history import History, adapters


def test_hand_edit_becomes_external_and_writer_keeps_its_source(tmp_path):
    path, history = tmp_path / 'health.yaml', History(tmp_path / 'h')
    path.write_text('# note\nwarn: 1\n')
    assert adapters.observe_file(history, 'health', path)['source'] == 'external'
    assert adapters.observe_file(history, 'health', path) is None          # unchanged
    path.write_text('# note\nwarn: 2\n')
    adapters.observe_file(history, 'health', path)                         # hand edit
    path.write_text('# note\nwarn: 3\n')
    change = adapters.record_file(history, 'health', 'tui:test', path)
    assert [c['source'] for c in history.changes('health')] == ['external', 'external', 'tui:test']
    assert history.version(change)['raw'] == '# note\nwarn: 3\n'           # comments survive


def test_broken_or_missing_file_is_skipped_and_failures_warn(tmp_path):
    history, seen = History(tmp_path / 'h'), []
    assert adapters.observe_file(history, 'keymap', tmp_path / 'none.yaml') is None
    (tmp_path / 'k.yaml').write_text('a: [\n')
    assert adapters.observe_file(history, 'keymap', tmp_path / 'k.yaml') is None
    (tmp_path / 'k.yaml').write_text('a: 1\n')
    history.dir.parent.mkdir(exist_ok=True)
    history.dir.write_text('not a directory')                              # the store cannot be written
    assert adapters.observe_file(history, 'keymap', tmp_path / 'k.yaml', seen.append) is None
    assert seen and 'keymap' in seen[0]


def test_observe_all_names_the_given_paths(tmp_path):
    history = History(tmp_path / 'h')
    (tmp_path / 'r.yaml').write_text('device: x\n')
    changes = adapters.observe_all(history, {s: tmp_path / 'absent' for s in adapters.SUBSYSTEMS} | {'rtk': tmp_path / 'r.yaml'})
    assert [c['subsystem'] for c in changes] == ['rtk']


def test_rtk_save_records_and_atomic_write_is_used(tmp_path, monkeypatch):
    path = tmp_path / 'rtk.yaml'
    monkeypatch.setenv('MHSEALS_RTK_CONFIG', str(path))
    path.write_text('device: hand\n')                                      # edited by hand before the save
    rtk.save_config(dict(rtk.DEFAULTS, device='/dev/ttyACM0'))
    changes = History().changes('rtk')
    assert [c['source'] for c in changes] == ['external', 'tui:rtk']
    assert yaml.safe_load(path.read_text())['device'] == '/dev/ttyACM0'
    assert not list(tmp_path.glob('*.tmp')) and not list(tmp_path.glob('.*.tmp'))


def test_hardware_version_carries_the_defaults_hash_and_never_the_arming_copy(tmp_path):
    path, history = tmp_path / 'hardware.yaml', History(tmp_path / 'h')
    path.write_text('channel_map: [3, 1, 2, 0]\n')
    change = adapters.observe_file(history, 'hardware', path)
    from hashlib import sha256
    from mhseals_hardware.resources import config_resource
    expected = sha256(config_resource('default.yaml').read_text().encode()).hexdigest()[:12]
    assert history.version(change)['meta']['defaults_sha'] == expected
    assert 'hardware' in adapters.SUBSYSTEMS and 'session' not in adapters.SUBSYSTEMS   # arming copy has no subsystem
