from mhseals_testing import calibration as cal
from mhseals_testing.history import History

HW = {'thruster_matrix': [[1, 0], [0, 1]], 'reversed': [], 'hardware_epoch': 3, 'channel_map': [1, 2], 'pwm_channels': [0, 1]}
FIT = {'id': 'f1', 'time': 't', 'session': 's', 'run': 'r', 'config': 'c0131'}


def _record(tmp_path, **extra):
    path = tmp_path / 'calibration' / 'current.json'
    cal.record('sway-', FIT, HW, extra=extra, path=path)
    return path


def test_scope_classes():
    assert cal.scope_of('thruster_matrix') == ('hardware', None)
    assert cal.scope_of('BRD_TYPE') == ('hardware', None)
    assert cal.scope_of('decoupling') == ('calibration', 'run')
    assert cal.scope_of('COMPASS_OFS_X') == ('calibration', 'site')
    assert cal.scope_of('INS_ACC2OFFS_X') == ('calibration', 'build')
    assert cal.scope_of('kp') == ('setting', None)
    assert cal.scope_of('INS_GYROFFS_X') == ('ignored', None)


def test_new_classes_and_yaw_deps():
    assert cal.scope_of('rtk.device') == ('hardware', None)
    assert cal.scope_of('gps_link') == ('hardware', None)
    assert cal.scope_of('rtk.positions') == ('calibration', 'site')
    assert cal.scope_of('nav2.vx_max') == ('calibration', 'derived')
    assert cal.scope_of('EK3_SRC1_YAW') == ('setting', None) and cal.is_yaw_dep('COMPASS_USE2')
    assert not cal.is_yaw_dep('kp')


def test_absent_epoch_hashes_as_zero_and_ids_compare_as_integers(tmp_path):
    bare = {k: v for k, v in HW.items() if k != 'hardware_epoch'}
    path = tmp_path / 'c.json'
    cal.record('sway-', {**FIT, 'config': 'c999'}, bare, path=path)
    entry = cal.load(path)['directions']['sway-']
    assert cal.status(entry, {**bare, 'hardware_epoch': 0})[0] == cal.OK
    class Log:
        def changes(self, subsystem):
            return [{'id': 'c0100'}]
    entry['fit']['config'] = 'c99'
    assert 'c0100' in cal.status(entry, {**bare, 'hardware_epoch': 1}, Log())[1]


def test_record_keeps_other_directions_and_provenance(tmp_path):
    path = _record(tmp_path, battery_v=15.8)
    cal.record('sway+', FIT, HW, path=path)
    stored = cal.load(path)
    assert stored['schema'] == cal.SCHEMA and set(stored['directions']) == {'sway-', 'sway+'}
    assert stored['directions']['sway-']['hardware']['epoch'] == 3
    assert stored['directions']['sway-']['fit']['config'] == 'c0131'


def test_status_ok_and_settings_or_key_order_do_not_matter(tmp_path):
    entry = cal.load(_record(tmp_path))['directions']['sway-']
    assert cal.status(entry, HW) == (cal.OK, '')
    shuffled = dict(reversed(list({**HW, 'kp': 9}.items())))
    assert cal.status(entry, shuffled)[0] == cal.OK


def test_epoch_bump_and_matrix_change_invalidate_with_change_id(tmp_path):
    entry = cal.load(_record(tmp_path))['directions']['sway-']
    history = History(tmp_path / 'h')
    history.record('hardware', {'a': 1}, source='t')
    assert cal.status(entry, {**HW, 'hardware_epoch': 4})[0] == cal.INVALID
    state, reason = cal.status(entry, {**HW, 'thruster_matrix': [[0, 1], [1, 0]]}, history)
    assert state == cal.INVALID and 'hardware changed' in reason


def test_verify_battery_and_muted_gate(tmp_path):
    entry = cal.load(_record(tmp_path, battery_v=15.8, gate={'muted': False}))['directions']['sway-']
    assert cal.status(entry, {**HW, 'channel_map': [2, 1]})[0] == cal.VERIFY
    assert cal.status(entry, HW, battery_v=14.9)[0] == cal.VERIFY
    assert cal.status(entry, HW, battery_v=15.5)[0] == cal.OK
    muted = cal.load(_record(tmp_path, gate={'muted': True}))['directions']['sway-']
    assert cal.status(muted, HW)[0] == cal.VERIFY
    assert cal.status(entry, {**HW, 'hardware_epoch': 9, 'channel_map': [2, 1]})[0] == cal.INVALID    # worst wins


def test_invalidation_never_deletes_and_status_is_computed_at_display_time(tmp_path):
    path = _record(tmp_path)
    before = path.read_text()
    changed = {**HW, 'hardware_epoch': 4}
    current = cal.load(path)
    assert cal.invalid_count(changed, current=current) == 1
    assert cal.snapshot(changed, current=current)['entries']['sway-']['state'] == cal.INVALID
    assert path.read_text() == before                              # nothing written or removed
    assert 'status' not in cal.load(path)                          # the snapshot is derived, not stored
    assert cal.invalid_count(HW, current=current) == 0             # reverting the hardware clears it again


def test_record_fit_marks_and_snapshot(tmp_path):
    path = tmp_path / 'calibration' / 'current.json'
    fit = cal.new_fit()
    cal.record_fit(['sway-', 'yaw+'], fit, HW, path=path,
                   context={'session': 's', 'battery_v': 15.8, 'muted': True, 'config': 'c1'}, conditions={'verdict': 'good'})
    entry = cal.load(path)['directions']['yaw+']
    assert entry['fit']['id'] == fit['id'] and entry['fit']['config'] == 'c1' and entry['battery_v'] == 15.8
    assert entry['gate'] == {'muted': True} and entry['conditions'] == {'verdict': 'good'}
    assert cal.marks(HW, current=cal.load(path)) == {'sway-': '?', 'yaw+': '?'}      # muted gate: verify
    changed = {**HW, 'hardware_epoch': 4}
    assert cal.marks(changed, current=cal.load(path)) == {'sway-': '✗', 'yaw+': '✗'}
    assert cal.invalid_count(changed, current=cal.load(path)) == 2
    assert cal.write_snapshot(changed, path=path)
    assert cal.load(path)['status']['entries']['sway-']['state'] == cal.INVALID
    assert not cal.write_snapshot(HW, path=tmp_path / 'none.json')


def test_invalid_fact_and_check(tmp_path):
    from mhseals_testing.health.probes import invalid_facts
    path = tmp_path / 'c.json'
    cal.record_fit(['sway-'], cal.new_fit(), HW, path=path)
    current = cal.load(path)
    assert invalid_facts(current, HW)['calibration.invalid'].value == 0
    fact = invalid_facts(current, {**HW, 'hardware_epoch': 9})['calibration.invalid']
    assert (fact.value, fact.label) == (1, 'sway-')
    assert invalid_facts({}, HW) == {}
    from mhseals_testing.health.config import load
    spec = next(s for s in load(None, None).specs if s.id == 'calib.invalid')
    assert set(spec.gates) == {'w', 's', 't'}
