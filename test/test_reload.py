"""Hot-reload classes: plant fields wait for disarm, topic changes start a new segment."""

from mhseals_testing.harness import reload
from mhseals_testing.harness.reload import Reload, Segments, classify, load_rules, rtk_restart

RULES = load_rules()
PROFILE = {'name': 'boat', 'command': {'limit': [1, 1, 1]}, 'sensors': [{'topic': '/a'}]}


def edited(**changes):
    return {**PROFILE, **changes}


def test_profile_fields_fall_into_their_classes():
    changes = classify(RULES, 'profiles', PROFILE,
                       edited(sensors=[], bringup={'fcu': 'true'}, command={'limit': [2, 1, 1]}))
    assert changes == {'live': ['sensors'], 'next': ['bringup.fcu'], 'disarmed': ['command.limit']}


def test_a_plant_field_is_refused_while_armed_and_applies_after_disarm():
    state = Reload(RULES)
    now = state.apply('profiles', PROFILE, edited(use_sim_time=True, name='x'), armed=True)
    assert now == {'live': ['name']}
    assert state.note('profiles') == '↻ 1 fields pending (disarm)'
    assert state.disarmed('profiles') == ['use_sim_time']
    assert state.note('profiles') == ''
    assert state.apply('profiles', PROFILE, edited(use_sim_time=True), armed=False) == {
        'disarmed': ['use_sim_time']}


def test_settings_and_missions_are_next_use_even_when_armed():
    state = Reload(RULES)
    for surface in ('settings', 'missions'):
        assert state.apply(surface, {'a': 1}, {'a': 2}, armed=True) == {'next': ['a']}
        assert state.note(surface) == ''


def test_rtk_positions_are_live_but_the_device_needs_a_base_restart():
    state = Reload(RULES)
    old = {'device': '/dev/a', 'positions': {'dock': [1, 2, 3]}, 'system_id': 1}
    new = {'device': '/dev/b', 'positions': {'dock': [1, 2, 4]}, 'system_id': 2}
    assert state.apply('rtk', old, new) == {'live': ['positions.dock', 'system_id']}
    assert state.note('rtk') == '↻ base restart needed'

    class Base:
        def __init__(self):
            self.calls = []
        def close(self):
            self.calls.append('close')
        def start(self, **kw):
            self.calls.append(kw)

    base, events = Base(), []
    assert rtk_restart(state, base, {'dock': [1, 2, 4]}, lambda *a, **k: events.append((a, k)))
    assert base.calls == ['close', {'survey': False, 'position': [1, 2, 4]}]
    assert events[0][0] == ('rtk_restart',) and state.note('rtk') == ''
    assert not rtk_restart(state, base, {}, lambda *a, **k: None)


def test_a_topic_change_waits_then_enter_starts_a_new_segment_and_logs_it():
    started, events = [], []
    row = Segments(started.append, lambda *a, **k: events.append((a, k)))
    row.recording(['/a', '/b'])
    row.changed(['/b', '/a'])  # reordered: nothing to do
    assert row.note == '' and not row.enter()
    row.changed(['/a', '/c'])
    assert row.note == '↻ topics changed' and started == []
    assert row.enter()
    assert started == [('/a', '/c')]
    assert events == [(('segment_restart',), {'added': ['/c'], 'removed': ['/b']})]
    assert row.note == '' and not row.enter()


def test_install_symlinked_names_the_copies(tmp_path):
    source, share = tmp_path / 'src', tmp_path / 'share'
    source.mkdir(), share.mkdir()
    (source / 'a.yaml').write_text('a')
    (share / 'a.yaml').symlink_to(source / 'a.yaml')
    assert reload.copies(source, share) == []
    (share / 'b.yaml').write_text('copy')
    assert reload.copies(source, share) == ['b.yaml']
    assert reload.copies(source, tmp_path / 'missing') is None
