import copy

import pytest
import yaml

from mhseals_testing.profiles import PROFILE_DIR, available_profiles, load_profile, parse_profile


def base():
    return yaml.safe_load((PROFILE_DIR / 'boat.yaml').read_text())


def test_shipped_profiles_load():
    assert {'boat', 'crane_direct', 'crane_full_stack'} <= set(available_profiles())
    for name in available_profiles():
        assert load_profile(name).name == name


def test_frames_drive_required_tf():
    assert load_profile('boat').required_tf == (('map', 'odom'), ('odom', 'base_link'))
    assert load_profile('crane_direct').required_tf == (('odom', 'base_link'),)


def test_velocity_control_needs_outputs():
    boat = load_profile('boat').with_velocity_control()
    assert boat.command_units == 'velocity' and boat.velocity_controller
    with pytest.raises(ValueError):
        load_profile('crane_direct').with_velocity_control()


def test_profile_path_and_unknown_name(tmp_path):
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(dict(base(), name='custom')))
    assert load_profile(str(path)).name == 'custom'
    with pytest.raises(FileNotFoundError, match='shipped'):
        load_profile('nope')


@pytest.mark.parametrize('edit,message', [
    (lambda d: d.pop('odometry'), 'missing odometry'),
    (lambda d: d['command'].update(type='joy'), 'command.type'),
    (lambda d: d['command'].update(units='rpm'), 'command.units'),
    (lambda d: d['command'].update(limit=[1, 1]), 'command.limit'),
    (lambda d: d['arming'].update(outputs='gpio'), 'arming.outputs'),
    (lambda d: d['arming'].update(velocity_controller=True), 'velocity_controller'),
    (lambda d: d['sensors'][0].update(kind='sonar'), 'kind'),
    (lambda d: d['sensors'][0].update(hz=0), r'sensors\[0\]\.hz'),
    (lambda d: d['sensors'][0].update(stale_after='soon'), 'stale_after'),
    (lambda d: d['sensors'].append(dict(d['sensors'][0])), 'unique'),
    (lambda d: d.update(components='yes'), 'components'),
    (lambda d: d.update(components={'a': {'nodes': []}}), 'launch or remote'),
    (lambda d: d.update(components={'a': {'launch': ['p', 'f'], 'needs': ['b']}}), 'unknown component'),
])
def test_invalid_profiles_name_the_field(edit, message):
    data = copy.deepcopy(base())
    edit(data)
    with pytest.raises(ValueError, match=message):
        parse_profile(data)


def test_boat_gps_declares_its_rate():
    gps = next(s for s in load_profile('boat').sensors if s.name == 'GPS')
    assert gps.hz == 5 and gps.deadline >= 1.13  # real gaps reach 1.13 s
