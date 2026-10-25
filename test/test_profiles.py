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


def test_relative_course_truth_resolves_under_the_data_root(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    assert load_profile('crane_full_stack').course_truth == str(tmp_path / 'course_truth.json')


def test_stamp_future_defaults_to_the_boats_tolerance_and_is_wider_for_the_sim():
    assert load_profile('boat').stamp_future == 0.25
    assert load_profile('crane_full_stack').stamp_future > 0.25


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
    (lambda d: d.update(components={'a': {'args': {}}}), 'come from the device profile'),
    (lambda d: d.update(components={'a': {'nodes': [1]}}), 'a.nodes: list of node names'),
    (lambda d: d.update(components={'a': {'needs': ['b']}}), 'unknown component'),
    (lambda d: d.update(placement={'self': 'sim'}), 'placement'),
    (lambda d: d.update(placement={'other': ['sim']}), 'placement'),
])
def test_invalid_profiles_name_the_field(edit, message):
    data = copy.deepcopy(base())
    edit(data)
    with pytest.raises(ValueError, match=message):
        parse_profile(data)


def test_boat_gps_declares_its_rate():
    gps = next(s for s in load_profile('boat').sensors if s.name == 'GPS')
    assert gps.hz == 10 and gps.deadline >= 1.13  # Here4 10 Hz; real gaps reached 1.13 s at 5 Hz


def test_boat_proof_nodes_are_node_names():
    """Proof nodes are ROS node names (launch name=), not executables. The Orin's velodyne driver
    runs on the large-data transport no other computer discovers, so the relay is the proof."""
    parts = load_profile('boat').placed[1]
    assert tuple(parts['sensors'].spec['nodes']) == ('lidar_relay_publish',)


def test_boat_lidar_counts_as_running_on_its_relay():
    """The driver is on the large-data transport, invisible to other computers; /points's
    default-transport publisher, lidar_relay_publish, proves the sensors part."""
    from mhseals_testing.harness.components import components
    [sensors] = [c for c in components(load_profile('boat')) if c.key == 'sensors']
    assert sensors.nodes == ('lidar_relay_publish',)


@pytest.mark.parametrize('choice', [None, True, 4, 'odom:', 'odom:/missing', 'unknown', []])
def test_invalid_profile_selector(choice):
    data = yaml.safe_load((PROFILE_DIR / 'crane_full_stack.yaml').read_text())
    data['conditions'] = {'velocity': choice}
    with pytest.raises(ValueError, match='conditions.velocity'):
        parse_profile(data)


def test_profile_defaults_and_sim_only():
    assert load_profile('boat').conditions_velocity == 'gpsraw'
    sim = load_profile('crane_full_stack')
    assert sim.conditions_velocity == 'odom:/odom/mavros'
    data = yaml.safe_load((PROFILE_DIR / 'boat.yaml').read_text())
    data['conditions'] = {'velocity': 'odom:odometry'}
    with pytest.raises(ValueError, match='conditions.velocity'):
        parse_profile(data)
