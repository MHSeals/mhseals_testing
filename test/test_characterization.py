import json
import math

import numpy
import pytest

from pytest import approx

from mhseals_testing.analysis.characterization import (
    audit_holds, characterize_samples, nav2_profile, read_json_samples)


def samples():
    rows = []
    commands = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1), (1, 1, 0), (-1, 1, -1))
    for stamp, command in enumerate(commands):
        rows.append({'time': stamp, 'command': command,
                     'velocity': (0.6 * command[0], 0.4 * command[1],
                                  0.3 * command[2])})
    return rows


def test_fits_gain_and_limits():
    result = characterize_samples(samples())
    assert result['sample_count'] == 8
    assert result['velocity_gain']['surge']['surge'] == approx(0.6)
    assert result['motion_limits']['velocity']['sway'] == approx(0.4)
    assert len(result['decoupling']) == 9 and result['drift'] == {'separated': False}
    assert result['motion_limits']['combined_directions']


def test_json_samples_accepts_sample_wrapper(tmp_path):
    path = tmp_path / 'samples.json'
    path.write_text(json.dumps({'samples': samples()}))
    assert len(read_json_samples(path)) == 8


def test_underexcited_bag_is_rejected():
    with pytest.raises(ValueError, match='excite'):
        characterize_samples([
            {'time': i, 'command': (1, 0, 0), 'velocity': (i * .1, 0, 0)}
            for i in range(8)])


def test_nav2_profile_applies_measured_limits_to_mppi(tmp_path):
    source = tmp_path / 'nav2.yaml'
    source.write_text('''
controller_server:
  ros__parameters:
    FollowPath:
      vx_max: 1.0
      wz_max: 1.0
velocity_smoother:
  ros__parameters:
    max_velocity: [1.0, 1.0, 1.0]
    min_velocity: [-1.0, -1.0, -1.0]
    max_accel: [1.0, 1.0, 1.0]
    max_decel: [-1.0, -1.0, -1.0]
behavior_server:
  ros__parameters:
    max_rotational_vel: 1.0
    rotational_acc_lim: 1.0
''')
    result = characterize_samples(samples())
    profile = nav2_profile(result, source)
    follow = profile['controller_server']['ros__parameters']['FollowPath']
    behavior = profile['behavior_server']['ros__parameters']
    assert (follow['vx_max'], follow['vx_min']) == approx((.6, -.2))
    assert (follow['vy_max'], follow['wz_max']) == approx((.4, .3))
    assert behavior['max_rotational_vel'] == approx(.3)
    assert behavior['rotational_acc_lim'] == approx(.3)


def test_samples_from_windows_skips_transients_and_pairs_commands():
    from mhseals_testing.analysis.characterization import samples_from_windows
    history = [(t / 10, {'vx': t / 10, 'vy': 0.0, 'wz': 0.0}) for t in range(50)]
    windows = [((0, 0, 0), 0.0, 1.9), ((0.5, 0, 0), 2.0, 4.0)]
    samples = samples_from_windows(history, windows, transient=1.0)
    assert {s['command'] for s in samples} == {(0, 0, 0), (0.5, 0, 0)}
    assert min(s['time'] for s in samples if s['command'][0]) == pytest.approx(3.0)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_samples_from_windows_drops_a_non_finite_sample(bad):
    from mhseals_testing.analysis.characterization import samples_from_windows
    clean = [(t / 10, {'vx': 0.5, 'vy': 0.0, 'wz': 0.0}) for t in range(50)]
    dirty = list(clean)
    dirty[30] = (3.0, {'vx': bad, 'vy': 0.0, 'wz': 0.0})
    windows = [((0.5, 0, 0), 0.0, 4.0)]
    kept = samples_from_windows(dirty, windows)
    assert len(kept) == len(samples_from_windows(clean, windows)) - 1
    assert all(math.isfinite(v) for s in kept for v in s['velocity'])


def test_samples_from_windows_skips_a_non_finite_command():
    from mhseals_testing.analysis.characterization import samples_from_windows
    history = [(t / 10, {'vx': 0.5, 'vy': 0.0, 'wz': 0.0}) for t in range(50)]
    assert samples_from_windows(history, [((math.nan, 0, 0), 0.0, 4.0)]) == []


GAIN = numpy.array(((0.6, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.3)))
BIAS = numpy.array((0.02, -0.01, 0.0))


def boat_samples(current, yaws, gain=GAIN, decoupling=numpy.eye(3)):
    """Steady samples of v = b_m + R(yaw)^T d + G C e over the given headings."""
    commands = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
                (0, 0, 0), (.5, .5, 0)]
    rows = []
    for i, yaw in enumerate(yaws):
        command = numpy.array(commands[i % len(commands)], float)
        velocity = BIAS + gain @ decoupling @ command
        c, s = math.cos(yaw), math.sin(yaw)
        velocity[:2] += (c * current[0] + s * current[1], -s * current[0] + c * current[1])
        rows.append({'time': i * .1, 'command': command.tolist(),
                     'velocity': velocity.tolist(), 'yaw': yaw})
    return rows


def test_fit_separates_a_world_current_from_the_boat():
    yaws = [2 * math.pi * i / 64 for i in range(64)]
    result = characterize_samples(boat_samples((0.1, 0.15), yaws))
    drift = result['drift']
    assert drift['separated'] and (drift['x'], drift['y']) == approx((0.1, 0.15), abs=1e-6)
    assert [result['mechanical_bias'][a] for a in ('surge', 'sway', 'yaw')] == approx(BIAS, abs=1e-6)
    assert result['velocity_gain']['yaw']['surge'] == approx(0.05)


def test_one_sample_without_a_heading_does_not_switch_off_drift_separation():
    yaws = [2 * math.pi * i / 64 for i in range(64)]
    samples = boat_samples((0.1, 0.15), yaws)
    clean = characterize_samples(samples)
    samples[5] = dict(samples[5], yaw=None)
    result = characterize_samples(samples)
    assert result['drift']['separated']
    assert (result['drift']['x'], result['drift']['y']) == approx((0.1, 0.15), abs=1e-6)
    assert {k: c['d1'] for k, c in result['speed_curves'].items()} == approx(
        {k: c['d1'] for k, c in clean['speed_curves'].items()}, abs=1e-6)


def test_one_heading_cannot_separate_and_falls_back_to_a_body_bias():
    result = characterize_samples(boat_samples((0.1, 0.0), [0.3] * 64))
    assert result['drift'] == {'separated': False}
    # The current is folded into the body-frame bias at that heading.
    assert result['mechanical_bias']['surge'] == approx(BIAS[0] + 0.1 * math.cos(0.3))


def test_refit_with_the_decoupling_active_composes():
    yaws = [2 * math.pi * i / 64 for i in range(64)]
    first = characterize_samples(boat_samples((0.0, 0.0), yaws))
    c = numpy.array(first['decoupling']).reshape(3, 3)
    assert GAIN @ c == approx(numpy.diag(numpy.diag(GAIN)), abs=1e-9)
    again = characterize_samples(boat_samples((0.0, 0.0), yaws, decoupling=c), c.flatten())
    assert numpy.array(again['decoupling']).reshape(3, 3) == approx(c, abs=1e-9)


def test_acceleration_comes_from_the_motion_not_sample_noise():
    from mhseals_testing.analysis.characterization import acceleration_limits
    # 50 Hz: a 0.5 m/s^2 ramp for 2 s, then steady, with +-0.02 m/s jitter on every sample.
    samples = []
    for i in range(300):
        t = i / 50
        v = 0.5 * min(t, 2.0) + (0.02 if i % 2 else -0.02)
        samples.append(((1, 0, 0), (v, 0.0, 0.0), t, 0.0))
    up, down = acceleration_limits(samples)
    assert up['surge'] == pytest.approx(0.5, abs=0.1)  # neighbour differences would say 2.5
    assert down['surge'] < 0.2


def test_limits_retain_the_tested_effort():
    half = [dict(s, command=[c / 2 for c in s['command']], velocity=[v / 2 for v in s['velocity']])
            for s in samples()]
    limits = characterize_samples(half)['motion_limits']
    assert limits['velocity']['surge'] == approx(0.3)  # measured at half effort; full speed is untested
    assert limits['measured_at_effort']['surge'] == approx(0.5)


def test_nav2_limits_do_not_freeze_planner_or_grant_unmeasured_reverse(tmp_path):
    import yaml
    source = tmp_path / 'base.yaml'
    source.write_text(yaml.safe_dump({
        'planner_server': {'ros__parameters': {'GridBased': {'plugin': 'current'}}},
        'controller_server': {'ros__parameters': {'FollowPath': {'vx_min': -.2}}},
        'velocity_smoother': {'ros__parameters': {'min_velocity': [-.2, -.4, -.3]}}}))
    profile = nav2_profile(characterize_samples(samples()), source)
    assert 'planner_server' not in profile
    follow = profile['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_min'] == -.2
    assert 'prune_distance' not in follow  # depends on the horizon at launch
    assert profile['velocity_smoother']['ros__parameters']['min_velocity'][0] == -.2


def test_rosbag_pairs_preceding_commands_and_preserves_transients(tmp_path):
    bag = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from mhseals_testing.analysis.characterization import read_rosbag

    path = tmp_path / 'motion'
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(path), storage_id='sqlite3'),
                bag.ConverterOptions('cdr', 'cdr'))
    for index, (name, kind) in enumerate((('/cmd_vel', 'geometry_msgs/msg/Twist'),
                                         ('/odom/local', 'nav_msgs/msg/Odometry'))):
        writer.create_topic(bag.TopicMetadata(
            id=index, name=name, type=kind, serialization_format='cdr'))
    command, reverse, odom = Twist(), Twist(), Odometry()
    command.linear.x, reverse.linear.x = .5, -.5
    odom.pose.pose.orientation.w = 1.
    for stamp, message in ((0, odom), (1, command), (2, odom), (3, command),
                           (4, odom), (5, reverse), (5.5, odom), (6, odom)):
        topic = '/cmd_vel' if isinstance(message, Twist) else '/odom/local'
        writer.write(topic, serialize_message(message), int(stamp * 1e9))
    del writer
    motion = read_rosbag(path, transient=0.)
    assert [sample['command_age'] for sample in motion] == [1., 3., .5, 1.]
    assert [sample['command'][0] for sample in motion] == [.5, .5, -.5, -.5]
    assert read_rosbag(path) == [sample for sample in motion if sample['command_age'] >= 1.]


def test_wiring_gate_checks_the_main_response_sign_only():
    from mhseals_testing.analysis.characterization import check_wiring
    # Field replay of 2026-10-18: right signs, large rad/s-vs-m/s cross terms.
    check_wiring([[1.13, 2.23, 0], [.1, .69, 0], [0, 0, 1.98]])
    for axis in range(3):
        gain = [[1., 0, 0], [0, 1., 0], [0, 0, 1.]]
        gain[axis][axis] = -1.
        with pytest.raises(ValueError, match=('surge', 'sway', 'yaw')[axis]):
            check_wiring(gain)


def hold_history(gap=None, frozen=False):
    """Five 6 s holds sampled at 10 Hz; ``gap`` removes a time range, ``frozen`` holds one velocity."""
    windows = [((0.5 * k, 0, 0), 6.0 * k, 6.0 * k + 6.0) for k in range(5)]
    history = [(t / 10, {'vx': 0.3 if frozen else 0.3 + 0.01 * math.sin(t), 'vy': 0.0, 'wz': 0.0})
               for t in range(300) if not gap or not gap[0] <= t / 10 < gap[1]]
    return history, windows


def test_audit_accepts_complete_holds():
    assert all(hold['ok'] for hold in audit_holds(*hold_history()))


def test_audit_rejects_a_hold_inside_an_odometry_gap():
    audit = audit_holds(*hold_history(gap=(12.0, 18.05)))
    assert [hold['ok'] for hold in audit] == [True, True, False, True, True]
    assert audit[2]['samples'] == 0 and audit[2]['coverage'] == 0.0


def test_audit_rejects_frozen_odometry():
    assert [hold['ok'] for hold in audit_holds(*hold_history(frozen=True))] == [True] + [False] * 4
