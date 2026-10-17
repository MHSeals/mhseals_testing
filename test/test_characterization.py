import json
import math

import numpy
import pytest

from pytest import approx

from mhseals_testing.analysis.characterization import (
    characterize_samples, nav2_profile, read_json_samples)


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
    # Nav2 gets 70 % of what the boat showed; the rest is the controller's authority.
    assert (follow['vx_max'], follow['vx_min']) == approx((.42, -.42))
    assert (follow['vy_max'], follow['wz_max']) == approx((.28, .21))
    assert behavior['max_rotational_vel'] == approx(.21)
    assert behavior['rotational_acc_lim'] == approx(.21)


def test_samples_from_windows_skips_transients_and_pairs_commands():
    from mhseals_testing.analysis.characterization import samples_from_windows
    history = [(t / 10, {'vx': t / 10, 'vy': 0.0, 'wz': 0.0}) for t in range(50)]
    windows = [((0, 0, 0), 0.0, 1.9), ((0.5, 0, 0), 2.0, 4.0)]
    samples = samples_from_windows(history, windows, transient=1.0)
    assert {s['command'] for s in samples} == {(0, 0, 0), (0.5, 0, 0)}
    assert min(s['time'] for s in samples if s['command'][0]) == pytest.approx(3.0)


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
