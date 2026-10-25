import json
import math

import numpy
import pytest

from pytest import approx

from mhseals_testing.analysis.characterization import (
    audit_holds, motion_limits, nav2_profile, rule_limits)


def samples():
    rows = []
    commands = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                (0, 0, 1), (0, 0, -1), (1, 1, 0), (-1, 1, -1))
    for stamp, command in enumerate(commands):
        rows.append({'time': stamp, 'command': command,
                     'velocity': (0.6 * command[0], 0.4 * command[1],
                                  0.3 * command[2])})
    return rows


def characterize_samples(rows, efforts=(1.0, 1.0, 1.0)):
    return dict(motion_limits=motion_limits(rows, rows, efforts))


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


RULES = {'delay_s': 0.15, 'tau_s': {'surge': 0.35, 'sway': 0.45, 'yaw': 0.15},
         'stop_distance_m': 1.0, 'ramp_lags': 3.0, 'yaw_tolerance': 0.25}


def test_rule_limits_stop_within_the_distance_and_overshoot_half_the_tolerance():
    velocity, acceleration = rule_limits(RULES)
    for axis in ('surge', 'sway'):
        lag = RULES['tau_s'][axis] + RULES['delay_s']
        v, a = velocity[axis], acceleration[axis]
        assert v * v / (2 * a) + v * lag == approx(RULES['stop_distance_m'])
        assert v / a == approx(RULES['ramp_lags'] * lag)
    assert velocity['yaw'] * (RULES['tau_s']['yaw'] + RULES['delay_s']) == approx(0.125)


def test_nav2_profile_takes_the_smaller_of_measured_and_rule(tmp_path):
    source = tmp_path / 'nav2.yaml'
    source.write_text('controller_server:\n  ros__parameters:\n    FollowPath: {vx_min: -0.1}\n'
                      'velocity_smoother:\n  ros__parameters:\n    min_velocity: [-0.1, -1.0, -1.0]\n')
    cap_v, cap_a = rule_limits(RULES)
    fast = {'motion_limits': {'velocity': {'surge': 2.0, 'sway': 0.1, 'yaw': 1.6},
                              'acceleration': {'surge': 3.0, 'sway': 0.05, 'yaw': 4.0},
                              'deceleration': {'surge': 3.0, 'sway': 2.0, 'yaw': 0.1}}}
    follow = nav2_profile(fast, source, rules=RULES)['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_max'] == approx(cap_v['surge'])       # top speed above the rule: the rule
    assert follow['vy_max'] == approx(0.1)                   # measured below the rule: measured
    assert follow['wz_max'] == approx(cap_v['yaw'])
    assert follow['ax_max'] == approx(cap_a['surge'])
    assert follow['ax_min'] == approx(-cap_a['surge'])
    assert follow['ay_max'] == approx(0.05)
    smoother = nav2_profile(fast, source, rules=RULES)['velocity_smoother']['ros__parameters']
    # deceleration: surge and sway above the rule take the rule, yaw below it keeps the measured 0.1
    assert smoother['max_decel'] == approx([-cap_a['surge'], -cap_a['sway'], -0.1])
    unruled = nav2_profile(fast, source)['controller_server']['ros__parameters']['FollowPath']
    assert unruled['vx_max'] == approx(2.0)


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
    limits = characterize_samples(half, (.5, .5, .5))['motion_limits']
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
