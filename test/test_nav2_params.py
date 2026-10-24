"""Nav2 parameter classes (HR6): what may be set live, what needs a reset or a restart."""

import pytest

from mhseals_testing.harness import nav2_params as n2

FOLLOW = n2.FOLLOW
CURRENT = {FOLLOW + 'vx_max': 0.4, FOLLOW + 'vy_max': 0.15, FOLLOW + 'wz_max': 0.3, FOLLOW + 'vx_min': -0.1, FOLLOW + 'time_steps': 48,
           FOLLOW + 'model_dt': 0.1, FOLLOW + 'prune_distance': 3.0,
           'controller_server.controller_frequency': 10.0,
           'velocity_smoother.max_velocity': [0.4, 0.15, 0.3]}


@pytest.mark.parametrize('key,cls', [
    (FOLLOW + 'vx_max', 'live'),
    (FOLLOW + 'CostCritic.cost_weight', 'live'),
    ('velocity_smoother.max_accel', 'live'),
    ('controller_server.precise_goal_checker.xy_goal_tolerance', 'live'),
    ('local_costmap.local_costmap.obstacle_layer.pointcloud.min_obstacle_height', 'live'),
    ('global_costmap.inflation_layer.inflation_radius', 'live'),
    ('local_costmap.inflation_layer.inflation_radius', 'reset'),
    ('local_costmap.local_costmap.inflation_layer.enabled', 'reset'),
    ('global_costmap.global_costmap.inflation_layer.enabled', 'live'),
    ('local_costmap.footprint', 'reset'),
    ('global_costmap.footprint', 'live'),
    ('controller_server.controller_frequency', 'reset'),
    (FOLLOW + 'critics', 'reset'),
    ('bt_navigator.bt_loop_duration', 'reset'),
    ('behavior_server.cycle_frequency', 'reset'),
    ('planner_server.GridBased.tolerance', 'live'),
    ('waypoint_follower.loop_rate', 'live'),
    ('controller_server.use_sim_time', 'restart'),
])
def test_classes(key, cls):
    assert n2.classify(key) == cls


@pytest.mark.parametrize('key', [FOLLOW + 'motion_model', FOLLOW + 'PathAngleCritic.mode'])
def test_motion_model_and_mode_are_never_live(key):
    assert n2.classify(key, 'Omni', 'DiffDrive') == 'reset'
    assert n2.classify(key, 'Omni', 'DiffDrive', declared={key[len('controller_server.'):]}) == 'reset'


def test_undeclared_key_needs_a_restart_even_if_a_rule_says_live():
    declared = {'FollowPath.vx_max'}
    assert n2.classify(FOLLOW + 'vx_max', declared=declared) == 'live'
    assert n2.classify(FOLLOW + 'NewCritic.cost_weight', declared=declared) == 'restart'
    assert n2.classify(FOLLOW + 'NewCritic.cost_weight') == 'live'  # declared unknown: no verdict


def test_vx_min_sign_change_resets():
    assert n2.classify(FOLLOW + 'vx_min', -0.1, -0.2) == 'live'
    assert n2.classify(FOLLOW + 'vx_min', -0.1, 0.05) == 'reset'


def test_target_names_the_ros_node():
    assert n2.target('controller_server.FollowPath.vx_max') == ('/controller_server', 'FollowPath.vx_max')
    assert n2.target('local_costmap.local_costmap.footprint') == ('/local_costmap/local_costmap', 'footprint')
    assert n2.target('global_costmap.footprint') == ('/global_costmap/global_costmap', 'footprint')


def test_plan_adds_derived_keys_and_orders_nodes():
    out = n2.plan({FOLLOW + 'vx_max': 0.5, 'controller_server.controller_frequency': 20.0}, CURRENT)
    assert out['reset']['controller_server'][FOLLOW[len('controller_server.'):] + 'model_dt'] == 0.05
    live = out['live']
    assert list(live) == ['velocity_smoother', 'controller_server']
    assert live['velocity_smoother']['max_velocity'][0] == 0.5
    assert 'FollowPath.prune_distance' not in live['controller_server']  # floor holds
    assert out['restart'] == {}


def test_plan_moves_shim_and_stds_with_limits():
    current = {**CURRENT, FOLLOW + 'rotate_to_heading_angular_vel': 0.3, FOLLOW + 'vx_std': 0.2}
    out = n2.plan({FOLLOW + 'wz_max': 0.2}, current)
    assert out['live']['controller_server']['FollowPath.rotate_to_heading_angular_vel'] == 0.2
    out = n2.plan({FOLLOW + 'vx_max': 0.1}, current)
    assert out['live']['controller_server']['FollowPath.vx_std'] == 0.1


def test_plan_drops_unchanged_values_and_flags_undeclared():
    out = n2.plan({FOLLOW + 'vx_max': 0.4, FOLLOW + 'Ghost.weight': 1.0}, CURRENT,
                  declared={'controller_server': {'FollowPath.vx_max'}})
    assert out['unchanged'] == [FOLLOW + 'vx_max']
    assert out['live'] == {}
    assert out['restart'] == {'controller_server': {'FollowPath.Ghost.weight': 1.0}}


def test_rules_file_rejects_unknown_class(tmp_path):
    path = tmp_path / 'r.yaml'
    path.write_text("rules:\n  - {node: x, param: y, class: hot}\n")
    with pytest.raises(ValueError):
        n2.load_rules(path)


def _compiled_plan_values(current, changes, result):
    """Reconstruct the preview value set from its class/node groups."""
    values = {**current, **changes}
    for kind in n2.CLASSES:
        for node, params in result[kind].items():
            values.update({node + '.' + key: value for key, value in params.items()})
    return values


@pytest.mark.parametrize('smoother', [True, False])
@pytest.mark.parametrize('plugin,name', [
    ('nav2_mppi_controller::MPPIController', 'Cruise'),
    ('nav2_rotation_shim_controller::RotationShimController', 'Cruise'),
    ('nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController', 'FollowPath'),
])
def test_preview_matches_actual_domain_compiler(plugin, name, smoother):
    from mhseals_nav import navigation_config as domain

    prefix = 'controller_server.' + name + '.'
    current = {key.replace(FOLLOW, prefix): value for key, value in CURRENT.items()
               if smoother or not key.startswith('velocity_smoother.')}
    current.update({'controller_server.controller_plugins': [name],
                    prefix + 'plugin': plugin, prefix + 'vx_std': 0.3,
                    prefix + 'vy_std': 0.1, prefix + 'wz_std': 0.2,
                    prefix + 'rotate_to_heading_angular_vel': 0.3})
    if plugin == 'nav2_rotation_shim_controller::RotationShimController':
        current[prefix + 'primary_controller'] = 'nav2_mppi_controller::MPPIController'
    changes = {prefix + 'vx_max': 0.1, 'controller_server.controller_frequency': 20.0}
    # Full launch's validated_merge_limits delegates to this exact domain compiler.
    expected = n2._flat(domain.validated_merge_limits(n2._nested(current), n2._nested(changes)))
    result = n2.plan(changes, current)
    assert _compiled_plan_values(current, changes, result) == expected
    assert n2.derived({**current, **changes}, 3.0) == n2._flat(
        domain.derived(n2._nested({**current, **changes}), n2._nested(current)))
    if name == 'FollowPath':
        assert expected[prefix + 'model_dt'] == current[prefix + 'model_dt']
        assert expected[prefix + 'vx_std'] == current[prefix + 'vx_std']


@pytest.mark.parametrize('assignment', [
    {FOLLOW + 'model_dt': 0.9},
    {'velocity_smoother.max_velocity': [9.0, 9.0, 9.0]},
])
def test_preview_rejects_conflicting_explicit_derived_assignments(assignment):
    with pytest.raises(ValueError, match='conflicts with canonical lowering'):
        n2.plan({FOLLOW + 'vx_max': 0.2, **assignment}, CURRENT)


def test_partial_legacy_fixture_does_not_invent_horizon_defaults():
    values = {FOLLOW + 'vx_max': 0.4}
    assert n2.derived(values) == values
    assert n2.plan({FOLLOW + 'vx_max': 0.5}, values)['live'] == {
        'controller_server': {'FollowPath.vx_max': 0.5}}


def test_preview_calls_domain_lowering(monkeypatch):
    from mhseals_nav import navigation_config as domain

    calls = []
    compiler = domain.lower_navigation

    def record(base, explicit):
        calls.append(explicit)
        return compiler(base, explicit)

    monkeypatch.setattr(domain, 'lower_navigation', record)
    n2.plan({FOLLOW + 'vx_max': 0.5}, CURRENT)
    assert calls == [{}, n2._nested({FOLLOW + 'vx_max': 0.5})]


def test_complete_request_rejects_invalid_domain_inputs():
    current = {**CURRENT, 'controller_server.controller_plugins': ['FollowPath'],
               FOLLOW + 'plugin': 'nav2_mppi_controller::MPPIController',
               FOLLOW + 'vx_std': 0.2, FOLLOW + 'vy_std': 0.1, FOLLOW + 'wz_std': 0.2}
    with pytest.raises(ValueError, match='finite number'):
        n2.plan({FOLLOW + 'vx_max': float('nan')}, current)
    del current[FOLLOW + 'time_steps']
    with pytest.raises(ValueError, match='time_steps'):
        n2.plan({FOLLOW + 'vx_max': 0.2}, current)


def test_selected_overlay_missing_is_empty_invalid_is_rejected(tmp_path):
    from mhseals_nav import navigation_config as domain

    missing = domain.load_overlay(tmp_path / 'missing.yaml', strict=True)
    assert n2.plan(n2._flat(missing), CURRENT)['live'] == {}
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text('controller_server: [broken\n')
    with pytest.raises(ValueError, match='Invalid overlay'):
        domain.load_overlay(invalid, strict=True)
    invalid.write_text('controller_server: 1\ncontroller_server: 2\n')
    with pytest.raises(ValueError, match='Duplicate key'):
        domain.load_overlay(invalid, strict=True)
