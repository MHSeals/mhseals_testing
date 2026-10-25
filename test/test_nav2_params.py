"""Nav2 parameter plans: which keys a running stack applies by itself, which need a reset."""

import pytest

from mhseals_testing.harness import nav2_params as n2

FOLLOW = n2.FOLLOW
CURRENT = {FOLLOW + 'vx_max': 0.4, FOLLOW + 'vy_max': 0.15, FOLLOW + 'wz_max': 0.3, FOLLOW + 'vx_min': -0.1, FOLLOW + 'time_steps': 48,
           FOLLOW + 'model_dt': 0.1, FOLLOW + 'prune_distance': 3.0,
           'controller_server.controller_frequency': 10.0,
           'velocity_smoother.max_velocity': [0.4, 0.15, 0.3]}


@pytest.mark.parametrize('key,live', [
    (FOLLOW + 'wz_max', True),
    (FOLLOW + 'CostCritic.cost_weight', False),
    (FOLLOW + 'prune_distance', True),
    ('velocity_smoother.max_accel', True),
    ('global_costmap.global_costmap.footprint', False),
    ('global_costmap.inflation_layer.inflation_radius', False),
    ('global_costmap.global_costmap.inflation_layer.cost_scaling_factor', False),
    # The local costmap feeds the MPPI CostCritic precompute, read at configure.
    ('local_costmap.local_costmap.footprint', False),
    ('local_costmap.inflation_layer.inflation_radius', False),
    # Copied once at configure by ConstraintCritic and PathAngleCritic (vx_min's sign).
    (FOLLOW + 'vx_max', False),
    (FOLLOW + 'vx_min', False),
    (FOLLOW + 'vy_max', False),
    # Read at configure, or by nothing that listens.
    (FOLLOW + 'motion_model', False),
    (FOLLOW + 'PathAngleCritic.mode', False),
    (FOLLOW + 'critics', False),
    (FOLLOW + 'time_steps', False),
    ('controller_server.controller_frequency', False),
    ('velocity_smoother.scale_velocities', False),
    ('controller_server.progress_checker.plugin', False),
    ('planner_server.GridBased.tolerance', False),
    ('bt_navigator.bt_loop_duration', False),
    ('controller_server.use_sim_time', False),
])
def test_only_the_listed_keys_are_live(key, live):
    assert n2.is_live(key) is live


def test_every_pattern_matches_a_real_nav2_key_shape():
    patterns = n2.live_patterns()
    assert len(patterns) == len(set(patterns))
    assert all(p.split('.')[0] in n2.ORDER for p in patterns)


def test_undeclared_key_is_missing_not_live_or_reset():
    declared = {'controller_server': {'FollowPath.wz_max'}}
    out = n2.plan({FOLLOW + 'wz_max': 0.2, FOLLOW + 'NewCritic.cost_weight': 1.0}, CURRENT, declared)
    assert out['missing'] == [FOLLOW + 'NewCritic.cost_weight']
    assert out['live']['controller_server']['FollowPath.wz_max'] == 0.2
    assert n2.plan({FOLLOW + 'NewCritic.cost_weight': 1.0}, CURRENT)['missing'] == []   # declared unknown


def test_target_names_the_ros_node():
    assert n2.target('controller_server.FollowPath.vx_max') == ('/controller_server', 'FollowPath.vx_max')
    assert n2.target('local_costmap.local_costmap.footprint') == ('/local_costmap/local_costmap', 'footprint')
    assert n2.target('global_costmap.footprint') == ('/global_costmap/global_costmap', 'footprint')


def test_plan_adds_derived_keys_and_orders_nodes():
    out = n2.plan({FOLLOW + 'vx_max': 0.5, 'controller_server.controller_frequency': 20.0}, CURRENT)
    assert out['reset']['controller_server'][FOLLOW[len('controller_server.'):] + 'model_dt'] == 0.05
    assert out['reset']['controller_server']['FollowPath.vx_max'] == 0.5
    assert list(out['live']) == ['velocity_smoother']
    assert out['live']['velocity_smoother']['max_velocity'][0] == 0.5
    assert 'FollowPath.prune_distance' not in out['reset']['controller_server']  # floor holds


def test_plan_moves_shim_and_stds_with_limits():
    current = {**CURRENT, FOLLOW + 'rotate_to_heading_angular_vel': 0.3, FOLLOW + 'vx_std': 0.2}
    out = n2.plan({FOLLOW + 'wz_max': 0.2}, current)
    assert out['live']['controller_server']['FollowPath.rotate_to_heading_angular_vel'] == 0.2
    out = n2.plan({FOLLOW + 'vx_max': 0.1}, current)
    assert out['live']['controller_server']['FollowPath.vx_std'] == 0.1


def test_plan_drops_unchanged_values():
    out = n2.plan({FOLLOW + 'vx_max': 0.4}, CURRENT)
    assert out['unchanged'] == [FOLLOW + 'vx_max']
    assert out['live'] == {} and out['reset'] == {}


def _compiled_plan_values(current, changes, result):
    """Reconstruct the preview value set from its class/node groups."""
    values = dict(current)
    for kind in ('live', 'reset'):
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
    assert n2.plan({FOLLOW + 'vx_max': 0.5}, values)['reset'] == {
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


@pytest.mark.parametrize('changes', [
    {FOLLOW + 'vx_min': -0.8},
    {FOLLOW + 'vx_max': 1},
])
def test_complete_preview_emits_canonical_caps_and_numeric_types(changes):
    from mhseals_nav import navigation_config as domain

    current = {**CURRENT, 'controller_server.controller_plugins': ['FollowPath'],
               FOLLOW + 'plugin': 'nav2_mppi_controller::MPPIController',
               FOLLOW + 'vx_std': 0.2, FOLLOW + 'vy_std': 0.1, FOLLOW + 'wz_std': 0.2}
    expected = n2._flat(domain.validated_merge_limits(n2._nested(current), n2._nested(changes)))
    result = n2.plan(changes, current)
    actual = _compiled_plan_values(current, changes, result)
    assert actual == expected
    assert {key: type(value) for key, value in actual.items()} == {
        key: type(value) for key, value in expected.items()}
    if FOLLOW + 'vx_min' in changes:
        assert result['unchanged'] == [FOLLOW + 'vx_min']
        assert all(not result[kind] for kind in ('live', 'reset'))
    else:
        emitted = result['reset']['controller_server']['FollowPath.vx_max']
        assert emitted == 1.0 and type(emitted) is float
