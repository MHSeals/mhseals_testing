"""Fusion inputs come from EKF parameters; the measurement weight needs two sources."""
import pytest

from mhseals_testing.analysis import fusion


def mask(*states):
    return [state in states for state in fusion.STATES]


PARAMS = {'odom0': '/odom/mavros', 'odom0_config': mask('vx', 'vy'),
          'imu0': '/imu/raw', 'imu0_config': mask('yaw', 'vyaw'),
          'odom1': '/odom/gps', 'odom1_config': mask('x', 'y'), 'world_frame': 'map'}


def test_inputs_follow_the_mask_order():
    inputs = {item.name: item for item in fusion.inputs_from_params(PARAMS)}
    assert inputs['odom0'].fused == ('vx', 'vy') and inputs['odom0'].topic == '/odom/mavros'
    assert inputs['imu0'].fused == ('yaw', 'vyaw')
    assert inputs['odom1'].fused == ('x', 'y') and len(inputs) == 3


def test_missing_or_short_mask_fuses_nothing_and_topicless_input_is_skipped():
    items = fusion.inputs_from_params({'odom0': '/a', 'imu0': '/b', 'imu0_config': [True], 'odom1': ''})
    assert [(i.name, i.fused) for i in items] == [('odom0', ()), ('imu0', ())]


def test_trust_is_inverse_variance_share():
    shares = fusion.trust({'a': 1.0, 'b': 2.0})
    assert shares['a'] == pytest.approx(0.8) and shares['b'] == pytest.approx(0.2)


@pytest.mark.parametrize('sigmas', [{}, {'a': 0.5}, {'a': 0.5, 'b': 0.0}, {'a': 0.5, 'b': float('nan')}])
def test_trust_hidden_below_two_usable_sources(sigmas):
    assert fusion.trust(sigmas) == {}
