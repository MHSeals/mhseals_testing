"""The outputs capability (profiles.OUTPUT_CAPABILITIES): one table says what `arming: outputs:` can do, and
code asks it instead of comparing a board name. The table must keep today's meaning exactly: only the pin
output drives real PWM pins, so a real boat is never treated as a simulator, nor the other way round."""

from types import SimpleNamespace

import pytest

from mhseals_testing import profiles

pytestmark = pytest.mark.smoke


def test_outputs_validation_values_unchanged():
    assert profiles.OUTPUTS == ('none', 'odroid', 'topic')
    assert profiles.ARMED_OUTPUTS == ('odroid', 'topic')


@pytest.mark.parametrize('outputs', profiles.OUTPUTS + ('bogus', None))
def test_capability_matches_the_old_board_comparison(outputs):
    profile = SimpleNamespace(outputs=outputs)
    assert profiles.drives_pwm_pins(profile) is (outputs == 'odroid')
    armed = outputs in profiles.ARMED_OUTPUTS
    assert armed is (outputs in ('odroid', 'topic'))


def test_no_profile_drives_no_pins():
    assert profiles.drives_pwm_pins(None) is False
    assert profiles.drives_pwm_pins(SimpleNamespace()) is False


@pytest.mark.parametrize('name, pins, capabilities', [
    ('boat', True, {'arm', 'pwm_pins'}),
    ('crane_full_stack', False, {'arm', 'pwm_topic'}),
    ('crane_direct', False, set()),
])
def test_shipped_profiles(name, pins, capabilities):
    profile = profiles.load_profile(name)
    assert profile.drives_pwm_pins is pins is profiles.drives_pwm_pins(profile)
    assert profile.capabilities == capabilities
    assert profile.can_arm is ('arm' in capabilities)


def test_pin_map_is_the_drivers_table():
    from mhseals_hardware.odroid_pwm import J2_PINS
    assert profiles.load_profile('boat').pin_map() == J2_PINS
    assert profiles.pin_map() is J2_PINS
    assert profiles.load_profile('crane_full_stack').pin_map() == {}
