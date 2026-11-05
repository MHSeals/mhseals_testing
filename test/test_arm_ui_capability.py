"""The arming, pins and thruster UI asks the outputs capability, never the board name (audit finding 2)."""
import re
from pathlib import Path

import pytest

from mhseals_testing.profiles import OUTPUTS, OUTPUT_DESCRIPTIONS, outputs_description

PACKAGE = Path(__file__).resolve().parents[1] / 'mhseals_testing'
FILES = ['ui/app/actions/arm.py', 'ui/app/actions/settings.py', 'ui/app/actions/holds.py', 'ui/popups/pins.py',
         'ui/popups/stop.py', 'ui/popups/authority.py', 'ui/panels/thrusters.py', 'ui/takeover.py',
         'ui/authority_chip.py', 'ui/feeds/authority.py', *(f'ui/app/actions/settings_rows/{p.name}' for p in
                                                            (PACKAGE / 'ui/app/actions/settings_rows').glob('*.py'))]


@pytest.mark.parametrize('name', FILES)
def test_no_board_name_comparison_or_driver_import(name):
    source = (PACKAGE / name).read_text()
    assert not re.search(r"""[=!]=\s*['"]odroid['"]|['"]odroid['"]\s*[=!]=""", source), name
    assert 'odroid_pwm' not in source and 'J2_PINS' not in source, name


def test_every_outputs_value_has_its_arm_description():
    assert set(OUTPUT_DESCRIPTIONS) == set(OUTPUTS)
    profile = type('P', (), {})()
    texts = {}
    for outputs in OUTPUTS:
        profile.outputs = outputs
        texts[outputs] = outputs_description(profile, '/pwm')
    assert texts == {'odroid': 'ODROID PWM pins (real thrusters)', 'topic': 'simulator over ROS (/pwm)',
                     'none': 'commands straight to the plant'}


def test_the_pin_popup_and_panel_read_the_driver_table_through_the_profile():
    from mhseals_hardware.odroid_pwm import J2_PINS
    from mhseals_testing.ui.panels import thrusters
    from mhseals_testing.ui.popups import pins
    assert pins.PWM_PINS == {pin: chip for out, (pin, _, _, chip) in J2_PINS.items() if out != 'mosfet'}
    assert pins.MOSFET_PIN == J2_PINS['mosfet'][0]
    assert [thrusters.pin_of({}, output) for output in range(1, 5)] == [J2_PINS[o][0] for o in range(1, 5)]
