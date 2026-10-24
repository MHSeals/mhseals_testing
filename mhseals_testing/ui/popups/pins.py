"""The thruster pin popup: PWM channels and the MOSFET line, restart-applied while disarmed.

The text is ``channels line`` such as ``0 0 0 0 29``: four PWM channel numbers
(outputs 1-4) then the MOSFET enable line. The chip paths stay as configured.
"""

import time

from mhseals_hardware.configuration import load_config
from mhseals_testing.health.pwm_sysfs import sample

READBACK_TIMEOUT = 5.0   # seconds the restarted driver gets to export and enable the outputs


def pin_text(config):
    return ' '.join(str(n) for n in [*config['pwm_channels'], config['mosfet_line']])


def parse_pins(text):
    """Return the config changes for ``text``; ValueError says what is wrong."""
    try:
        numbers = [int(word) for word in text.split()]
    except ValueError:
        raise ValueError('numbers only') from None
    if len(numbers) != 5:
        raise ValueError('need 4 channels and the MOSFET line')
    if min(numbers) < 0:
        raise ValueError('numbers must not be negative')
    return dict(pwm_channels=numbers[:4], mosfet_line=numbers[4])


def edit_pins(app):
    """Save new pins; while disarmed, restart just the driver and read the sysfs outputs back."""
    if app.armed or (app.arming and app.arming.pending is not None):
        return 'Pins refused: disarm first, pins move real outputs'
    text = app.ask_text('Thruster pins', 'PWM channels 1-4 then MOSFET line',
                        pin_text(app.args.hardware_config))
    if text is None:
        return 'Pins unchanged'
    try:
        changes = parse_pins(text)
    except ValueError as error:
        return f'Pins rejected: {error}'
    if not app.save_hardware(**changes):
        return f'Pins not saved: {app.notice}'
    if app.profile.outputs != 'odroid' or app.arming is None:
        return 'Pins saved; no ODROID driver in this profile, nothing restarted'
    config = load_config(app.args.config)   # the file just saved, not the session's snapshot
    app.arming.restart_driver(config)
    deadline = time.monotonic() + READBACK_TIMEOUT
    while True:
        outputs = sample(config['pwm_chips'], config['pwm_channels'], config['frequency']).outputs
        if all(output.state == 'enabled' for output in outputs):
            return ('Pins saved; driver restarted; sysfs shows the new channels enabled '
                    '(MOSFET line not read back)')
        if time.monotonic() >= deadline:
            return ('Pins saved; driver restarted; sysfs readback does not show the new channels '
                    'enabled: check the driver log')
        time.sleep(0.2)
