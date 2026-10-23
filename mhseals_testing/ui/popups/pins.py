"""The thruster pin popup: PWM channels and the MOSFET line, applied live.

The text is ``channels line`` such as ``0 0 0 0 29``: four PWM channel numbers
(outputs 1-4) then the MOSFET enable line. The chip paths stay as configured.
"""

from mhseals_testing.harness.apply import HardwareApply


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
    """Ask for new pins and hot-apply them; returns the notice to show."""
    text = app.ask_text('Thruster pins', 'PWM channels 1-4 then MOSFET line',
                        pin_text(app.args.hardware_config))
    if text is None:
        return 'Pins unchanged'
    try:
        changes = parse_pins(text)
    except ValueError as error:
        return f'Pins rejected: {error}'

    def save(changes):
        if not app.save_hardware(**changes):
            raise ValueError(app.notice)
    result = HardwareApply(save, app.driver_values, lambda: app.armed,
                           lambda: app.args.hardware_config,
                           serving=lambda: app.ros.parameter_service('thruster_pwm_node')).stage(changes)
    if result.status == 'saved':
        return 'Pins saved; applied at next arm'
    return 'Pins applied' if result.ok else f'Pins {result.status.replace("_", " ")}: {result.detail}'
