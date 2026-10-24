"""The thruster pin popup: PWM channels and the MOSFET line, applied when the next session starts.

The text is ``channels line`` such as ``0 0 0 0 29``: four PWM channel numbers
(outputs 1-4) then the MOSFET enable line. The chip paths stay as configured.
"""


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
    """Save new pins to the operator YAML; a session snapshots it at start, so restart the TUI."""
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
    return 'Pins saved; applied when the next session starts (restart the TUI)'
