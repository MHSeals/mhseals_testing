"""Thruster pins, restart-applied while disarmed.

``M`` edits the PWM channels and the MOSFET line: ``0 0 0 0 29`` is four channel
numbers (outputs 1-4) then the line. ``P`` moves the selected thruster's output to
another J2 header pin, one of the board's PWM pins (odroid_pwm.J2_PINS) that no
other output uses; it is a draft that ``s`` saves. A saved pin change must pass
the sysfs readback before arming.
"""

import hashlib
import json
import time
from pathlib import Path

from mhseals_hardware.configuration import load_config
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.odroid_pwm import J2_PINS
from mhseals_hardware.thruster_mixer import POSITIONS
from mhseals_testing.health.pwm_sysfs import sample
from mhseals_testing.ui import theme

READBACK_TIMEOUT = 5.0   # seconds the restarted driver gets to export and enable the outputs
PWM_PINS = {pin: chip for output, (pin, _pad, _pwm, chip) in J2_PINS.items() if output != 'mosfet'}
REFUSED = 'Pins refused: disarm first, pins move real outputs'
# Only the table's pins: each has its own controller. Another pad on a used PWM (J2 pin 22 is
# PWM3's other mux, beside pin 12) would carry the same pulse, so it is not offered.
PWM_NOTE = f"PWM pins {' '.join(map(str, sorted(PWM_PINS)))}; one pin per PWM (22 shares 12's)"


def busy(app):
    """True while armed or arming: pins never move under a running driver."""
    return app.armed or (app.arming is not None and app.arming.pending is not None)


def pin_hash(config):
    """A digest of the PWM chips and channels, which the readback checks.

    The MOSFET chip and line are left out: sysfs cannot show them, so a readback
    proves nothing about them and they must not count as verified.
    """
    return hashlib.sha256(json.dumps([list(config['pwm_chips']), list(config['pwm_channels'])])
                          .encode()).hexdigest()[:16]


def marker_path(app):
    """The verified-pins marker, beside the operator hardware.yaml so it outlives the TUI."""
    return Path(app.args.config).with_name('pins-verified.json')


def verified_hash(app):
    try:
        return json.loads(marker_path(app).read_text()).get('verified')
    except (OSError, ValueError, AttributeError):
        return None


def write_marker(app, digest):
    try:
        marker_path(app).write_text(json.dumps(dict(verified=digest)))
    except OSError:
        pass   # the in-process flag still refuses; the file only carries it across a restart


def mark_pending(app):
    """Before a pin write: arming is refused until a readback, here and after a TUI restart.

    A first change records the pins in force as the last verified, so the new ones differ.
    """
    app.pins_unverified = True
    if verified_hash(app) is None:
        write_marker(app, pin_hash(app.args.hardware_config))


def refusal(app):
    """Why arming must wait on the pins, or ''."""
    if app.pins_unverified:
        return 'pins changed, not read back yet: s runs the readback'
    if app.pins_moved():
        return 'unsaved pin change: s saves and reads it back, u undoes'
    if app.profile.outputs == 'odroid':
        digest = verified_hash(app)
        if digest is not None and digest != pin_hash(app.args.hardware_config):
            return 'pins differ from the last readback: s runs the readback'
    return ''


def output_pin(chips, output):
    """The J2 pin of physical ``output`` (1-based) under ``chips``; None for a path off the table."""
    return next((pin for pin, chip in PWM_PINS.items() if chip == chips[output - 1]), None)


def parse_pin(text, chips, output):
    """Return ``chips`` with ``output`` on the J2 pin in ``text``; ValueError says why not."""
    try:
        pin = int(text)
    except ValueError:
        raise ValueError('a J2 pin number') from None
    if pin == J2_PINS['mosfet'][0]:
        raise ValueError(f'pin {pin} is the MOSFET enable')
    if pin not in PWM_PINS:
        raise ValueError(f"pin {pin} has no PWM; use {' '.join(map(str, sorted(PWM_PINS)))}")
    for other in range(1, 5):
        if other != output and output_pin(chips, other) == pin:
            raise ValueError(f'pin {pin} already drives output {other}')
    chips = list(chips)
    chips[output - 1] = PWM_PINS[pin]
    return chips


def edit_row_pin(app, position):
    """P: put the selected thruster's output on another PWM pin, as a draft (s saves)."""
    if busy(app):
        return REFUSED
    channel_map, reversed_ = app.mapping()
    chips, output = app.draft_chips(), channel_map[position]
    pin = output_pin(chips, output)
    text = app.ask_text(f'{POSITIONS[position].upper()} pin', f'Output {output}{theme.SEPARATOR}{PWM_NOTE}',
                        '' if pin is None else str(pin), lambda text: parse_pin(text, chips, output))
    if text is None:
        return 'Pin unchanged'
    try:
        chips = parse_pin(text, chips, output)
    except ValueError as error:
        return f'Pin rejected: {error}'
    app.edit_mapping(channel_map, reversed_, chips)
    return app.notice


def pin_text(config):
    return ' '.join(str(n) for n in [*config['pwm_channels'], config['mosfet_line']])


def parse_pins(text):
    """Return the config changes for ``text``; ValueError says what is wrong."""
    try:
        numbers = [int(word) for word in text.split()]
    except ValueError:
        raise ValueError('numbers only') from None
    if len(numbers) != 5:
        raise ValueError(f'need 5 numbers (4 channels, the MOSFET line), got {len(numbers)}')
    if min(numbers) < 0:
        raise ValueError('numbers must not be negative')
    return dict(pwm_channels=numbers[:4], mosfet_line=numbers[4])


def edit_pins(app):
    """Save new pins; while disarmed, run just the driver, read sysfs back, then stop it."""
    if busy(app):
        return REFUSED
    text = app.ask_text('Thruster pins', 'PWM channels 1-4 then MOSFET line',
                        pin_text(app.args.hardware_config), parse_pins)
    if text is None:
        return 'Pins unchanged'
    try:
        changes = parse_pins(text)
    except ValueError as error:
        return f'Pins rejected: {error}'
    mark_pending(app)
    if not app.save_hardware(**changes):
        return f'Pins not saved: {app.notice}'
    return verify(app)


def verify(app):
    """After a saved pin change: start just the driver, read sysfs back, stop it.

    Arming stays refused (``app.pins_unverified``) until a readback confirms the pins.
    """
    if busy(app):
        return REFUSED
    if app.profile.outputs != 'odroid' or app.arming is None:
        app.pins_unverified = False   # nothing to read back; the marker keeps the real driver waiting
        return 'Pins saved; no ODROID driver in this profile, nothing restarted'
    app.pins_unverified = True
    blocked = app.thruster_refusal()
    if blocked:
        return f'Pins saved; driver not started: {blocked}'
    try:   # the readback starts the real driver: the same typed word as arming
        ok = app.confirm_drive(f'Pin readback{theme.SEPARATOR}{app.profile.name}')
    except Cancelled:
        ok = False
    if not ok:
        return 'Pins saved; driver not started, readback not confirmed'
    config = load_config(app.args.config)
    try:
        confirmed = app.arming.restart_driver(config, lambda: readback(config))
    except RuntimeError as error:
        return f'Pins saved; driver not restarted: {error}'
    if confirmed:
        app.pins_unverified = False
        write_marker(app, pin_hash(config))
        return ('Pins saved; driver restarted, sysfs showed the new channels enabled, driver '
                'stopped again (MOSFET line not read back)')
    return ('Pins saved; driver restarted; sysfs readback did not show the new channels '
            'enabled: check the driver log')


def readback(config):
    """Return whether sysfs shows all four channels enabled at the configured period in time."""
    deadline = time.monotonic() + READBACK_TIMEOUT
    while True:
        outputs = sample(config['pwm_chips'], config['pwm_channels'], config['frequency']).outputs
        if all(output.state == 'enabled' and not output.issue for output in outputs):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.2)
