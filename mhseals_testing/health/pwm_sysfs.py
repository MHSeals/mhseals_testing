"""Single read-only acquisition of local kernel PWM settings."""

import time

from mhseals_hardware.odroid_pwm import period_ns, resolve_pwm_chip
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM
from mhseals_testing.health.pwm import Output, Readback


def sample(chips, channels, frequency=50., *, verified_local=False, clock=time.monotonic):
    """Require caller-established local ODROID ownership for usable evidence.

    Legacy presentation may read unverified local settings; those observations
    cannot establish Conditions neutrality. Remote callers must leave this false.
    Missing/unreadable chips remain unavailable, not disabled.
    """
    outputs = []
    configured = len(chips) == len(channels) == 4
    expected = period_ns(frequency)
    for chip, channel in zip(chips, channels):
        identity = f'{chip}/pwm{channel}'
        try:
            if type(channel) is not int or channel < 0:
                raise ValueError('invalid channel')
            chip = resolve_pwm_chip(chip)
            identity = str(chip / f'pwm{channel}')
            path = chip / f'pwm{channel}'
            if chip.is_dir() and not path.exists():
                outputs.append(Output(identity, 'unexported'))
                continue
            enable = (path / 'enable').read_text().strip()
            if enable not in ('0', '1'):
                raise ValueError('invalid enable value')
            period = int((path / 'period').read_text())
            duty = int((path / 'duty_cycle').read_text())
        except (OSError, ValueError):
            outputs.append(Output(identity, 'unavailable'))
            continue
        issue = ''
        if period <= 0 or duty < 0 or duty > period:
            issue = 'invalid period/duty'
        elif enable == '1':
            if abs(period - expected) > 1:
                issue = f'expected {frequency:g}Hz'
            elif not MIN_PWM * 1000 <= duty <= MAX_PWM * 1000:
                issue = f'pulse outside {MIN_PWM}-{MAX_PWM}us'
        outputs.append(Output(identity, 'enabled' if enable == '1' else 'disabled',
                              period, duty, issue))
    return Readback('odroid', clock(), tuple(outputs), verified_local, configured)
