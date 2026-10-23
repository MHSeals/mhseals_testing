"""Advisory checks for the PWM driver and readback (ported by ts027).

Each function turns raw readings into ``(level, detail)`` for
``BoatMonitor.advise``. None of them can block motion (see readiness.py).
Power, flight controller, clock and compute are health checks now: see
``health/probes.py`` and ``config/health/``.
"""

from mhseals_hardware.odroid_pwm import period_ns, resolve_pwm_chip
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM
from mhseals_testing.ros.arming import DRIVER


def pwm_driver(nodes, subscribers, topic, armed, managed):
    """Compare the driver's ROS endpoint and supervised process with arm state."""
    drivers = [name for name in subscribers if name.rsplit('/', 1)[-1] == DRIVER]
    names = [name for name in nodes if name.rsplit('/', 1)[-1] == DRIVER]
    count = max(len(names), len(drivers))
    if count > 1:
        return 'FAIL', f'DUPLICATE; {count} {DRIVER} nodes/endpoints discovered'
    if not count:
        if armed or managed:
            return ('FAIL' if armed else 'WARN',
                    f'NO NODE; no {DRIVER} subscription on {topic}; '
                    f"managed process {'running' if managed else 'stopped'}")
        return 'OK', 'STOPPED; starts on arm'
    if not drivers:
        return ('FAIL' if armed else 'WARN',
                f'NO INPUT; {names[0]} discovered but not subscribed to {topic}')
    if not armed or not managed:
        return ('FAIL' if armed else 'WARN',
                f'UNMANAGED; {drivers[0]} subscribes to {topic} outside this armed session')
    return 'OK', f'RUNNING; managed process and {drivers[0]} subscription on {topic}'


def pwm_readback(chips, channels, armed=False, frequency=50.0):
    """Return level, summary and per-output kernel settings, without writing pins.

    Enabled PWM also pulses at neutral. Sysfs reports the kernel configuration,
    not measured pin voltage, MOSFET state, ESC power or propeller motion.
    """
    rows, issues = [], []
    enabled_count = unknown = non_neutral = 0
    expected_period = period_ns(frequency)
    for output, (chip, channel) in enumerate(zip(chips, channels), 1):
        try:
            chip = resolve_pwm_chip(chip)
            path = chip / f'pwm{channel}'
            if chip.is_dir() and not path.exists():
                rows.append('OFF (unexported)')
                continue
            enable = (path / 'enable').read_text().strip()
            if enable not in ('0', '1'):
                raise ValueError('invalid enable value')
            enabled = enable == '1'
            period = int((path / 'period').read_text())
            duty = int((path / 'duty_cycle').read_text())
        except (OSError, ValueError):
            rows.append('unavailable')
            unknown += 1
            continue
        enabled_count += enabled
        non_neutral += enabled and duty != NEUTRAL_PWM * 1000
        rows.append(f"{'ON' if enabled else 'OFF'} {duty / 1000:g}us / {1e9 / period:g}Hz"
                    if period > 0 else f"{'ON' if enabled else 'OFF'} period unset")
        if duty < 0 or period < 0 or duty > period:
            issues.append(f'output {output}: invalid period/duty')
        elif enabled:
            if abs(period - expected_period) > 1:
                issues.append(f'output {output}: expected {frequency:g}Hz')
            if not MIN_PWM * 1000 <= duty <= MAX_PWM * 1000:
                issues.append(f'output {output}: pulse outside {MIN_PWM}-{MAX_PWM}us')
    if armed and (enabled_count != len(chips) or unknown or issues):
        level = 'FAIL'
    elif (enabled_count and not armed) or issues:
        level = 'WARN'
    else:
        level = 'N/A' if unknown else 'OK'
    summary = f'{enabled_count}/{len(chips)} ON' + ('?' if unknown else '')
    detail = (f'{summary}; {non_neutral} non-neutral, {unknown} unavailable; '
              + ' | '.join(f'output {index}: {row}' for index, row in enumerate(rows, 1)))
    if enabled_count and not armed:
        issues.append('outputs enabled while this session is disarmed')
    if issues:
        detail += '; ' + '; '.join(issues)
    return level, detail, rows
