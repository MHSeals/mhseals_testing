"""Advisory checks for the PWM driver and readback (ported by ts027).

Each function turns raw readings into ``(level, detail)`` for
``BoatMonitor.advise``. None of them can block motion (see readiness.py).
Power, flight controller, clock and compute are health checks now: see
``health/probes.py`` and ``config/health/``.
"""

from mhseals_hardware.thruster_mixer import NEUTRAL_PWM
from mhseals_testing.health.pwm_sysfs import sample
from mhseals_testing.ros.arming import DRIVER


def pwm_driver(nodes, subscribers, topic, armed, managed, driver=DRIVER):
    """Compare the driver's ROS endpoint and supervised process with arm state."""
    drivers = [name for name in subscribers if name.rsplit('/', 1)[-1] == driver]
    names = [name for name in nodes if name.rsplit('/', 1)[-1] == driver]
    count = max(len(names), len(drivers))
    if count > 1:
        return 'FAIL', f'DUPLICATE; {count} {driver} nodes/endpoints discovered'
    if not count:
        if armed or managed:
            return ('FAIL' if armed else 'WARN',
                    f'NO NODE; no {driver} subscription on {topic}; '
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
    return pwm_readback_rows(sample(chips, channels, frequency), armed)


def pwm_readback_rows(observation, armed=False):
    """Compatibility presentation of an existing acquisition, without more I/O.

    This legacy summary is not neutrality evidence: use observation.neutral
    and observation.health with a TTL and verified locality for that purpose.
    """
    rows, issues = [], []
    enabled_count = unknown = non_neutral = 0
    for index, output in enumerate(observation.outputs, 1):
        rows.append(output.row())
        unknown += output.state == 'unavailable'
        enabled = output.state == 'enabled'
        enabled_count += enabled
        non_neutral += enabled and output.duty_ns != NEUTRAL_PWM * 1000
        if output.issue:
            issues.append(f'output {index}: {output.issue}')
    count = len(observation.outputs)
    if armed and (enabled_count != count or unknown or issues):
        level = 'FAIL'
    elif (enabled_count and not armed) or issues:
        level = 'WARN'
    else:
        level = 'N/A' if unknown else 'OK'
    summary = f'{enabled_count}/{count} ON' + ('?' if unknown else '')
    detail = (f'{summary}; {non_neutral} non-neutral, {unknown} unavailable; '
              + ' | '.join(f'output {index}: {row}' for index, row in enumerate(rows, 1)))
    if enabled_count and not armed:
        issues.append('outputs enabled while this session is disarmed')
    if issues:
        detail += '; ' + '; '.join(issues)
    return level, detail, rows
