"""Advisory checks: power, flight controller, clock, compute and PWM readback.

Each function turns raw readings into ``(level, detail)`` for
``BoatMonitor.advise``. None of them can block motion (see readiness.py);
they exist so the operator sees a weak battery or a hot computer before it
matters.
"""

import glob
import math
import os
from pathlib import Path
import shutil

from mhseals_hardware.odroid_pwm import period_ns, resolve_pwm_chip
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM
from mhseals_testing.ros.arming import DRIVER

BATTERY_WARN, BATTERY_FAIL = 0.30, 0.15     # remaining fraction
VOLTAGE_ABSENT = 65.5                       # volts; MAVLink UINT16_MAX reads 65.535
CLOCK_WARN = 0.5                            # seconds between sender stamp and our clock
LOAD_WARN, TEMP_WARN, TEMP_FAIL = 0.9, 75.0, 85.0
DISK_WARN, DISK_FAIL = 5e9, 1e9             # bytes free where the bag is written


def battery(voltage, percentage):
    """``percentage`` is 0..1; the FCU's "unknown" arrives as NaN or a small negative (-1 %).

    MAVLink's UINT16_MAX "no voltage" arrives as 65.535 V and counts as absent.
    """
    volts = f'{voltage:.1f} V' if math.isfinite(voltage) and voltage < VOLTAGE_ABSENT else None
    if not math.isfinite(percentage) or percentage < 0:
        return ('OK', f'{volts} (no charge estimate)') if volts else ('N/A', 'no battery data')
    level = ('FAIL' if percentage < BATTERY_FAIL else
             'WARN' if percentage < BATTERY_WARN else 'OK')
    return level, f'{volts or "no voltage"}, {percentage:.0%}'


def flight_controller(connected, armed, mode):
    """MAVROS state: an armed FCU or a mode other than MANUAL could also drive the thrusters."""
    if not connected:
        return 'WARN', 'not connected'
    detail = f"{'ARMED' if armed else 'disarmed'}, {mode}"
    return ('WARN' if armed or mode not in ('MANUAL', 'HOLD') else 'OK'), detail


def clock(snapshot):
    """Largest stamp offset among live sensors: big values mean an unsynced computer."""
    ages = {name: row['stamp_age'] for name, row in snapshot['sensors'].items()
            if row['status'] != 'STALE' and row['stamp_age'] is not None}
    if not ages:
        return 'N/A', 'no live stamped sensors'
    name = max(ages, key=lambda key: abs(ages[key]))
    level = 'WARN' if abs(ages[name]) > CLOCK_WARN else 'OK'
    return level, f'worst {name} {ages[name]:+.2f}s'


def worst(*levels):
    return max(levels, key=('N/A', 'OK', 'WARN', 'FAIL').index)


def compute(load1, cpus, temperatures, disk_free):
    hottest = max(temperatures, default=None)
    levels = [
        'WARN' if load1 / max(cpus, 1) > LOAD_WARN else 'OK',
        'OK' if hottest is None or hottest <= TEMP_WARN else
        'WARN' if hottest <= TEMP_FAIL else 'FAIL',
        'FAIL' if disk_free < DISK_FAIL else 'WARN' if disk_free < DISK_WARN else 'OK',
    ]
    hot = f', {hottest:.0f}°C' if hottest is not None else ''
    return worst(*levels), f'load {load1:.1f}/{cpus}{hot}, {disk_free / 1e9:.1f} GB free'


def read_compute(bag_directory):
    """Sample this computer's load, thermal zones and the disk the bag is written to."""
    temperatures = []
    for zone in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try:
            temperatures.append(int(Path(zone).read_text()) / 1000)
        except (OSError, ValueError):
            pass
    directory = Path(bag_directory)
    while not directory.exists():
        directory = directory.parent
    return compute(os.getloadavg()[0], os.cpu_count() or 1, temperatures,
                   shutil.disk_usage(directory).free)


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
