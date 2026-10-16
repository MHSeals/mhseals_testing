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

from mhseals_hardware.odroid_pwm import resolve_pwm_chip

BATTERY_WARN, BATTERY_FAIL = 0.30, 0.15     # remaining fraction
CLOCK_WARN = 0.5                            # seconds between sender stamp and our clock
LOAD_WARN, TEMP_WARN, TEMP_FAIL = 0.9, 75.0, 85.0
DISK_WARN, DISK_FAIL = 5e9, 1e9             # bytes free where the bag is written


def battery(voltage, percentage):
    """``percentage`` is 0..1, NaN when the FCU doesn't estimate it (sensor_msgs/BatteryState)."""
    if not math.isfinite(percentage):
        return 'OK', f'{voltage:.1f} V (no charge estimate)'
    level = ('FAIL' if percentage < BATTERY_FAIL else
             'WARN' if percentage < BATTERY_WARN else 'OK')
    return level, f'{voltage:.1f} V, {percentage:.0%}'


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


def pwm_readback(chips, channels):
    """Read what the kernel is actually outputting on each PWM pin."""
    rows, level = [], 'OK'
    for chip, channel in zip(chips, channels):
        try:
            path = resolve_pwm_chip(chip) / f'pwm{channel}'
            enabled = (path / 'enable').read_text().strip() == '1'
            period = int((path / 'period').read_text())
            duty = int((path / 'duty_cycle').read_text())
        except (OSError, ValueError):
            rows.append('unavailable')
            level = 'N/A'
            continue
        rows.append(f"{'ON' if enabled else 'OFF'} {duty / 1000:g}us / {1e9 / period:g}Hz"
                    if period else 'period unset')
    return level, rows
