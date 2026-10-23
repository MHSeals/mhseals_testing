"""Map the ZED wrapper DiagnosticStatus (JP5 4.0 or JP6 5.4.1) to the standard perception keys.

Works on any object with ``level``, ``message`` and ``values`` (``key``/``value``), so
ROS messages and test doubles both fit. The wrappers repeat some keys; the last
occurrence wins. Stages hold only what the status reports.
"""

import re

# DiagnosticStatus key -> (stage, field)
KEYS = {
    'Capture Mean Frequency': ('camera', 'hz'),
    'Capture Tot. Processing Time': ('camera', 'proc_ms'),
    'Object detection Mean Frequency': ('sdk', 'hz'),
    'Object detection Processing Time': ('sdk', 'proc_ms'),
}
LEVELS = {0: 'ok', 1: 'warn', 2: 'fail', 3: 'stale'}
_NUMBER = re.compile(r'\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*([A-Za-z]*)')


def _number(text, field):
    """Float from '14.9 Hz' / '0.021 sec' / '21 msec'; proc_ms values are converted to ms."""
    match = _NUMBER.match(str(text))
    if not match:
        return None
    value, unit = float(match.group(1)), match.group(2).lower()
    if field == 'proc_ms' and unit in ('s', 'sec', 'secs', 'second', 'seconds'):
        value *= 1000.0
    return value if value == value and abs(value) != float('inf') else None


def parse(status):
    """{'status': {'level', 'message'}, 'stages': {stage: {field: float}}}."""
    stages = {}
    for item in status.values:
        mapped = KEYS.get(item.key.strip())
        if not mapped:
            continue
        value = _number(item.value, mapped[1])
        if value is not None:
            stages.setdefault(mapped[0], {})[mapped[1]] = value
    level = int(status.level)
    return {'status': {'level': LEVELS.get(level, 'stale'), 'message': status.message},
            'stages': stages}


def is_zed(status):
    """True for a Stereolabs status (hardware_id starts with Stereolabs)."""
    return str(getattr(status, 'hardware_id', '')).startswith('Stereolabs')
