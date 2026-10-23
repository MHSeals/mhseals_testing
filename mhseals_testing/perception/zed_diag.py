"""Map the ZED wrapper DiagnosticStatus (JP5 4.0 or JP6 5.4.1) to the standard perception keys.

Works on any object with ``level``, ``message`` and ``values`` (``key``/``value``), so
ROS messages and test doubles both fit. The wrapper keys are section names and the field
is the prefix of the value ("Mean Frequency: 14.9 Hz (99.3%)"); sections repeat, so an
entry is identified by (section, value prefix), never by key alone.
"""

import re

# section key (JP6 'Data Capture', JP5 'Capture') -> stage
SECTIONS = {'data capture': 'camera', 'capture': 'camera', 'object detection': 'sdk'}
# value prefix -> field
FIELDS = {'mean frequency': 'hz', 'tot. processing time': 'proc_ms', 'processing time': 'proc_ms'}
LEVELS = {0: 'ok', 1: 'warn', 2: 'fail', 3: 'stale'}
_VALUE = re.compile(r'\s*([A-Za-z. ]+?)\s*:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*([A-Za-z]*)')


def _number(value, unit, field):
    """Float, with proc_ms given in seconds converted to ms; None when not finite."""
    if field == 'proc_ms' and unit.lower() in ('s', 'sec', 'secs', 'second', 'seconds'):
        value *= 1000.0
    return value if value == value and abs(value) != float('inf') else None


def parse(status):
    """{'status': {'level', 'message'}, 'stages': {stage: {field: float}}, 'inference_ms'}.

    ``inference_ms`` is the sdk processing time (None when absent), per orin.md T4.
    """
    stages = {}
    for item in status.values:
        stage = SECTIONS.get(item.key.strip().lower())
        match = _VALUE.match(str(item.value))
        field = FIELDS.get(match.group(1).lower()) if match else None
        if not stage or not field:
            continue
        value = _number(float(match.group(2)), match.group(3), field)
        if value is not None:
            stages.setdefault(stage, {})[field] = value
    level = int(status.level)
    return {'status': {'level': LEVELS.get(level, 'stale'), 'message': status.message},
            'stages': stages, 'inference_ms': stages.get('sdk', {}).get('proc_ms')}


def is_zed(status):
    """True for a Stereolabs status (hardware_id starts with Stereolabs)."""
    return str(getattr(status, 'hardware_id', '')).startswith('Stereolabs')
