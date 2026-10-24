"""YAML course authoring -> disposable JSON input for Unity's existing loader."""
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import struct
import tempfile

from mhseals_testing.strict_yaml import load_strict


def _strings(value):
    return isinstance(value, list) and all(type(v) is str for v in value)


def parse(raw):
    """Preserve supplied fields; omitted Unity fields retain Unity's defaults."""
    data = load_strict(raw)
    if not isinstance(data, dict) or set(data) - {'name', 'hide', 'jitter', 'elements'}:
        raise ValueError('expected a course mapping')
    if 'name' in data and type(data['name']) is not str:
        raise ValueError('course name must be a string')
    if 'hide' in data and not _strings(data['hide']):
        raise ValueError('course hide must be a string list')
    elements = data.get('elements')
    if not isinstance(elements, list) or not elements:
        raise ValueError('course needs nonempty elements')
    text = {'task', 'template', 'label', 'color', 'vessel', 'vesselLabel'}
    numbers = {'x', 'y', 'yaw', 'scale'}
    for element in elements:
        if (not isinstance(element, dict) or set(element) - text - numbers - {'bays', 'occupied'}
                or type(element.get('template')) is not str or not element['template']):
            raise ValueError('each course element needs a template and supported fields')
        if any(type(element[k]) is not str for k in text & element.keys()):
            raise ValueError('course element text must be strings')
        if 'bays' in element and not _strings(element['bays']):
            raise ValueError('course bays must be a string list')
        if 'occupied' in element and (not isinstance(element['occupied'], list)
                or any(type(v) is not int or not -(2 ** 31) <= v < 2 ** 31
                       for v in element['occupied'])):
            raise ValueError('course occupied must be a signed int32 list')
    values = [data['jitter']] if 'jitter' in data else []
    values += [e[k] for e in elements for k in numbers & e.keys()]
    for value in values:
        try:
            finite = (type(value) in (int, float) and math.isfinite(value)
                      and math.isfinite(struct.unpack('<f', struct.pack('<f', value))[0]))
        except (OverflowError, struct.error):
            finite = False
        if not finite:
            raise ValueError('course numeric fields must remain finite Unity float32 values')
    return data


def source(selection):
    """Bare names select installed resources; explicit YAML paths select files."""
    path = Path(selection).expanduser()
    if path.suffix.lower() in ('.yaml', '.yml'):
        return path.resolve()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', str(selection)):
        raise ValueError('select a shipped course name or a YAML path')
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_testing')) / 'config/courses' / (str(selection) + '.yaml')


def compile_course(selection):
    """Validate before cache effects; publish JSON atomically without overwriting."""
    document = parse(source(selection).read_text())
    encoded = (json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()
    cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache'))).expanduser()
    if not cache.is_absolute():
        raise ValueError('XDG_CACHE_HOME must be absolute')
    cache = cache / 'mhseals_testing/course-json'
    target = cache / (sha256(encoded).hexdigest() + '.json')
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.compile-', dir=cache) as temporary:
        staged = Path(temporary) / 'course.json'
        staged.write_bytes(encoded)
        try:
            os.link(staged, target)
        except FileExistsError:
            pass
    if target.is_symlink() or target.read_bytes() != encoded:
        raise ValueError('generated course cache conflict; remove the corrupt cache entry')
    return target
