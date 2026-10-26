"""Session toggles and procedure parameters: one table, validated, persisted.

``SPECS`` is the single source for the Session rows, the fold/expand setting rows
and ``--set key=value``. ``Settings`` keeps plain attribute access, so procedure
code reads ``ctx.settings.amplitude``. The last used values persist per user in
``$XDG_CONFIG_HOME/mhseals/boat_test.yaml`` except the ones that must be chosen
again each start (``Spec.reset``): the command mode and the bench override.
"""

import math
import os
from dataclasses import dataclass
from pathlib import Path

from mhseals_testing.strict_yaml import load_strict

import yaml


@dataclass(frozen=True)
class Spec:
    key: str
    default: object
    kind: type = float          # float, int, str or bool
    low: float = None           # inclusive, except lower_open
    high: float = None
    step: float = None
    choices: tuple = ()
    lower_open: bool = False    # low itself is not allowed
    group: str = 'Session'      # row group: Session or the procedure family
    reset: bool = False         # never persisted; back to default at start
    note: str = ''

    def parse(self, value):
        """The validated value for ``value`` (text or typed); ValueError names the key."""
        try:
            if self.kind is bool:
                if isinstance(value, str):
                    if value.lower() not in ('on', 'off', 'true', 'false', '1', '0'):
                        raise ValueError
                    value = value.lower() in ('on', 'true', '1')
                value = bool(value)
            elif self.kind is str:
                value = str(value)
                if self.choices and value not in self.choices:   # no choices: the user checks it
                    raise ValueError
            else:
                value = self.kind(value)
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError
                if self.low is not None and (value < self.low or (self.lower_open and value == self.low)):
                    raise ValueError
                if self.high is not None and value > self.high:
                    raise ValueError
        except (TypeError, ValueError):
            raise ValueError(f'{self.key}: {value!r} is not {self.describe()}') from None
        return value

    def describe(self):
        if self.choices:
            return 'one of ' + '/'.join(self.choices)
        if self.kind is str:
            return 'text'
        if self.kind is bool:
            return 'on or off'
        low = '(' if self.lower_open else '['
        return f'within {low}{self.low:g}, {self.high:g}]'


def _timing(key, default, note=''):
    return Spec(key, default, low=0, high=60, step=1, lower_open=True, group='Trials', note=note)


SPECS = (
    Spec('command_mode', 'effort', str, choices=('effort', 'velocity'), reset=True,
         note='disarmed only'),
    Spec('bench_override', False, bool, reset=True, note='motion without sensors; scoring off'),
    Spec('stale_after', 2.0, low=0, high=5, step=0.25, lower_open=True),
    Spec('amplitude', 0.25, low=0, high=1, step=0.05, lower_open=True, group='Trials',
         note="in the profile's command units"),
    _timing('baseline', 5.0, 'neutral s before a trial'),
    _timing('command', 3.0, 's the command is held'),
    _timing('settle', 6.0, 'neutral s after a trial (the first coast_s of a characterization settle, see characterize.yaml, are coast)'),
    Spec('repeats', 1, int, low=1, high=5, step=1, group='Trials', note='trials per direction'),
    _timing('observe', 10.0, 'passive s for perception checks'),
    Spec('nav_distance', 4.0, low=1, high=50, step=1, group='Navigation', note='waypoint square side, m'),
    Spec('station_hold', 60.0, low=10, high=600, step=10, group='Navigation', note='station keeping, s'),
    Spec('station_tolerance', 1.5, low=0.2, high=10, step=0.1, group='Navigation', note='radius, m'),
    _timing('neutral', 20.0, 'characterization: neutral s to rate the water first'),
    Spec('auto_tab', True, bool, group='Navigation', note='switch to Map when Navigate or Mission starts'),
    Spec('agents', 'allow', str, choices=('allow', 'block'), group='Control', reset=True,
         note='block refuses new agent claims; allow again at start'),
    Spec('agent_motion', 'confirm', str, choices=('confirm', 'allow', 'observe'), group='Control', reset=True,
         note='agent run/goal: confirm asks Enter each time, observe refuses; confirm again at start'),
    Spec('direction', '', str, group='Trials', reset=True,
         note='characterization: re-refine only this column, e.g. sway+'),
    Spec('dock_step', 'weigh_in', str, choices=('weigh_in',), group='Physics',
         note='Dock physics (d) runs this step; config/physics.yaml holds its parameters'),
)
BY_KEY = {spec.key: spec for spec in SPECS}
RETIRED = ('agent_view',)   # still in older settings files; read past, never an error


class Settings:
    """The current values; ``Settings(settle=2)`` overrides defaults, validated."""

    def __init__(self, **values):
        for spec in SPECS:
            setattr(self, spec.key, spec.default)
        self.update(values)

    def update(self, values):
        for key, value in values.items():
            if key not in BY_KEY:
                raise ValueError(f'unknown setting {key}')
            setattr(self, key, BY_KEY[key].parse(value))

    def set(self, assignment):
        """Apply ``key=value`` (``--set``)."""
        key, separator, value = assignment.partition('=')
        if not separator:
            raise ValueError(f'{assignment!r}: expected key=value')
        self.update({key.strip(): value.strip()})

    def persistent(self):
        return {s.key: getattr(self, s.key) for s in SPECS if not s.reset}


def settings_path():
    base = os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config'
    return Path(base) / 'mhseals' / 'boat_test.yaml'


def load(path=None, strict=False):
    """
    Settings from the user's file; a broken file or value falls back to the default.

    ``strict`` (the hot reload) raises on a broken file or value instead; a missing file is the
    defaults.
    """
    settings = Settings()
    try:
        saved = load_strict(Path(path or settings_path()).read_text())
    except FileNotFoundError:
        return settings
    except (OSError, ValueError, yaml.YAMLError):
        if strict:
            raise
        return settings
    if strict and saved is not None and not isinstance(saved, dict):
        raise ValueError('expected a mapping')
    for key, value in (saved if isinstance(saved, dict) else {}).items():
        spec = BY_KEY.get(key)
        if spec is None and strict and key not in RETIRED:
            raise ValueError(f'unknown setting {key}')
        if spec is None or spec.reset:
            continue
        try:
            setattr(settings, key, spec.parse(value))
        except ValueError:
            if strict:
                raise
    return settings


def save(settings, path=None):
    """Write the persistent values; False when the file can't be written."""
    path = Path(path or settings_path())
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(settings.persistent()))
        temporary.replace(path)
    except OSError:
        return False
    return True
