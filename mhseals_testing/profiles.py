"""Plant profiles: what the suite talks to (the real boat or a simulator).

A profile names the command topic and its meaning, the odometry and sensors to
watch, the frames, and how thrusters are armed. Procedures read the profile
instead of branching on "boat or sim", and the menu disables any procedure a
profile cannot run.
"""

from dataclasses import dataclass, replace
from pathlib import Path

import yaml


def _profile_dir():
    """config/profiles in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parent.parent / 'config' / 'profiles'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'profiles'
    except (ImportError, KeyError):  # PackageNotFoundError is a KeyError
        return source


PROFILE_DIR = _profile_dir()
SENSOR_KINDS = ('odometry', 'imu', 'gps', 'points', 'image', 'compressed', 'detections')
OUTPUTS = ('none', 'odroid', 'topic')


@dataclass(frozen=True)
class SensorSpec:
    name: str
    topic: str
    kind: str
    required: bool = False


@dataclass(frozen=True)
class PlantProfile:
    name: str
    description: str
    command_topic: str
    command_type: str            # 'twist' or 'twist_stamped'
    command_units: str           # 'effort' (normalized thrust) or 'velocity' (m/s, rad/s)
    command_limit: tuple         # per-axis clamp for surge, sway, yaw
    odometry: str
    ground_truth: str | None     # simulator truth, for estimate-vs-truth error
    map_frame: str | None        # None when the plant has no map frame
    odom_frame: str
    base_frame: str
    use_sim_time: bool
    outputs: str                 # thruster outputs to arm: none, odroid (pins) or topic (sim)
    velocity_controller: bool    # put calibrated_velocity_controller in front of the PWM node
    bringup: dict | None         # measurement.launch.py arguments, or None to attach only
    sensors: tuple

    @property
    def can_arm(self):
        return self.outputs != 'none'

    @property
    def required_tf(self):
        chains = [(self.odom_frame, self.base_frame)]
        if self.map_frame:
            chains.insert(0, (self.map_frame, self.odom_frame))
        return tuple(chains)

    def with_velocity_control(self):
        """The same plant driven through the calibrated controller (m/s in, effort out)."""
        if not self.can_arm:
            raise ValueError(f'{self.name} has no thruster outputs to put a controller in front of')
        return replace(self, name=self.name + '+velocity', command_units='velocity',
                       velocity_controller=True)


def _require(data, key, kind, where):
    if key not in data:
        raise ValueError(f'{where}: missing {key}')
    value = data[key]
    if kind is not None and not isinstance(value, kind):
        raise ValueError(f'{where}.{key}: expected {getattr(kind, "__name__", kind)}')
    return value


def parse_profile(data, where='profile'):
    """Validate a profile mapping and return a PlantProfile."""
    if not isinstance(data, dict):
        raise ValueError(f'{where}: must be a mapping')
    command = _require(data, 'command', dict, where)
    command_type = _require(command, 'type', str, where + '.command')
    units = _require(command, 'units', str, where + '.command')
    limit = tuple(float(value) for value in _require(command, 'limit', list, where + '.command'))
    if command_type not in ('twist', 'twist_stamped'):
        raise ValueError(f'{where}.command.type: twist or twist_stamped')
    if units not in ('effort', 'velocity'):
        raise ValueError(f'{where}.command.units: effort or velocity')
    if len(limit) != 3 or any(value <= 0 for value in limit):
        raise ValueError(f'{where}.command.limit: three positive values')
    frames = _require(data, 'frames', dict, where)
    arming = _require(data, 'arming', dict, where)
    outputs = _require(arming, 'outputs', str, where + '.arming')
    if outputs not in OUTPUTS:
        raise ValueError(f'{where}.arming.outputs: one of {OUTPUTS}')
    controller = bool(arming.get('velocity_controller', False))
    if controller and (outputs == 'none' or units != 'velocity'):
        raise ValueError(f'{where}.arming.velocity_controller needs outputs and velocity units')
    sensors = []
    for index, spec in enumerate(_require(data, 'sensors', list, where)):
        here = f'{where}.sensors[{index}]'
        sensor = SensorSpec(_require(spec, 'name', str, here), _require(spec, 'topic', str, here),
                            _require(spec, 'kind', str, here), bool(spec.get('required', False)))
        if sensor.kind not in SENSOR_KINDS:
            raise ValueError(f'{here}.kind: one of {SENSOR_KINDS}')
        sensors.append(sensor)
    if len({sensor.name for sensor in sensors}) != len(sensors):
        raise ValueError(f'{where}.sensors: names must be unique')
    bringup = data.get('bringup')
    if bringup is not None and not isinstance(bringup, dict):
        raise ValueError(f'{where}.bringup: mapping of launch arguments or null')
    return PlantProfile(
        name=_require(data, 'name', str, where),
        description=str(data.get('description', '')),
        command_topic=_require(command, 'topic', str, where + '.command'),
        command_type=command_type, command_units=units, command_limit=limit,
        odometry=_require(data, 'odometry', str, where),
        ground_truth=data.get('ground_truth'),
        map_frame=frames.get('map'),
        odom_frame=_require(frames, 'odom', str, where + '.frames'),
        base_frame=_require(frames, 'base', str, where + '.frames'),
        use_sim_time=bool(data.get('use_sim_time', False)),
        outputs=outputs, velocity_controller=controller, bringup=bringup,
        sensors=tuple(sensors))


def available_profiles(directory=PROFILE_DIR):
    return sorted(path.stem for path in Path(directory).glob('*.yaml'))


def load_profile(name_or_path, directory=PROFILE_DIR):
    """Load a shipped profile by name, or any profile YAML by path."""
    path = Path(name_or_path)
    if path.suffix != '.yaml':
        path = Path(directory) / f'{name_or_path}.yaml'
    if not path.exists():
        raise FileNotFoundError(f'no profile {name_or_path!r}; shipped: {available_profiles(directory)}')
    return parse_profile(yaml.safe_load(path.read_text()), where=path.name)
