"""Plant profiles: what the suite talks to (the real boat or a simulator).

A profile names the command topic and its meaning, the odometry and sensors to
watch, the frames, and how thrusters are armed. Procedures read the profile
instead of branching on "boat or sim", and the menu disables any procedure a
profile cannot run.

Profiles: ``boat`` (real PWM and sensors, MAVROS + EKFs), ``crane_full_stack`` (simulator through
navigation: EKFs without an FCU, sim time, ROS ``/thrusters/pwm``) and ``crane_direct`` (attach
only; Unity's own velocity controller and truth). Components without ``start: true`` start from
Status rows. Profiles that command ``/cmd_vel`` start ``mhseals_hardware``'s ``command_mux`` on
arming; the driver, or the velocity controller in front of it, listens on ``/cmd_vel/applied``.
Each component is a device part (mhseals_nav config/devices/): its launch, args and host come from
the device registry (mhseals_testing.devices); ``placement: {self: [sim]}`` runs the named device
profiles on this machine instead. Odometry and endpoint startup defaults come from
``mhseals_nav/config/{odometry,endpoint}_startup.yaml`` (real boat: ``included`` context;
simulator: ``standalone``), and conflicting selections refuse before any process starts.
"""

from dataclasses import dataclass, replace
from functools import cached_property
from pathlib import Path

from mhseals_testing import data as records
from mhseals_testing.analysis.power import CHEMISTRIES
from mhseals_testing.harness.components import parse_components
from mhseals_testing.strict_yaml import load_strict
from mhseals_testing.validation import FUTURE_TOLERANCE


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
SENSOR_KINDS = ('odometry', 'imu', 'gps', 'gpsraw', 'mag', 'points', 'image', 'compressed', 'detections')
MIN_DEADLINE = 1.5   # s, floor of the rate-derived freshness deadline (real GPS gaps reach 1.13 s)
PLOT_SCALE = (0.6, 0.3, 0.5)   # about 1.5x the limits_v2.yaml limits; fixed, never autoscaled
OUTPUTS = ('none', 'odroid', 'topic')


MUX_INPUT = '/cmd_vel'  # the topic Nav2, tasks and agents publish; the mux owns it
MANUAL_TOPIC = '/cmd_vel/manual'  # the mux input that outranks auto and yields to rc


@dataclass(frozen=True)
class SensorSpec:
    name: str
    topic: str
    kind: str
    required: bool = False
    hz: float | None = None            # nominal rate; None leaves rate and deadline unchecked
    stale_after: float | None = None   # explicit freshness deadline, normally derived from hz

    @property
    def deadline(self):
        """Seconds of silence before the sensor is stale: ``max(3/hz, 1.5)`` unless set."""
        if self.stale_after is not None:
            return self.stale_after
        return None if not self.hz else max(3 / self.hz, MIN_DEADLINE)


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
    course_truth: str | None     # the simulator's course truth JSON, for the referee
    map_frame: str | None        # None when the plant has no map frame
    odom_frame: str
    base_frame: str
    use_sim_time: bool
    outputs: str                 # thruster outputs to arm: none, odroid (pins) or topic (sim)
    velocity_controller: bool    # put calibrated_velocity_controller in front of the PWM node
    sensors: tuple
    components: dict | None = None   # stack components (harness.components); None attaches only
    battery_cells: int | None = None   # `battery: {cells: N}`; None auto-detects from the voltage
    battery_chemistry: str | None = None     # `battery: {chemistry}`: lipo, liion or lifepo4
    battery_capacity_mah: float | None = None    # `battery: {capacity_mah}`, for the time left

    @property
    def battery(self):
        """The configured keys as the dict ``analysis.power.Power`` takes."""
        given = {'cells': self.battery_cells, 'chemistry': self.battery_chemistry,
                 'capacity_mah': self.battery_capacity_mah}
        return {key: value for key, value in given.items() if value is not None}
    plot_scale: tuple = PLOT_SCALE     # Motion plot half-range for surge, sway (m/s) and yaw (rad/s)

    # s a sensor stamp may run ahead of the clock (a simulator's link jitters)
    stamp_future: float = FUTURE_TOLERANCE
    conditions_velocity: str = 'gpsraw'  # explicit odom:<topic|name> is simulation-only

    placement: dict | None = None    # {self: [device profiles]}: all on this machine; None: the registry's roles

    @cached_property
    def placed(self):
        """(this machine's devices.Host, {part: devices.Part}) for the components; DeviceError if refused."""
        from mhseals_testing import devices
        wanted = [spec.get('part', key) for key, spec in (self.components or {}).items()]
        return devices.resolve(self.placement, wanted)

    @property
    def uses_fcu(self):
        """True when the odometry component brings up MAVROS (a real flight controller)."""
        spec = (self.components or {}).get('odometry')
        if spec is None:
            return False
        return self.placed[1][spec.get('part', 'odometry')].arguments.get('fcu', 'true') != 'false'

    @property
    def muxed(self):
        """True when a command_mux child arbitrates the command topic when armed."""
        return self.outputs != 'none' and self.command_topic == MUX_INPUT

    @property
    def manual_topic(self):
        """Where the TUI's own keys publish: the mux's manual input, else the command topic."""
        return MANUAL_TOPIC if self.muxed else self.command_topic

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


def _positive(data, key, where):
    """Optional positive finite number, or None when absent."""
    value = data.get(key)
    if value is None:
        return None
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not 0 < value < float('inf')):
        raise ValueError(f'{where}.{key}: positive number')
    return float(value)


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
                            _require(spec, 'kind', str, here), bool(spec.get('required', False)),
                            _positive(spec, 'hz', here), _positive(spec, 'stale_after', here))
        if sensor.kind not in SENSOR_KINDS:
            raise ValueError(f'{here}.kind: one of {SENSOR_KINDS}')
        sensors.append(sensor)
    if len({sensor.name for sensor in sensors}) != len(sensors):
        raise ValueError(f'{where}.sensors: names must be unique')
    conditions = data.get('conditions', {})
    if not isinstance(conditions, dict):
        raise ValueError(f'{where}.conditions: mapping required')
    velocity = conditions.get('velocity', 'gpsraw')
    if velocity != 'gpsraw':
        matches = [s for s in sensors if isinstance(velocity, str)
                   and velocity.startswith('odom:') and velocity[5:] in (s.name, s.topic)
                   and s.kind == 'odometry']
        if not data.get('use_sim_time') or len(matches) != 1:
            raise ValueError(f'{where}.conditions.velocity: gpsraw or unique sim odom:<topic|name>')
    components = parse_components(data.get('components'), where + '.components')
    if components and type(data.get('use_sim_time')) is not bool:
        raise ValueError(f'{where}.use_sim_time: managed components require a boolean')
    placement = data.get('placement')
    if placement is not None and not (
            isinstance(placement, dict) and set(placement) == {'self'}
            and isinstance(placement['self'], list) and placement['self']
            and all(isinstance(name, str) for name in placement['self'])):
        raise ValueError(f'{where}.placement: {{self: [device profiles]}}')
    scale = tuple(float(value) for value in data.get('plot_scale', PLOT_SCALE))
    if len(scale) != 3 or not all(0 < value < float('inf') for value in scale):
        raise ValueError(f'{where}.plot_scale: three positive numbers')
    battery = data.get('battery') or {}
    cells = battery.get('cells')
    if cells is not None and (not isinstance(cells, int) or isinstance(cells, bool) or cells < 1):
        raise ValueError(f'{where}.battery.cells: positive integer')
    chemistry, capacity = battery.get('chemistry'), battery.get('capacity_mah')
    if chemistry is not None and chemistry not in CHEMISTRIES:
        raise ValueError(f'{where}.battery.chemistry: one of {sorted(CHEMISTRIES)}')
    if capacity is not None and (isinstance(capacity, bool) or not isinstance(capacity, (int, float))
                                 or not 0 < capacity < float('inf')):
        raise ValueError(f'{where}.battery.capacity_mah: positive number')
    return PlantProfile(
        name=_require(data, 'name', str, where),
        description=str(data.get('description', '')),
        command_topic=_require(command, 'topic', str, where + '.command'),
        command_type=command_type, command_units=units, command_limit=limit,
        odometry=_require(data, 'odometry', str, where),
        ground_truth=data.get('ground_truth'),
        course_truth=_under_data_root(data.get('course_truth')),
        map_frame=frames.get('map'),
        odom_frame=_require(frames, 'odom', str, where + '.frames'),
        base_frame=_require(frames, 'base', str, where + '.frames'),
        use_sim_time=bool(data.get('use_sim_time', False)),
        outputs=outputs, velocity_controller=controller,
        sensors=tuple(sensors), components=components or None, placement=placement, battery_cells=cells,
        battery_chemistry=chemistry, battery_capacity_mah=capacity,
        plot_scale=scale, stamp_future=_positive(data, 'stamp_future', where) or FUTURE_TOLERANCE,
        conditions_velocity=velocity)


def available_profiles(directory=PROFILE_DIR):
    return sorted(path.stem for path in Path(directory).glob('*.yaml'))


def _under_data_root(path):
    """A relative profile path resolves under the data root ($MHSEALS_DATA, else $ROS_WS/bags)."""
    if not path or Path(path).is_absolute():
        return path
    return str(records.path(path))


def load_profile(name_or_path, directory=PROFILE_DIR):
    """Load a shipped profile by name, or any profile YAML by path."""
    path = Path(name_or_path)
    if path.suffix != '.yaml':
        path = Path(directory) / f'{name_or_path}.yaml'
    if not path.exists():
        raise FileNotFoundError(f'no profile {name_or_path!r}; shipped: {available_profiles(directory)}')
    return parse_profile(load_strict(path.read_text()), where=path.name)


def profile_descriptions(directory=PROFILE_DIR):
    """Read explanatory names from the same YAML as the executable profiles."""
    return [(name, load_profile(name, directory).description)
            for name in available_profiles(directory)]
