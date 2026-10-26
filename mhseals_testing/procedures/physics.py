"""Physics characterization: the dock and open-water steps the crane physics import needs.

Two menu entries, each running the step its setting names (Session rows, persisted):
  Dock physics  (``dock_step``)   weigh_in, then the other dock steps; nothing moves.
  Thrust physics (``thrust_step``) the steps that drive thrusters.
No step changes the correction C+/C-: the driver's C is restart-only, so every motion step runs
the deployed C and records it (with the driver's own report of the file it runs) at fit time.
Each step writes one record (physics_record.py) and records its summary in the session.
"""

import math
from pathlib import Path

from mhseals_hardware.configuration import load_config
from mhseals_hardware.thruster_mixer import (
    thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer, validate_reverse_ratio,
    validate_reversed,
)
from mhseals_testing import physics_record
from mhseals_testing.analysis import physics
from mhseals_testing.strict_yaml import load_strict

DOCK_STEPS = ('weigh_in', 'dock_noise')
THRUST_STEPS = ('bollard',)


class Refused(ValueError):
    """A step cannot run or its input cannot be used; the message says why."""


def load_settings(path=None):
    """config/physics.yaml from the source tree, else the installed share directory."""
    path = Path(path or Path(__file__).resolve().parents[2] / 'config' / 'physics.yaml')
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'physics.yaml'
    return load_strict(path.read_text(), path)


def refuse(ctx, step, reason):
    refusal = dict(procedure='physics', step=step, verdict='REFUSED', reason=reason)
    ctx.record(refusal)
    return refusal


def ask_number(ctx, title, message, low, high, optional=False):
    """One number typed in the TUI's text popup, within [low, high]; None for an optional blank."""
    ask = ctx.services.get('ask_text')
    if ask is None:
        raise Refused('needs the TUI text entry')

    def check(text):
        if not text.strip():
            if optional:
                return None
            raise ValueError('a value is required')
        value = float(text)
        if not low <= value <= high:
            raise ValueError(f'expected {low:g} to {high:g}')
        return value
    text = ask(title, message, '', check)
    if text is None:
        raise Refused('cancelled by operator')
    return check(text)


def weigh_in(ctx, cfg):
    """Mass and CG typed from the scale and the balance points; I_z from a bifilar swing if one was done."""
    low, high = cfg['mass_kg']
    span = cfg['cg_m']
    mass = ask_number(ctx, 'Weigh-in', 'Boat mass as it runs (battery, computers, lid), kg', low, high)
    cg = [ask_number(ctx, 'Centre of gravity', f'CG {axis} from base_link, m ({sense})', -span, span)
          for axis, sense in (('x', 'forward +'), ('y', 'left +'), ('z', 'up +'))]
    shortest, longest = cfg['bifilar_m']
    period = ask_number(ctx, 'Yaw inertia (optional)',
                        'Bifilar swing: mean period of 10 swings / 10, s (blank if not swung)', 0.2, 30.0, True)
    iz = None
    if period is not None:
        separation = ask_number(ctx, 'Yaw inertia', 'String separation, m', shortest, longest)
        length = ask_number(ctx, 'Yaw inertia', 'String length, m', shortest, longest)
        iz = dict(value=physics.bifilar_iz(mass, period, separation, length), method='bifilar',
                  period_s=period, separation_m=separation, length_m=length)
    body = dict(mass=mass, cg=cg, iz=iz or dict(value=None, method='none: prior'),
                units=dict(mass='kg', cg='m (base_link x fwd, y left, z up)', iz='kg m^2', period_s='s'),
                counts=dict(entries=4 + (3 if iz else 0)))
    return body, None, None


def require_driver(ctx):
    """The driver's report of the file it runs, or Refused: motion steps name the mixer and C they ran."""
    status = (ctx.services.get('driver_status') or (lambda: None))()
    hardware = getattr(getattr(ctx, 'profile', None), 'outputs', None) == 'odroid'
    driver, reason = physics_record.driver_check(status, ctx.services.get('user_config'), hardware)
    if reason:
        raise Refused(reason)
    return driver


def commanded_pulses(command, config):
    """The pulses per output the driver makes of an effort command under ``config`` (its C, mixer, flips)."""
    return list(thruster_pulses(
        command, validate_mixer(config['thruster_matrix']), validate_channel_map(config['channel_map']),
        validate_reversed(config['reversed']), validate_decoupling(config['decoupling']),
        validate_decoupling(config['decoupling_negative'] or config['decoupling']),
        validate_reverse_ratio(config['reverse_thrust_ratio'])))


class Log:
    """One step's segments and samples on the monitor clock (design 9 step 2, synced logging).

    Each segment names what was sent (an effort command through the driver, or raw pulses per
    output), the pulses that makes under the fit-time config, the battery at its start and end and
    whether an override touched it. ``samples()`` adds the odometry and the raw gyro over the run,
    the gyro kept apart for the estimator-lag check. The driver publishes no pulse readback, so
    ``pulse_source`` says these are commanded pulses.
    """

    def __init__(self, ctx, config=None):
        self.ctx, self.segments = ctx, []
        self.config = config if config is not None else load_config(ctx.services['user_config'])

    def battery(self):
        context = dict((self.ctx.services.get('fit_context') or dict)())
        return [self.ctx.monitor.clock(), context.get('battery_v')]

    def _segment(self, runner, label, start, end, **sent):
        invalid = getattr(runner, 'invalid', None)
        self.segments.append(dict(label=label, start=start, end=end, **sent,
                                  overridden=bool(invalid and invalid(start, end))))
        return self.segments[-1]

    def hold(self, runner, command, seconds, label):
        """An effort hold through the driver (its deployed C applies)."""
        battery = self.battery()
        start, end = runner.hold(command, seconds, label)
        return self._segment(runner, label, start, end, command=list(command),
                             pulses=commanded_pulses(command, self.config), battery=[battery, self.battery()])

    def pulse(self, runner, pulses, seconds, label, sample=None):
        """Raw pulses per output for ``seconds`` (~/test_pulses: no C, no mixer)."""
        battery = self.battery()
        self.ctx.services['pulses'](list(pulses))
        start = self.ctx.monitor.clock()
        runner.supervise(seconds, label=label, sample=sample)
        return self._segment(runner, label, start, self.ctx.monitor.clock(), pulses=list(pulses),
                             battery=[battery, self.battery()])

    def samples(self, margin=1.0):
        """{segments, odometry [t, vx, vy, wz, yaw], gyro [t, wz]} over the run."""
        if not self.segments:
            return dict(clock='monitor', segments=[], odometry=[], gyro=[])
        since, until = self.segments[0]['start'] - margin, self.segments[-1]['end'] + margin
        odometry = [[t, *(v.get(k) for k in ('vx', 'vy', 'wz', 'yaw'))]
                    for t, v in self.ctx.history(since, until)]
        gyro_name = getattr(self.ctx.monitor, 'gyro_sensor', None)
        gyro = [[t, v.get('wz')] for t, v in self.ctx.history(since, until, gyro_name)] if gyro_name else []
        return dict(clock='monitor receive time, s', pulse_source='commanded', segments=self.segments,
                    odometry=odometry, gyro=gyro)

    def counts(self):
        return dict(segments=len(self.segments),
                    overridden=sum(segment['overridden'] for segment in self.segments))


CHANNELS = (('odometry', 'vx'), ('odometry', 'vy'), ('odometry', 'wz'), ('gyro', 'wz'))


def dock_noise(ctx, cfg):
    """Sensor sigma per channel with the boat tied tight and still: the part of the wave floor that is sensor."""
    ctx.prompt(f'Boat tied tight at the dock, disarmed, nobody aboard or touching lines. '
               f'{cfg["seconds"]:g} s of recording. Press Enter... ', 'Dock noise')
    with ctx.observe('physics_dock_noise') as runner:
        start = ctx.monitor.clock()
        runner.supervise(cfg['seconds'], label='recording sensor noise (tied)')
        end = ctx.monitor.clock()
    gyro_name = getattr(ctx.monitor, 'gyro_sensor', None)
    series = dict(odometry=ctx.history(start, end), gyro=ctx.history(start, end, gyro_name) if gyro_name else [])
    noise = {}
    for source, key in CHANNELS:
        rows = [(t, v.get(key)) for t, v in series[source]]
        found = physics.channel_noise(*zip(*rows)) if rows else None
        if found:
            noise[f'{source}.{key}'] = found
    if 'odometry.vx' not in noise:
        raise Refused('too few odometry samples in the recording')
    body = dict(window=[start, end], noise=noise,
                units=dict(sigma='m/s for vx, vy; rad/s for wz', lag1='-', window='s, monitor clock'),
                counts={name: found['n'] for name, found in noise.items()},
                summary=dict(sigma={name: round(found['sigma'], 5) for name, found in noise.items()}))
    samples = dict(clock='monitor receive time, s', odometry=[[t, *(v.get(k) for k in ('vx', 'vy', 'wz'))]
                                                              for t, v in series['odometry']],
                   gyro=[[t, v.get('wz')] for t, v in series['gyro']])
    return body, samples, None


def output_names(config):
    """{physical output: (thruster name, reversed)} from the channel map (canonical FL FR RR RL -> output)."""
    return {output: (physics_record.THRUSTERS[index], bool(config['reversed'][index]))
            for index, output in enumerate(config['channel_map'])}


def bollard(ctx, cfg):
    """Per-thruster pull on a scale: each output alone through a forward and reverse sweep, read after each hold."""
    driver = require_driver(ctx)
    log, scale = Log(ctx), cfg['scale_unit']
    to_newton = physics.G if scale == 'kgf' else 1.0
    names = output_names(log.config)
    ctx.prompt('Boat tied with the scale in the line (or the thruster on its rig). Each output pulls alone at '
               f'each level for {cfg["hold_s"]:g} s; read the steady {scale} during the hold and type it after. '
               'Press Enter... ', 'Bollard pull')
    readings = []
    try:
        for output in cfg['outputs']:
            name, flipped = names[output]
            angle = ask_number(ctx, 'Bollard pull', f'Output {output} ({name}): angle between the line and the '
                               'thruster axis, deg (0 on a rig)', 0.0, 80.0)
            for sign in (1, -1):
                for level in cfg['levels']:
                    pulse = round(1500 + sign * 400 * level)
                    pulses = [1500] * 4
                    pulses[output - 1] = pulse
                    with ctx.motion('physics_bollard', output=output, pulse=pulse) as runner:
                        segment = log.pulse(runner, pulses, cfg['hold_s'], f'output {output} {pulse} us')
                        log.pulse(runner, [1500] * 4, cfg['rest_s'], 'neutral')
                    ctx.services['pulses'](None)
                    if segment['overridden']:
                        continue
                    value = ask_number(ctx, 'Bollard pull', f'Output {output} ({name}) at {pulse} us: '
                                       f'steady reading, {scale}', 0.0, cfg['max_reading'])
                    force = value * to_newton / math.cos(math.radians(angle))
                    segment.update(reading=value, angle_deg=angle, force_n=force)
                    readings.append((output, pulse, force))
    finally:
        ctx.services['pulses'](None)
    fits = physics.bollard_fit(readings, cfg['resolution'] * to_newton)
    thrusters = {str(output): dict(fit, thruster=names[output][0], reversed=names[output][1])
                 for output, fit in fits.items()}
    voltages = [b for s in log.segments for _, b in s['battery'] if b is not None]
    body = dict(thrusters=thrusters, readings=[list(r) for r in readings], scale_unit=scale,
                resolution_n=cfg['resolution'] * to_newton,
                pack_v=dict(min=min(voltages), max=max(voltages)) if voltages else None,
                time_constants='not measured: a spring scale has no time series (tau_T, t_d stay prior)',
                units=dict(force='N', K='N at full pulse (u = 1)', pulse='us', db_us='us', o_us='us', pack_v='V'),
                counts=dict(readings=len(readings), **log.counts()),
                summary=dict(K_f={k: round(v['forward']['K'], 2) for k, v in thrusters.items() if v['forward']},
                             K_r={k: round(v['reverse']['K'], 2) for k, v in thrusters.items() if v['reverse']}))
    return body, log.samples(), driver


STEPS = dict(weigh_in=weigh_in, dock_noise=dock_noise, bollard=bollard)


def _run(ctx, step, allowed):
    if step not in allowed:
        return refuse(ctx, step, f'{step!r} is not one of {", ".join(allowed) or "none yet"}')
    cfg = load_settings()
    try:
        body, samples, driver = STEPS[step](ctx, cfg.get(step, {}))
        header = physics_record.provenance(ctx, step, driver=driver)
    except Refused as error:
        return refuse(ctx, step, str(error))
    path = physics_record.write(header, body, samples, ctx.services.get('physics_root'))
    result = dict(procedure='physics', step=step, verdict='SAVED', path=str(path),
                  counts=body['counts'], **body.get('summary', {}))
    ctx.record(result)
    return result


def dock(ctx):
    return _run(ctx, ctx.settings.dock_step, DOCK_STEPS)


def thrust(ctx):
    return _run(ctx, ctx.settings.thrust_step, THRUST_STEPS)
