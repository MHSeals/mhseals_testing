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

import numpy
import yaml

from mhseals_hardware.configuration import load_config
from mhseals_hardware.thruster_mixer import (
    decouple, thruster_pulses, validate_channel_map, validate_decoupling, validate_mixer, validate_reverse_ratio,
    validate_reversed,
)
from mhseals_testing import data, physics_record
from mhseals_testing.analysis import physics
from mhseals_testing.strict_yaml import load_strict

DOCK_STEPS = ('weigh_in', 'dock_noise')
THRUST_STEPS = ('bollard', 'pulses', 'survey', 'maneuvers')


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


def pulses(ctx, cfg):
    """Single-thruster pulses afloat: each output alone, both ways, from rest; the onset acceleration per axis.

    The experiments are (cmd4, W_hat, sigma) for crane's estimate_mixer: cmd4 is the canonical
    effort per thruster in FL FR RR RL order (the motor effort, flipped where ``reversed``), W_hat
    the body acceleration [ax, ay, alpha] the pulse started, net of the drift before it.
    """
    driver = require_driver(ctx)
    log = Log(ctx)
    names = output_names(log.config)
    ctx.prompt(f'Boat afloat, untied, clear water all round. Each output pulses alone to '
               f'{", ".join(f"{level:+.0%}" for level in cfg["levels"])} for {cfg["hold_s"]:g} s from '
               f'{cfg["rest_s"]:g} s of rest. Press Enter... ', 'Single-thruster pulses')
    runs = []
    try:
        with ctx.motion('physics_pulses', levels=list(cfg['levels'])) as runner:
            for output in cfg['outputs']:
                for level in cfg['levels']:
                    pulse = [1500] * 4
                    pulse[output - 1] = round(1500 + 400 * level)
                    log.pulse(runner, [1500] * 4, cfg['rest_s'], 'rest')
                    segment = log.pulse(runner, pulse, cfg['hold_s'], f'output {output} {level:+.0%}')
                    runs.append((output, level, segment))
            log.pulse(runner, [1500] * 4, cfg['rest_s'], 'rest')
    finally:
        ctx.services['pulses'](None)
    rows = [(t, v.get('vx'), v.get('vy'), v.get('wz'))
            for t, v in ctx.history(log.segments[0]['start'], log.segments[-1]['end'])]
    experiments, skipped = [], []
    floor = cfg['sigma_floor']
    for output, level, segment in runs:
        found = None if segment['overridden'] else physics.onset(
            rows, segment['start'], cfg['skip_s'], cfg['edge_s'], cfg['rest_s'] / 2)
        if found is None:
            skipped.append(segment['label'])
            continue
        name, flipped = names[output]
        cmd4 = [0.0] * 4
        cmd4[physics_record.THRUSTERS.index(name)] = -level if flipped else level
        experiments.append(dict(output=output, thruster=name, motor_effort=level, cmd4=cmd4, W_hat=found[0],
                                sigma=[max(s, f) for s, f in zip(found[1], floor)]))
    if not experiments:
        raise Refused('no pulse left a usable onset: check odometry')
    body = dict(experiments=experiments, skipped=skipped, order=list(physics_record.THRUSTERS),
                window=dict(skip_s=cfg['skip_s'], edge_s=cfg['edge_s'], before_s=cfg['rest_s'] / 2),
                units=dict(cmd4='normalized canonical effort', W_hat='m/s^2, m/s^2, rad/s^2 (body)',
                           sigma='same as W_hat'),
                counts=dict(experiments=len(experiments), skipped=len(skipped), samples=len(rows), **log.counts()),
                summary=dict(experiments=len(experiments), skipped=len(skipped)))
    return body, log.samples(), driver


AXES = ('surge', 'sway', 'yaw')
KEYS = ('vx', 'vy', 'wz')


def applied(command, config):
    """The efforts the driver mixes for ``command`` under the fit-time C+/C- (the deployed correction)."""
    return list(decouple(command, validate_decoupling(config['decoupling']),
                         validate_decoupling(config['decoupling_negative'] or config['decoupling'])))


def hold_stats(rows, segment, share):
    """Means and SDs of vx, vy, wz (and the mean yaw) over the last ``share`` of a hold."""
    begin = segment['end'] - share * (segment['end'] - segment['start'])
    inside = [v for t, v in rows if begin <= t <= segment['end']]
    stats = dict(n=len(inside))
    for key in KEYS + ('yaw',):
        values = [v[key] for v in inside if v.get(key) is not None and math.isfinite(v[key])]
        stats[key] = float(numpy.mean(values)) if values else None
        if key != 'yaw':
            stats[key + '_sd'] = float(numpy.std(values, ddof=1)) if len(values) > 1 else None
    return stats


def latest(ctx, step):
    """The newest record of ``step`` in the characterization folder, or None."""
    root = Path(ctx.services.get('physics_root') or data.path('characterization'))
    found = sorted(root.glob(f'*-{step}.yaml'))
    return yaml.safe_load(found[-1].read_text()) if found else None


def survey(ctx, cfg):
    """Open water: every signed axis at each level, repeated; the neutral after each hold is its coast-down.

    Runs the deployed C (restart-only in the driver, never staged here): the main-axis effort the
    curves use is the one the driver applies under the fit-time C+/C-, recorded per hold with the
    cross efforts C added. Per signed axis it saves the speed curve with its covariance, residuals
    and repeat spread, the coast-down D/M fit, and every hold's means; the run's neutral tails give
    the wave floor, split against the latest dock_noise sigma when there is one.
    """
    driver = require_driver(ctx)
    log, amplitude = Log(ctx), ctx.settings.amplitude
    levels = [level * amplitude for level in cfg['levels']]
    if len(set(levels)) < 4 or cfg['repeats'] < 2:
        raise Refused('the survey needs at least 4 distinct levels and 2 repeats (config/physics.yaml)')
    conditions = ctx.services.get('conditions')
    if conditions:
        water = conditions.summary(ctx.monitor.clock())
        if water.get('verdict') == 'poor':
            raise Refused(f'conditions poor: {water.get("reason")}')
    ctx.prompt(f'Open water, at least 30 m clear all round. {len(levels)} levels x {cfg["repeats"]} repeats x '
               f'6 directions, {cfg["hold_s"]:g} s holds with {cfg["settle_s"]:g} s coasts. Press Enter... ',
               'Speed survey')
    holds = []
    with ctx.motion('physics_survey', levels=levels, repeats=cfg['repeats']) as runner:
        log.hold(runner, (0.0, 0.0, 0.0), cfg['settle_s'], 'neutral')
        for repeat in range(cfg['repeats']):
            if repeat and cfg['heading_spread']:
                turn(ctx, log, runner, 2 * math.pi / cfg['repeats'], cfg)
            for level in (levels if repeat % 2 == 0 else levels[::-1]):
                for axis in range(3):
                    for sign in (1, -1):
                        command = [0.0] * 3
                        command[axis] = sign * level
                        name = AXES[axis] + ('+' if sign > 0 else '-')
                        segment = log.hold(runner, tuple(command), cfg['hold_s'], f'{name} {level:.2f}')
                        segment.update(axis=axis, sign=sign, level=level, repeat=repeat,
                                       applied=applied(command, log.config))
                        coast = log.hold(runner, (0.0, 0.0, 0.0), cfg['settle_s'], 'coast')
                        holds.append((segment, coast))
                        if repeat == 0 and level == levels[0] and sign == 1:
                            moved = ctx.history(segment['start'], segment['end'])
                            if moved and moved[-1][1].get(KEYS[axis], 0.0) * sign <= 0:
                                runner.stop()
                                raise Refused(f'{AXES[axis]} responds backwards: check wiring, run Thrusters')
    rows = ctx.history(log.segments[0]['start'], log.segments[-1]['end'])
    per_hold, curves, coasts = [], {}, {}
    for segment, coast in holds:
        stats = hold_stats(rows, segment, cfg['steady_share'])
        segment['stats'] = stats
        per_hold.append(dict(name=segment['label'], start=segment['start'], end=segment['end'],
                             command=segment['command'], applied=segment['applied'], repeat=segment['repeat'],
                             overridden=segment['overridden'], **stats))
    for axis in range(3):
        for sign in (1, -1):
            name = AXES[axis] + ('+' if sign > 0 else '-')
            mine = [(s, c) for s, c in holds if s['axis'] == axis and s['sign'] == sign and not s['overridden']
                    and s['stats'][KEYS[axis]] is not None]
            curves[name] = physics.speed_fit([(sign * s['stats'][KEYS[axis]], sign * s['applied'][axis], s['level'])
                                              for s, _ in mine])
            top = sorted({s['level'] for s, _ in mine})[-cfg['coast_levels']:]
            windows = [[(t, sign * v[KEYS[axis]]) for t, v in rows
                        if c['start'] + cfg['coast_skip_s'] <= t <= c['end'] and v.get(KEYS[axis]) is not None]
                       for s, c in mine if s['level'] in top]
            coasts[name] = physics.coast_fit(windows, cfg['coast_span_s'], cfg['coast_floor'][axis])
    floor = neutral_floor(rows, [c for _, c in holds], cfg)
    noise = (latest(ctx, 'dock_noise') or {}).get('noise', {})
    env = {key: dict(zip(('value', 'ok'), physics.env_floor(floor[key], noise[f'odometry.{key}']['sigma'])))
           for key in KEYS if floor.get(key) is not None and f'odometry.{key}' in noise}
    headings = [h['yaw'] for h in per_hold if h['yaw'] is not None and not h['overridden']]
    spread = physics.heading_spread(headings)
    body = dict(amplitude=amplitude, levels=levels, repeats=cfg['repeats'], speed_curves=curves, coast_down=coasts,
                holds=per_hold, wave_floor=floor, env_floor=env or None,
                heading_spread=dict(R=spread, ok=spread is not None and spread < cfg['heading_R_max'],
                                    max=cfg['heading_R_max'], n=len(headings)),
                model=dict(curve='e|e| = d1 v + d2 v|v| (e: applied main-axis effort, v: hold mean)',
                           coast='-dv/dt = c0 + d1_over_m v + d2_over_m v|v| (thrust off)'),
                units=dict(effort='normalized', v='m/s (surge, sway), rad/s (yaw)', d1='effort^2 per v',
                           d2='effort^2 per v^2', d1_over_m='1/s', d2_over_m='1/m (surge, sway), 1/rad (yaw)',
                           c0='v per s', wave_floor='per-sample SD, same as v', time='s, monitor clock'),
                counts=dict(holds=len(holds), samples=len(rows), **log.counts(),
                            per_direction={n: (c or {}).get('n', 0) for n, c in curves.items()}),
                summary=dict(directions={n: dict(d1=round(c['d1'], 4), d2=round(c['d2'], 4), dof=c['dof'])
                                         for n, c in curves.items() if c}))
    return body, log.samples(), driver


def turn(ctx, log, runner, angle, cfg):
    """Yaw the boat by about ``angle`` rad in short effort chunks, closed on odometry yaw, then settle.

    Between repeats, so each level is held at another heading and drift splits into wind and current
    (design 9 step 7). Stops at ``turn_max_s`` however far it got; the R it reached is in the record.
    """
    def yaw():
        rows = ctx.history(ctx.monitor.clock() - 1.0, ctx.monitor.clock())
        return next((v['yaw'] for _, v in reversed(rows) if v.get('yaw') is not None), None)
    begin, turned, elapsed = yaw(), 0.0, 0.0
    previous = begin
    while begin is not None and turned < angle and elapsed < cfg['turn_max_s']:
        log.hold(runner, (0.0, 0.0, cfg['turn_effort']), cfg['turn_chunk_s'], 'turn')
        elapsed += cfg['turn_chunk_s']
        now = yaw()
        if now is not None:
            turned += math.remainder(now - previous, 2 * math.pi)
            previous = now
    log.hold(runner, (0.0, 0.0, 0.0), cfg['settle_s'], 'neutral')
    return turned


def maneuvers(ctx, cfg):
    """Turning circles both ways and a zigzag at each speed, withheld: saved for validation, never fitted.

    They are the combined surge-yaw motion the single-axis steps never excite (C_A, a26). A zigzag
    holds yaw effort one way until the heading is ``zigzag_deg`` off the base heading, then the other,
    for ``zigzag_switches`` switches or ``zigzag_max_s``.
    """
    driver = require_driver(ctx)
    log, amplitude = Log(ctx), ctx.settings.amplitude
    ctx.prompt('Open water, 40 m clear all round: turning circles and zigzags at '
               f'{len(cfg["speeds"])} speeds. Press Enter... ', 'Maneuvers (withheld)')
    runs = []

    def yaw():
        rows = ctx.history(ctx.monitor.clock() - 1.0, ctx.monitor.clock())
        return next((v['yaw'] for _, v in reversed(rows) if v.get('yaw') is not None), None)

    with ctx.motion('physics_maneuvers', speeds=list(cfg['speeds'])) as runner:
        for speed in cfg['speeds']:
            surge = speed * amplitude
            for sign in (1, -1):
                log.hold(runner, (surge, 0.0, 0.0), cfg['approach_s'], 'approach')
                start = len(log.segments)
                log.hold(runner, (surge, 0.0, sign * cfg['circle_yaw'] * amplitude), cfg['circle_s'],
                         f'circle{"+" if sign > 0 else "-"} {surge:.2f}')
                runs.append(dict(kind='circle', surge=surge, sign=sign, segments=[start, len(log.segments)]))
                log.hold(runner, (0.0, 0.0, 0.0), cfg['settle_s'], 'coast')
            log.hold(runner, (surge, 0.0, 0.0), cfg['approach_s'], 'approach')
            base, sign, switches, elapsed = yaw(), 1, 0, 0.0
            start = len(log.segments)
            limit = math.radians(cfg['zigzag_deg'])
            while base is not None and switches < cfg['zigzag_switches'] and elapsed < cfg['zigzag_max_s']:
                log.hold(runner, (surge, 0.0, sign * cfg['zigzag_yaw'] * amplitude), cfg['chunk_s'],
                         f'zigzag{"+" if sign > 0 else "-"} {surge:.2f}')
                elapsed += cfg['chunk_s']
                now = yaw()
                if now is not None and sign * math.remainder(now - base, 2 * math.pi) >= limit:
                    sign, switches = -sign, switches + 1
            runs.append(dict(kind='zigzag', surge=surge, switches=switches, segments=[start, len(log.segments)]))
            log.hold(runner, (0.0, 0.0, 0.0), cfg['settle_s'], 'coast')
    body = dict(withheld=True, use='validation and C_A / a26 only: never fitted with the other steps',
                runs=runs, units=dict(surge='normalized effort', circle_yaw='normalized effort', zigzag_deg='deg',
                                      time='s, monitor clock', segments='index range into samples.segments'),
                counts=dict(runs=len(runs), samples=len(ctx.history(log.segments[0]['start'], log.segments[-1]['end'])),
                            **log.counts()),
                summary=dict(runs=len(runs)))
    return body, log.samples(), driver


def neutral_floor(rows, coasts, cfg):
    """Per-sample SD of each velocity over the settled tail of every coast, a line removed from each tail.

    The line takes a coast still dying out and a drift change; what is left is the wave floor.
    """
    residuals = {key: [] for key in KEYS}
    for coast in coasts:
        begin = coast['end'] - cfg['floor_tail_s']
        for key in KEYS:
            tail = [(t, v[key]) for t, v in rows if begin <= t <= coast['end'] and v.get(key) is not None]
            if len(tail) > 3:
                t, y = (numpy.array(c) for c in zip(*tail))
                residuals[key].extend(y - numpy.polyval(numpy.polyfit(t - t[0], y, 1), t - t[0]))
    return {key: float(numpy.sqrt(numpy.dot(r, r) / (len(r) - 2 * len(coasts)))) if len(r) > 2 * len(coasts)
            else None for key, r in residuals.items()}


STEPS = dict(weigh_in=weigh_in, dock_noise=dock_noise, bollard=bollard, pulses=pulses, survey=survey,
             maneuvers=maneuvers)


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
