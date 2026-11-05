"""Characterization: one combined survey, then per-direction refinement of the correction columns.

The correction C+/C- is staged to identity first (harness/apply.py: live on the driver, read back), so every hold drives raw
efforts and the driver adds nothing; the survey sends each direction's deployed column itself. The survey holds each axis both ways at two amplitudes: it
gives the speed curves, the Nav2 limits and each direction's uncorrected cross response. Every
direction outside tolerance then gets its column refined (analysis/column.py). The result is
written to the hardware YAML only on the operator's answer, and always after the Nav2 overlay.
``--direction sway+`` skips the survey and re-refines that one column from the deployed one.
Any abort restores the saved C+/C-.
"""

from datetime import datetime, timezone
import math
import shlex

import yaml

from mhseals_hardware.configuration import load_config
from mhseals_hardware.thruster_mixer import AXES
from mhseals_testing import calibration, strict_yaml
from mhseals_testing.agent import ask
from mhseals_testing.nav2 import limits as nav2_limits
from mhseals_testing.analysis.characterization import (
    audit_holds, hold_t95, motion_limits, nav2_profile, neutral_noise, samples_from_windows,
)
from mhseals_testing.analysis.column import _within, refine, worst
from mhseals_testing.analysis.thrust import speed_curve
from mhseals_testing.harness.apply import IDENTITY, Staging
from mhseals_testing.health.config import config_path
from mhseals_testing.history import adapters
from mhseals_testing.runner import NEUTRAL

class UnusableResponse(ValueError):
    """A measurement or candidate cannot safely feed the characterization."""


DIRECTIONS = tuple((axis, sign) for axis in range(3) for sign in (1, -1))


def load_settings(path=None):
    """config/characterize.yaml."""
    return strict_yaml.load(path or config_path('config', 'characterize.yaml'))


def label(axis, sign):
    return AXES[axis] + ('+' if sign > 0 else '-')


def parse_direction(text):
    """'sway+' -> (1, 1); None for an empty or unknown name."""
    return next(((a, s) for a, s in DIRECTIONS if label(a, s) == text.strip().lower()), None)


def cross_axes(axis):
    return [k for k in range(3) if k != axis]


def command_for(axis, sign, amplitude, column=(0.0, 0.0)):
    """Raw efforts: the main axis at ``sign*amplitude`` plus the column's cross efforts (e_k = c * e_j)."""
    command = [0.0] * 3
    command[axis] = sign * amplitude
    for k, value in zip(cross_axes(axis), column):
        command[k] = value * sign * amplitude
    return tuple(command)


def _rows(history, window, skip):
    """The usable odometry samples of ``window`` minus its first ``skip`` s: (t, vx, vy, wz, values)."""
    start, end = window
    rows = []
    for stamp, values in history:
        if start + skip <= stamp <= end:
            v = [values.get(key) for key in ('vx', 'vy', 'wz')]
            if all(isinstance(x, (int, float)) and math.isfinite(x) for x in v):
                rows.append((stamp, *v, values))
    return rows


def window_evidence(rows, window, skip=0.0):
    """What one window measured, for the verdict: samples, coverage, displacement, speeds, receipt lag."""
    start, end = window[0] + skip, window[1]
    doc = dict(t0=round(start, 3), t1=round(end, 3), span_s=round(end - start, 3), n=len(rows))
    if not rows:
        return dict(doc, max_gap_s=round(end - start, 3))
    times = [start] + [row[0] for row in rows] + [end]
    doc['max_gap_s'] = round(max(b - a for a, b in zip(times, times[1:])), 3)
    doc['rate_hz'] = round(len(rows) / max(end - start, 1e-9), 1)
    doc['mean'] = [round(sum(row[k] for row in rows) / len(rows), 4) for k in (1, 2, 3)]
    doc['mean_speed'] = round(sum(math.hypot(row[1], row[2]) for row in rows) / len(rows), 4)
    first, last = rows[0][4], rows[-1][4]
    if all(isinstance(v.get(k), (int, float)) for v in (first, last) for k in ('x', 'y')):
        doc['displacement_m'] = round(math.hypot(last['x'] - first['x'], last['y'] - first['y']), 3)
    if all(isinstance(v.get('yaw'), (int, float)) for v in (first, last)):
        turn = (last['yaw'] - first['yaw'] + math.pi) % (2 * math.pi) - math.pi
        doc['yaw_change_deg'] = round(math.degrees(turn), 1)
    lags = [row[4]['stamp_age'] for row in rows if isinstance(row[4].get('stamp_age'), (int, float))]
    if lags:
        doc['lag_s'] = dict(mean=round(sum(lags) / len(lags), 3), max=round(max(lags), 3))
    return doc


def _covered(evidence, coverage):
    """'' when the window has enough odometry to judge it, else why not (characterize.yaml coverage)."""
    if not evidence['n'] or evidence.get('rate_hz', 0.0) < coverage['min_hz']:
        return f"{evidence['n']} samples in {evidence['span_s']:g} s (under {coverage['min_hz']:g} Hz)"
    if evidence['max_gap_s'] > coverage['max_gap_s']:
        return f"a {evidence['max_gap_s']:g} s gap (over {coverage['max_gap_s']:g} s)"
    return ''


def _world(row):
    """The sample's horizontal velocity in the odom frame (body twist rotated by its yaw), or None."""
    yaw = row[4].get('yaw')
    if not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    return c * row[1] - s * row[2], s * row[1] + c * row[2], yaw


def response(history, settle, hold, transient, coast=0.0, evidence=None, coverage=None):
    """Velocity change of a hold over the neutral before it, per body axis.

    The settle's mean velocity is the drift (current, wind) and lives in the odom frame: each hold sample has it
    subtracted in that frame and is rotated back into the body at its own heading, so a boat that turns
    during the hold does not turn a steady current into a response (11-04 pool: a waterfall current read
    as "sway responds backwards"). Without headings it is the plain body-frame difference.
    The first ``coast`` seconds of the settle are skipped: it still holds the previous hold's
    coast-down, whose sign is opposite in the paired order (at most half the settle is skipped).
    ``evidence`` (a dict) is filled with what both windows measured; ``coverage`` refuses thin data.
    """
    skip = min(coast, 0.5 * (settle[1] - settle[0]))
    before, during = _rows(history, settle, skip), _rows(history, hold, transient)
    measured = dict(settle=window_evidence(before, settle, skip), hold=window_evidence(during, hold, transient))
    if evidence is not None:
        evidence.update(measured)
    if not before or not during:
        raise UnusableResponse(f"no usable velocity samples in settle or hold (settle n={len(before)}, "
                               f"hold n={len(during)})")
    if coverage:
        for name in ('settle', 'hold'):
            thin = _covered(measured[name], coverage)
            if thin:
                raise UnusableResponse(f'odometry too thin to judge the {name}: {thin}')
    wz = sum(r[3] for r in during) / len(during) - sum(r[3] for r in before) / len(before)
    world_before, world_during = [_world(r) for r in before], [_world(r) for r in during]
    if all(world_before) and all(world_during):
        de = sum(w[0] for w in world_before) / len(before)
        dn = sum(w[1] for w in world_before) / len(before)
        body = [(math.cos(y) * (e - de) + math.sin(y) * (n - dn), -math.sin(y) * (e - de) + math.cos(y) * (n - dn))
                for e, n, y in world_during]
        delta = [sum(b[0] for b in body) / len(body), sum(b[1] for b in body) / len(body), wz]
        frame = 'drift removed in odom, body axes'
    else:
        delta = [sum(r[k] for r in during) / len(during) - sum(r[k] for r in before) / len(before)
                 for k in (1, 2)] + [wz]
        frame = 'body difference (no heading)'
    if not all(math.isfinite(value) for value in delta):
        raise UnusableResponse('nonfinite velocity response')
    if evidence is not None:
        evidence.update(response=[round(v, 4) for v in delta], frame=frame)
    return delta


def cross_response(delta, axis, sign):
    """(main response along the command, cross responses per unit of it) of a response vector."""
    main = max(sign * delta[axis], 1e-6)
    ratios = tuple(delta[k] / main for k in cross_axes(axis))
    if not all(math.isfinite(value) for value in (main, *ratios)):
        raise UnusableResponse('nonfinite cross response')
    return main, ratios


def matrices(columns, base=None):
    """Flat C+ and C-: identity with each direction's cross entries set from its column."""
    result = {sign: list(base[sign] if base else IDENTITY) for sign in (1, -1)}
    for (axis, sign), column in columns.items():
        for k, value in zip(cross_axes(axis), column):
            result[sign][3 * k + axis] = float(value)
    return result[1], result[-1]


def refuse(ctx, reason, measured=None):
    refusal = dict(procedure='characterization', verdict='REFUSED', reason=reason)
    if measured:
        refusal['measured'] = measured
    ctx.record(refusal)
    return refusal


def slices(wall, windows, end):
    """One journal window [t0_ns, t1_ns] per command, contiguous: each begins where the last ended.

    ``windows`` are the (command, start, end) holds, neutral settles included; a slice
    runs from the end of the previous command to the end of its own, so its settle
    belongs to it. The first starts with the run and the last ends at ``end``.
    """
    holds = [window for window in windows if tuple(window[0]) != NEUTRAL]
    edges = [windows[0][1]] + [hold[2] for hold in holds[:-1]] + [end]
    return [dict(command=list(hold[0]), window=[wall(t0), wall(t1)])
            for hold, t0, t1 in zip(holds, edges, edges[1:])]


def run(ctx):
    """The survey and refinement under staged identity C (the TUI's LiveStaging applies it on the driver)."""
    cfg, settings = load_settings(), ctx.settings
    only = parse_direction(settings.direction)
    if settings.direction and only is None:
        return refuse(ctx, f'unknown direction {settings.direction!r}: use e.g. sway+ or yaw-')
    staging = ctx.services.get('staging') or Staging(ctx.services['user_config'])
    hi, lo = (level * settings.amplitude for level in cfg['levels'])
    if cfg['refine']['headroom'] / hi - 1.0 < cfg['refine']['min_room']:
        return refuse(ctx, f'amplitude {settings.amplitude:g} leaves no room for the correction '
                           f'(a(1 + sum|c|) <= {cfg["refine"]["headroom"]:g}): use a smaller one')
    history, warn = ctx.services.get('history'), ctx.services.get('warn')
    if history:     # a hand edit is logged before staging rewrites the file
        adapters.observe_file(history, 'hardware', staging.config, warn)
    with staging.staged():      # first: a driver that refuses C stops the run before the operator is asked
        ctx.prompt(f'Characterization drives {"one direction" if only else "every direction"} for '
                   f'{settings.command:g}s holds. Clear all sides and press Enter... ')
        try:
            return _characterize(ctx, cfg, staging, only, hi, lo)
        except UnusableResponse as error:
            return refuse(ctx, str(error), getattr(error, 'measured', None))


def topic_of(ctx):
    """The odometry topic the procedure's history comes from, for the verdict."""
    return getattr(getattr(ctx, 'profile', None), 'odometry', None) or 'odometry'


def said(measure):
    """One line of numbers out of a hold's evidence: the response, samples, window, displacement."""
    hold = measure.get('hold') or {}
    parts = [f"hold {hold.get('t0', 0):.1f}-{hold.get('t1', 0):.1f} s", f"n={hold.get('n', 0)}"]
    if 'displacement_m' in hold:
        parts.append(f"moved {hold['displacement_m']:.2f} m")
    if 'mean_speed' in hold:
        parts.append(f"speed {hold['mean_speed']:.2f} m/s")
    return f"{measure.get('topic', 'odometry')}: " + ', '.join(parts)


def _characterize(ctx, cfg, staging, only, hi, lo):
    settings, windows, holds, measured = ctx.settings, [], [], []
    transient, tolerance = cfg['transient_s'], cfg['tolerance']
    coast = cfg['coast_s']
    deployed = staging.pending() or {}  # the C+/C- in force before this run
    base = {1: deployed.get('decoupling') or IDENTITY,
            -1: deployed.get('decoupling_negative') or deployed.get('decoupling') or IDENTITY}

    # Progress: one step per survey hold, one per direction refined (its refine holds vary).
    steps = {'done': 0, 'total': (0 if only else 2 * len(DIRECTIONS)) + (1 if only else len(DIRECTIONS))}

    def progress(runner, name, advance=0):
        steps['done'] += advance
        if hasattr(runner, 'progress'):
            runner.progress(steps['done'], steps['total'], name)

    def drive(runner, command, seconds, name):
        """Neutral then ``command``; returns the velocity change. The applied effort is logged."""
        if not all(math.isfinite(value) for value in command):
            runner.stop()
            raise UnusableResponse('nonfinite characterization command')
        progress(runner, name)
        settle = runner.hold(NEUTRAL, settings.settle, 'settle')
        hold = runner.hold(command, seconds, name)
        windows.extend([(NEUTRAL, *settle), (command, *hold)])
        holds.append(dict(command=list(command), start=hold[0], end=hold[1], label=name))
        evidence = dict(label=name, command=[round(c, 4) for c in command], topic=topic_of(ctx))
        measured.append(evidence)
        try:
            return response(ctx.history(settle[0], hold[1]), settle, hold, transient, coast, evidence,
                            cfg['coverage'])
        except UnusableResponse as error:
            error.measured = measured   # the refusal names what was (not) measured
            raise

    room = cfg['refine']['headroom'] / hi - 1.0  # largest sum|c| that keeps a_hi(1 + sum|c|) in range

    def deployed_column(axis, sign):
        """The deployed cross entries, scaled into the headroom like every other column."""
        return _within(tuple(base[sign][3 * k + axis] for k in cross_axes(axis)), room)

    survey, start = {}, None
    conditions, water = ctx.services.get('conditions'), None
    with ctx.motion('characterization', directions=1 if only else 6) as runner:
        if conditions:
            # Step 1: rate the water at neutral. This gates the start only, never the survey itself.
            start = runner.hold(NEUTRAL, settings.neutral, 'conditions')
            water = conditions.summary(runner.clock())
            if water['verdict'] == 'poor':
                runner.stop()
                refusal = dict(procedure='characterization', verdict='ABORTED', conditions=water,
                               reason=f'conditions poor: {water["reason"]}')
                ctx.record(refusal)
                return refusal
        if only is None:
            if water is None:
                start = runner.hold(NEUTRAL, cfg['neutral_s'], 'neutral')
            windows.append((NEUTRAL, *start))
            # Wiring gate: the first hold of each axis must move the boat the right way.
            for level in (hi, lo):
                for axis, sign in DIRECTIONS:
                    command = command_for(axis, sign, level, deployed_column(axis, sign))
                    delta = drive(runner, command, settings.command, label(axis, sign))
                    # Backwards only when the boat clearly went the other way: a weak or curving response
                    # (within the margin) is judged by the dead gate below, not called miswired.
                    margin = cfg['backwards_margin'][AXES[axis]]
                    if level == hi and sign == 1 and delta[axis] * math.copysign(1.0, command[axis]) < -margin:
                        runner.stop()
                        return refuse(ctx, f'{AXES[axis]} responds backwards ({delta[axis]:+.3f} against a '
                                           f'{command[axis]:+.2f} command, margin {margin:g}; {said(measured[-1])}): '
                                           'check wiring, run Thrusters', measured)
                    survey[(axis, sign, level)] = (windows[-1], *cross_response(delta, axis, sign))
                    progress(runner, label(axis, sign), 1)
        columns, refined = {}, {}
        # Dead-response gate: a direction whose a_hi hold moved the boat less than the floor is
        # neither refined nor fitted; its response is noise and its cross ratios are meaningless.
        # A single direction has no survey: its first a_hi hold, at the deployed column, is the gate.
        dead = [] if only else [(axis, sign) for axis, sign in DIRECTIONS
                                if survey[(axis, sign, hi)][1] < cfg['min_response'][AXES[axis]]]
        for axis, sign in ([only] if only else DIRECTIONS):
            begin = deployed_column(axis, sign)  # the survey drove it too: its response seeds the search
            if only:
                main, ratios = cross_response(drive(runner, command_for(axis, sign, hi, begin),
                                                    settings.command, label(axis, sign)), axis, sign)
                if main < cfg['min_response'][AXES[axis]]:
                    dead.append(only)
            if (axis, sign) in dead:
                columns[(axis, sign)] = begin
                gate = (measured[-1] if only else
                        next((m for m in measured if m['label'] == label(axis, sign)), {}))
                refined[label(axis, sign)] = dict(
                    column=begin, converged=False, dead=True,
                    why=f"main response {(gate.get('response') or [0, 0, 0])[axis] * sign:+.3f} under "
                        f"{cfg['min_response'][AXES[axis]]:g} ({said(gate)})")
                progress(runner, label(axis, sign) + ' dead', 1)
                continue
            known = [(begin, ratios if only else survey[(axis, sign, hi)][2])]

            driven = []  # the refine holds actually run

            def measure(column, axis=axis, sign=sign):
                driven.append(column)
                delta = drive(runner, command_for(axis, sign, hi, column), settings.command,
                              label(axis, sign) + ' refine')
                return cross_response(delta, axis, sign)[1]

            found = refine(measure, begin, tolerance[AXES[axis]], cfg['refine']['probe'],
                           room,
                           cfg['refine']['max_holds'], known)
            if not driven and (axis, sign, lo) in survey:  # the survey point passed: its a_lo hold is the check
                low = survey[(axis, sign, lo)][2]
            else:  # homogeneity check: the final column at the low amplitude
                low = cross_response(drive(runner, command_for(axis, sign, lo, found['column']),
                                           settings.command, label(axis, sign) + ' low'), axis, sign)[1]
            found['low_response'] = low
            found['low_ok'] = worst(low, tolerance[AXES[axis]]) <= 1.0
            columns[(axis, sign)], refined[label(axis, sign)] = found['column'], found
            progress(runner, label(axis, sign) + ' done', 1)
        end = runner.hold(NEUTRAL, settings.settle, 'settle')[1]

    plus, minus = matrices(columns, base if only else None)
    summary = dict(procedure='characterization', directions={
        name: {key: (list(value) if isinstance(value, tuple) else value)
               for key, value in found.items()} for name, found in refined.items()}, holds=holds,
                   measured=measured)
    if water:
        summary['conditions'] = water
    if windows:  # the timeline's run window and one contiguous slice per command
        wall = ctx.services.get('wall_ns', lambda t: int(t * 1e9))
        summary.update(window=[wall(windows[0][1]), wall(end)], slices=slices(wall, windows, end))
    result = profile = None
    if only is None:
        # Holds touched by an override (the interrupted one included) never reach the fit.
        windows = [w for w in windows if not runner.invalid(w[1], w[2])]
        driven = windows    # every hold that ran, for the coast that follows a rejected one
        history = ctx.history(start[0], end)
        audit = audit_holds(history, windows, transient)
        windows = [w for w, hold in zip(windows, audit) if hold['ok']]
        rejected = [hold for hold in audit if not hold['ok']]
        bad = {(h['start'], h['end']) for h in rejected}
        # A dead direction keeps its old curve (if any) instead of a fit of noise.
        old = (ctx.services.get('hardware_config') or {}).get('characterization') or {}
        curves = {}
        for axis, sign in DIRECTIONS:
            if (axis, sign) in dead:
                if label(axis, sign) in (old.get('speed_curves') or {}):
                    curves[label(axis, sign)] = old['speed_curves'][label(axis, sign)]
                continue
            points = [(survey[(axis, sign, level)][1], level) for level in (hi, lo)
                      if (survey[(axis, sign, level)][0][1:]) not in bad]
            curve = speed_curve(points)
            if curve:
                curves[label(axis, sign)] = dict(d1=curve['d1'], d2=curve['d2'])
        t95 = {}    # the first (a_hi, +) hold of each axis: the boat's lag, read by the tracking predictor
        for axis in range(3):
            hold = survey[(axis, 1, hi)][0]
            if hold[1:] not in bad:
                lag = hold_t95([s for s in history if s[0] >= hold[1] - settings.settle], hold,
                               ('vx', 'vy', 'wz')[axis])
                if lag is not None:
                    t95[AXES[axis]] = lag
        result = dict(
            version=2, source='mhseals_testing session', identified_at=datetime.now(timezone.utc).isoformat(),
            sample_count=len(samples_from_windows(history, windows, transient)),
            speed_curves=curves, t95=t95,
            wave_floor=neutral_noise(history, windows, transient, cfg['coast_s'], t95, driven),
            motion_limits=motion_limits(samples_from_windows(history, windows, transient),
                                        samples_from_windows(history, windows, 0.0), [hi, hi, hi]))
        # A direction whose refine did not converge (a dead one included) or whose low-amplitude
        # check failed is not fitted.
        weak = [name for name, found in refined.items() if not found['converged'] or not found['low_ok']]
        summary.update(verdict='PARTIAL' if rejected or dead or weak else 'FITTED', holds_rejected=rejected,
                       directions_unfitted=weak, dead_directions=[label(*d) for d in dead],
                       velocity_limit=result['motion_limits']['velocity'])
        nav2_params, overlay = ctx.services.get('nav2_params'), ctx.services.get('nav2_overlay')
        if nav2_params and overlay:
            # Only measured keys; navigation.launch.py merges these with current defaults.
            profile = nav2_profile(result, nav2_params, rules=cfg['nav2_rules'])
            overlay.write_text(yaml.safe_dump(profile, sort_keys=False))
            summary['nav2_overlay'] = str(overlay)
    else:
        summary['verdict'] = 'FITTED' if refined[label(*only)]['converged'] else 'PARTIAL'
        if dead:
            summary['dead_directions'] = [label(*only)]
    agent = ctx.services.get('agent')    # an agent's run: nobody need be at the keys, so it never asks
    if agent:
        summary['save'] = save_line(plus, minus, agent)
    ctx.record(dict(summary, characterization=result) if result else summary)
    choice = 'no' if agent else ctx.choose('Save the characterization to the hardware config?',
                                           ('no', 'save'), {'n': 0, 's': 1})
    if choice == 'no':
        return result or summary
    fit = calibration.new_fit()     # hardware.yaml keeps {id, time}; the rest goes to calibration/current.json
    kept = (result or load_config(staging.config).get('characterization') or {})
    staging.commit(plus, minus, characterization={**kept, 'fit': {**kept.get('fit', {}),
                                                                  **{name: dict(fit) for name in refined}}})
    history, warn = ctx.services.get('history'), ctx.services.get('warn')
    if history:
        adapters.record_file(history, 'hardware', 'characterize', staging.config, warn)
    _record_calibration(ctx, refined, fit, water, staging.config, history)
    saved = dict(procedure='characterization', verdict='SAVED')
    picked = max((d for d in columns if d not in dead), default=None, key=lambda d: sum(abs(c) for c in columns[d]))
    if picked and any(columns[picked]):
        try:
            saved['verify'] = _verify(ctx, cfg, picked, columns[picked], lo, tolerance)
        except UnusableResponse as error:
            saved['verify'] = dict(direction=label(*picked), ok=False, reason=str(error))
    ctx.record(saved)
    if profile and ctx.services.get('nav2_user'):
        # The lasting overlay the Navigation component launches with.
        ctx.record(dict(procedure='characterization', verdict='SAVED',
                        path=str(nav2_limits.save(profile, ctx.services['nav2_user']))))
    return result or summary


def save_line(plus, minus, agent):
    """The answer an agent's run gets instead of the save question: not saved, and the ``boat_agent set`` line
    (agent/ask.py) that saves this fit's C+ and C- under the agent's claim."""
    words = ' '.join(shlex.quote(f'{key}={ask.show(value)}')
                     for key, value in (('decoupling', plus), ('decoupling_negative', minus)))
    why = shlex.quote('save the characterization fit')
    return (f"not saved (an agent's run never saves by itself); to save this fit: "
            f"boat_agent set {words} --claim {agent.get('claim') or 'ID'} --why {why}")


def _record_calibration(ctx, refined, fit, water, config, history):
    """Store the fit's provenance in calibration/current.json. Best effort: a failure is a warning, never a lost save."""
    services = ctx.services
    try:
        context = dict((services.get('fit_context') or dict)())
        head = history.head('hardware') if history else None
        calibration.record_fit(list(refined), fit, load_config(config), path=services.get('calibration_path'),
                               context=dict(context, config=head['id'] if head else None), conditions=water,
                               dead=[name for name, found in refined.items() if found.get('dead')])
    except Exception as error:
        if services.get('warn'):
            services['warn'](f'Calibration provenance could not be recorded: {error}')


def _verify(ctx, cfg, direction, column, amplitude, tolerance):
    """One hold through the driver's own path (it now adds C+/C-): the saved correction must work."""
    axis, sign = direction
    settings = ctx.settings
    with ctx.motion('characterization_verify', direction=label(*direction)) as runner:
        runner.hold(NEUTRAL, cfg['sync_s'], 'sync')
        settle = runner.hold(NEUTRAL, settings.settle, 'settle')
        hold = runner.hold(command_for(axis, sign, amplitude), settings.command, 'verify')
    delta = response(ctx.history(settle[0], hold[1]), settle, hold, cfg['transient_s'], cfg['coast_s'])
    cross = cross_response(delta, axis, sign)[1]
    return dict(direction=label(*direction), response=list(cross),
                ok=worst(cross, tolerance[AXES[axis]]) <= 1.0)
