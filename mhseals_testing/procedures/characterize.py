"""Characterization: one combined survey, then per-direction refinement of the correction columns.

The correction C+/C- is staged to identity first (harness/apply.py), so every hold drives raw
efforts and the driver adds nothing; the survey sends each direction's deployed column itself. The survey holds each axis both ways at two amplitudes: it
gives the speed curves, the Nav2 limits and each direction's uncorrected cross response. Every
direction outside tolerance then gets its column refined (analysis/column.py). The result is
written to the hardware YAML only on the operator's answer, and always after the Nav2 overlay.
``--direction sway+`` skips the survey and re-refines that one column from the deployed one.
Any abort restores the saved C+/C-.
"""

from datetime import datetime, timezone
from pathlib import Path

import yaml

from mhseals_hardware.thruster_mixer import AXES
from mhseals_testing import nav2_limits
from mhseals_testing.analysis.characterization import (
    audit_holds, motion_limits, nav2_profile, samples_from_windows,
)
from mhseals_testing.analysis.column import _within, refine, worst
from mhseals_testing.analysis.thrust import speed_curve
from mhseals_testing.harness.apply import IDENTITY, Staging
from mhseals_testing.history import adapters
from mhseals_testing.runner import NEUTRAL

DIRECTIONS = tuple((axis, sign) for axis in range(3) for sign in (1, -1))


def load_settings(path=None):
    """config/characterize.yaml from the source tree, else the installed share directory."""
    path = Path(path or Path(__file__).resolve().parents[2] / 'config' / 'characterize.yaml')
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'characterize.yaml'
    return yaml.safe_load(path.read_text())


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


def means(history, window, skip):
    """Mean of each velocity component over the window, minus the first ``skip`` seconds."""
    rows = [v for t, v in history if window[0] + skip <= t <= window[1]
            and all(v.get(key) is not None for key in ('vx', 'vy', 'wz'))]
    return [sum(v[key] for v in rows) / len(rows) if rows else 0.0 for key in ('vx', 'vy', 'wz')]


def response(history, settle, hold, transient):
    """Velocity change of a hold over the neutral before it, per component."""
    return [h - s for h, s in zip(means(history, hold, transient), means(history, settle, 0.0))]


def cross_response(delta, axis, sign):
    """(main response along the command, cross responses per unit of it) of a response vector."""
    main = max(sign * delta[axis], 1e-6)
    return main, tuple(delta[k] / main for k in cross_axes(axis))


def matrices(columns, base=None):
    """Flat C+ and C-: identity with each direction's cross entries set from its column."""
    result = {sign: list(base[sign] if base else IDENTITY) for sign in (1, -1)}
    for (axis, sign), column in columns.items():
        for k, value in zip(cross_axes(axis), column):
            result[sign][3 * k + axis] = float(value)
    return result[1], result[-1]


def refuse(ctx, reason):
    refusal = dict(procedure='characterization', verdict='REFUSED', reason=reason)
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
    cfg, settings = load_settings(), ctx.settings
    only = parse_direction(settings.direction)
    if settings.direction and only is None:
        return refuse(ctx, f'unknown direction {settings.direction!r}: use e.g. sway+ or yaw-')
    staging = ctx.services.get('staging') or Staging(ctx.services['user_config'])
    hi, lo = (level * settings.amplitude for level in cfg['levels'])
    if cfg['refine']['headroom'] / hi - 1.0 < cfg['refine']['min_room']:
        return refuse(ctx, f'amplitude {settings.amplitude:g} leaves no room for the correction '
                           f'(a(1 + sum|c|) <= {cfg["refine"]["headroom"]:g}): use a smaller one')
    ctx.prompt(f'Characterization drives {"one direction" if only else "every direction"} for '
               f'{settings.command:g}s holds. Clear all sides and press Enter... ')
    history, warn = ctx.services.get('history'), ctx.services.get('warn')
    if history:     # a hand edit is logged before staging rewrites the file
        adapters.observe_file(history, 'hardware', staging.config, warn)
    with staging.staged():
        return _characterize(ctx, cfg, staging, only, hi, lo)


def _characterize(ctx, cfg, staging, only, hi, lo):
    settings, windows, holds = ctx.settings, [], []
    transient, tolerance = cfg['transient_s'], cfg['tolerance']
    deployed = staging.pending() or {}  # the C+/C- in force before this run
    base = {1: deployed.get('decoupling') or IDENTITY,
            -1: deployed.get('decoupling_negative') or deployed.get('decoupling') or IDENTITY}

    def drive(runner, command, seconds, name):
        """Neutral then ``command``; returns the velocity change. The applied effort is logged."""
        settle = runner.hold(NEUTRAL, settings.settle, 'settle')
        hold = runner.hold(command, seconds, name)
        windows.extend([(NEUTRAL, *settle), (command, *hold)])
        holds.append(dict(command=list(command), start=hold[0], end=hold[1], label=name))
        return response(ctx.history(settle[0], hold[1]), settle, hold, transient)

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
                    if level == hi and sign == 1 and delta[axis] * command[axis] <= 0:
                        runner.stop()
                        return refuse(ctx, f'{AXES[axis]} responds backwards: check wiring, run Thrusters')
                    survey[(axis, sign, level)] = (windows[-1], *cross_response(delta, axis, sign))
        columns, refined = {}, {}
        for axis, sign in ([only] if only else DIRECTIONS):
            begin = deployed_column(axis, sign)  # the survey drove it too: its response seeds the search
            known = () if only else [(begin, survey[(axis, sign, hi)][2])]

            driven = []  # the refine holds actually run

            def measure(column, axis=axis, sign=sign):
                driven.append(column)
                delta = drive(runner, command_for(axis, sign, hi, column), settings.command,
                              label(axis, sign) + ' refine')
                return cross_response(delta, axis, sign)[1]

            found = refine(measure, begin, tolerance[AXES[axis]], cfg['refine']['probe'],
                           room,
                           cfg['refine']['max_holds'] + (0 if known else 1), known)  # no survey: +1 for the start
            if known and not driven:  # the survey point passed: its a_lo hold is the homogeneity check
                low = survey[(axis, sign, lo)][2]
            else:  # homogeneity check: the final column at the low amplitude
                low = cross_response(drive(runner, command_for(axis, sign, lo, found['column']),
                                           settings.command, label(axis, sign) + ' low'), axis, sign)[1]
            found['low_response'] = low
            found['low_ok'] = worst(low, tolerance[AXES[axis]]) <= 1.0
            columns[(axis, sign)], refined[label(axis, sign)] = found['column'], found
        end = runner.hold(NEUTRAL, settings.settle, 'settle')[1]

    plus, minus = matrices(columns, base if only else None)
    summary = dict(procedure='characterization', directions={
        name: {key: (list(value) if isinstance(value, tuple) else value)
               for key, value in found.items()} for name, found in refined.items()}, holds=holds)
    if water:
        summary['conditions'] = water
    if windows:  # the timeline's run window and one contiguous slice per command
        wall = ctx.services.get('wall_ns', lambda t: int(t * 1e9))
        summary.update(window=[wall(windows[0][1]), wall(end)], slices=slices(wall, windows, end))
    result = profile = None
    if only is None:
        # Holds touched by an override (the interrupted one included) never reach the fit.
        windows = [w for w in windows if not runner.invalid(w[1], w[2])]
        history = ctx.history(start[0], end)
        audit = audit_holds(history, windows, transient)
        windows = [w for w, hold in zip(windows, audit) if hold['ok']]
        rejected = [hold for hold in audit if not hold['ok']]
        bad = {(h['start'], h['end']) for h in rejected}
        curves = {}
        for axis, sign in DIRECTIONS:
            points = [(survey[(axis, sign, level)][1], level) for level in (hi, lo)
                      if (survey[(axis, sign, level)][0][1:]) not in bad]
            curve = speed_curve(points)
            if curve:
                curves[label(axis, sign)] = dict(d1=curve['d1'], d2=curve['d2'])
        result = dict(
            version=2, source='mhseals_testing session', identified_at=datetime.now(timezone.utc).isoformat(),
            sample_count=len(samples_from_windows(history, windows, transient)),
            speed_curves=curves,
            motion_limits=motion_limits(samples_from_windows(history, windows, transient),
                                        samples_from_windows(history, windows, 0.0), [hi, hi, hi]))
        summary.update(verdict='PARTIAL' if rejected else 'FITTED', holds_rejected=rejected,
                       velocity_limit=result['motion_limits']['velocity'])
        nav2_params, overlay = ctx.services.get('nav2_params'), ctx.services.get('nav2_overlay')
        if nav2_params and overlay:
            # Only measured keys; navigation.launch.py merges these with current defaults.
            profile = nav2_profile(result, nav2_params)
            overlay.write_text(yaml.safe_dump(profile, sort_keys=False))
            summary['nav2_overlay'] = str(overlay)
    else:
        summary['verdict'] = 'FITTED' if refined[label(*only)]['converged'] else 'PARTIAL'
    ctx.record(dict(summary, characterization=result) if result else summary)
    choice = ctx.choose('Save the characterization to the hardware config?',
                        ('no', 'save'), {'n': 0, 's': 1})
    if choice == 'no':
        return result or summary
    staging.commit(plus, minus, **(dict(characterization=result) if result else {}))
    history, warn = ctx.services.get('history'), ctx.services.get('warn')
    if history:
        adapters.record_file(history, 'hardware', 'characterize', staging.config, warn)
    saved = dict(procedure='characterization', verdict='SAVED')
    picked = max(columns, key=lambda d: sum(abs(c) for c in columns[d]))
    if any(columns[picked]):
        saved['verify'] = _verify(ctx, cfg, picked, columns[picked], lo, tolerance)
    ctx.record(saved)
    if profile and ctx.services.get('nav2_user'):
        # The lasting overlay the Navigation component launches with (config_history.md 2.3).
        ctx.record(dict(procedure='characterization', verdict='SAVED',
                        path=str(nav2_limits.save(profile, ctx.services['nav2_user']))))
    return result or summary


def _verify(ctx, cfg, direction, column, amplitude, tolerance):
    """One hold through the driver's own path (it now adds C+/C-): the saved correction must work."""
    axis, sign = direction
    settings = ctx.settings
    with ctx.motion('characterization_verify', direction=label(*direction)) as runner:
        runner.hold(NEUTRAL, cfg['sync_s'], 'sync')
        settle = runner.hold(NEUTRAL, settings.settle, 'settle')
        hold = runner.hold(command_for(axis, sign, amplitude), settings.command, 'verify')
    delta = response(ctx.history(settle[0], hold[1]), settle, hold, cfg['transient_s'])
    cross = cross_response(delta, axis, sign)[1]
    return dict(direction=label(*direction), response=list(cross),
                ok=worst(cross, tolerance[AXES[axis]]) <= 1.0)
