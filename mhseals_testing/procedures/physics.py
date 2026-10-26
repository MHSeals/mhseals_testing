"""Physics characterization: the dock and open-water steps the crane physics import needs.

Two menu entries, each running the step its setting names (Session rows, persisted):
  Dock physics  (``dock_step``)   weigh_in, then the other dock steps; nothing moves.
  Thrust physics (``thrust_step``) the steps that drive thrusters.
No step changes the correction C+/C-: the driver's C is restart-only, so every motion step runs
the deployed C and records it (with the driver's own report of the file it runs) at fit time.
Each step writes one record (physics_record.py) and records its summary in the session.
"""

from pathlib import Path

from mhseals_testing import physics_record
from mhseals_testing.analysis import physics
from mhseals_testing.strict_yaml import load_strict

DOCK_STEPS = ('weigh_in',)
THRUST_STEPS = ()


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


STEPS = dict(weigh_in=weigh_in)


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
